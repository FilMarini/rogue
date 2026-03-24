#-----------------------------------------------------------------------------
# Company    : SLAC National Accelerator Laboratory
#-----------------------------------------------------------------------------
# Description:
#   PyRogue Device for the FPGA RoCEv2 engine AXI-lite register block.
#
#   This Device exposes the raw MetaData TX/RX channel used to configure
#   the FPGA's internal PD / MR / QP resource manager.  It is not meant to
#   be used directly by application code; RoCEv2Server._start() calls the
#   higher-level helpers defined here.
#
#   Register map (from firmware):
#     0xF00 [0]    SendMetaData  RW  Pulse 0→1→0 to send MetaDataTx to engine
#     0xF00 [1]    RecvMetaData  RO  1 = engine has a response ready in MetaDataRx
#     0xF04        MetaDataTx    RW  303-bit request bus
#     0xF2C        MetaDataRx    RO  276-bit response bus
#
#   MetaData bus encoding is defined in BusStructs.py (SLAC BSV library).
#   The high-level send/receive helpers in this class encapsulate that
#   encoding so callers only deal with plain Python integers.
#-----------------------------------------------------------------------------
# This file is part of the rogue software platform. It is subject to
# the license terms in the LICENSE.txt file found in the top-level directory
# of this distribution and at:
#    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
#-----------------------------------------------------------------------------
import time
import random
import pyrogue as pr
from typing import Tuple


# ---------------------------------------------------------------------------
# MetaData bus constants  (mirror of BusStructs.py / BSVSettings.py)
# ---------------------------------------------------------------------------

META_DATA_TX_BITS  = 303
META_DATA_RX_BITS  = 276

# Bus type tags (top 2 bits of the response bus)
METADATA_PD_T  = 0
METADATA_MR_T  = 1
METADATA_QP_T  = 2

# QP request types
REQ_QP_CREATE  = 0
REQ_QP_MODIFY  = 2

# QP types
IBV_QPT_RC     = 2

# QP states
IBV_QPS_INIT   = 1
IBV_QPS_RTR    = 2
IBV_QPS_RTS    = 3

# QP attribute mask bits  (1 << N)
IBV_QP_STATE             = 1
IBV_QP_ACCESS_FLAGS      = 8
IBV_QP_PKEY_INDEX        = 16
IBV_QP_PATH_MTU          = 256
IBV_QP_TIMEOUT           = 512
IBV_QP_RETRY_CNT         = 1024
IBV_QP_RNR_RETRY         = 2048
IBV_QP_RQ_PSN            = 4096
IBV_QP_MAX_QP_RD_ATOMIC  = 8192
IBV_QP_MIN_RNR_TIMER     = 32768
IBV_QP_SQ_PSN            = 65536
IBV_QP_MAX_DEST_RD_ATOMIC= 131072
IBV_QP_DEST_QPN          = 1048576

# PMTU
IBV_MTU_4096   = 5

# Field widths (bits) — from BusStructs.py
_PD_INDEX_B          = 3   # log2(MAX_PD=8)
_PD_ALLOC_OR_NOT_B   = 1
_PD_HANDLER_B        = 32
_PD_KEY_B            = _PD_HANDLER_B - _PD_INDEX_B   # 29

_MR_INDEX_B          = 4   # log2(MAX_MR_PER_PD)
_MR_ALLOC_OR_NOT_B   = 1
_MR_LADDR_B          = 64
_MR_LEN_B            = 32
_MR_ACCFLAGS_B       = 8
_MR_PDHANDLER_B      = 32
_MR_KEY_B            = 32
_MR_LKEYPART_B       = _MR_KEY_B - _MR_INDEX_B       # 28
_MR_RKEYPART_B       = _MR_KEY_B - _MR_INDEX_B       # 28
_MR_LKEYORNOT_B      = 1

_QPI_TYPE_B          = 4
_QPI_SQSIGALL_B      = 1

_QPA_QPSTATE_B       = 4
_QPA_CURRQPSTATE_B   = 4
_QPA_PMTU_B          = 3
_QPA_QKEY_B          = 32
_QPA_RQPSN_B         = 24
_QPA_SQPSN_B         = 24
_QPA_DQPN_B          = 24
_QPA_QPACCFLAGS_B    = 8
_QPA_CAP_B           = 40
_QPA_PKEY_B          = 16
_QPA_SQDRAINING_B    = 1
_QPA_MAXREADATOMIC_B = 8
_QPA_MAXDESTRD_B     = 8
_QPA_RNRTIMER_B      = 5
_QPA_TIMEOUT_B       = 5
_QPA_RETRYCNT_B      = 3
_QPA_RNRRETRY_B      = 3

_QP_REQTYPE_B        = 2
_QP_PDHANDLER_B      = 32
_QP_QPN_B            = 24
_QP_ATTRMASK_B       = 26
_QP_ATTR_B           = 212
_QP_INITATTR_B       = _QPI_TYPE_B + _QPI_SQSIGALL_B  # 5

# Access permissions for the MR the FPGA writes into
_ACC_PERM            = 0x0F   # local_write | remote_write | remote_read | remote_atomic

# Default RC tuning knobs
_DEFAULT_RETRY_NUM   = 3
_DEFAULT_RNR_TIMER   = 1
_DEFAULT_TIMEOUT     = 14    # ~4 seconds, conservative for FPGA links
_MAX_QP_RD_ATOM      = 16
_CAP_VALUE           = 0x2020010100  # from Utils4Test.bsv


# ---------------------------------------------------------------------------
# Helper: pack a list of (value, width) tuples MSB-first into one integer
# ---------------------------------------------------------------------------
def _pack(*fields) -> int:
    """
    Pack fields MSB-first into a single integer.
    Each element of fields is a (value, width_in_bits) tuple.
    The first field occupies the most significant bits.
    """
    result = 0
    for value, width in fields:
        result = (result << width) | (int(value) & ((1 << width) - 1))
    return result


def _extract(bus: int, total_bits: int, msb: int, lsb: int) -> int:
    """Extract bits [msb:lsb] (inclusive) from bus."""
    width = msb - lsb + 1
    return (bus >> lsb) & ((1 << width) - 1)


# ---------------------------------------------------------------------------
# MetaData bus encoders
# ---------------------------------------------------------------------------

def _encode_alloc_pd(pd_key: int) -> int:
    """Build a 303-bit PD allocation request."""
    # Bus layout (MSB first):
    #   busType(2) | allocOrNot(1) | pdKey(PD_KEY_B) | padding to 303
    allocOrNot = 1
    bus_type   = METADATA_PD_T
    payload    = _pack(
        (allocOrNot, _PD_ALLOC_OR_NOT_B),
        (pd_key,     _PD_KEY_B),
    )
    # Pad to META_DATA_TX_BITS - 2 (bus_type occupies top 2 bits)
    inner_bits = _PD_ALLOC_OR_NOT_B + _PD_KEY_B
    padding    = META_DATA_TX_BITS - 2 - inner_bits
    full       = _pack(
        (bus_type, 2),
        (payload,  inner_bits),
        (0,        padding),
    )
    return full


def _encode_alloc_mr(pd_handler: int,
                     laddr:      int,
                     length:     int,
                     lkey_part:  int,
                     rkey_part:  int) -> int:
    """Build a 303-bit MR allocation request."""
    allocOrNot = 1
    lkey_or_not = 0   # let engine assign lkey index
    bus_type   = METADATA_MR_T
    payload    = _pack(
        (allocOrNot,  _MR_ALLOC_OR_NOT_B),
        (laddr,       _MR_LADDR_B),
        (length,      _MR_LEN_B),
        (_ACC_PERM,   _MR_ACCFLAGS_B),
        (pd_handler,  _MR_PDHANDLER_B),
        (lkey_part,   _MR_LKEYPART_B),
        (rkey_part,   _MR_RKEYPART_B),
        (lkey_or_not, _MR_LKEYORNOT_B),
    )
    inner_bits = (_MR_ALLOC_OR_NOT_B + _MR_LADDR_B + _MR_LEN_B +
                  _MR_ACCFLAGS_B + _MR_PDHANDLER_B +
                  _MR_LKEYPART_B + _MR_RKEYPART_B + _MR_LKEYORNOT_B)
    padding    = META_DATA_TX_BITS - 2 - inner_bits
    full       = _pack(
        (bus_type, 2),
        (payload,  inner_bits),
        (0,        padding),
    )
    return full


def _encode_create_qp(pd_handler: int) -> int:
    """Build a 303-bit QP create request (RC, sq_sig_all=0)."""
    bus_type      = METADATA_QP_T
    qp_req_type   = REQ_QP_CREATE
    sq_sig_all    = 0
    payload       = _pack(
        (qp_req_type,  _QP_REQTYPE_B),
        (pd_handler,   _QP_PDHANDLER_B),
        (IBV_QPT_RC,   _QPI_TYPE_B),
        (sq_sig_all,   _QPI_SQSIGALL_B),
    )
    inner_bits = _QP_REQTYPE_B + _QP_PDHANDLER_B + _QPI_TYPE_B + _QPI_SQSIGALL_B
    padding    = META_DATA_TX_BITS - 2 - inner_bits
    full       = _pack(
        (bus_type, 2),
        (payload,  inner_bits),
        (0,        padding),
    )
    return full


def _make_qp_attr(qp_state:  int,
                  pmtu:      int   = IBV_MTU_4096,
                  dqpn:      int   = 0,
                  rq_psn:    int   = 0,
                  sq_psn:    int   = 0) -> int:
    """Build a _QP_ATTR_B-wide QP attribute field."""
    return _pack(
        (qp_state,          _QPA_QPSTATE_B),
        (0,                 _QPA_CURRQPSTATE_B),   # don't-care
        (pmtu,              _QPA_PMTU_B),
        (0,                 _QPA_QKEY_B),           # RC doesn't use qkey
        (rq_psn,            _QPA_RQPSN_B),
        (sq_psn,            _QPA_SQPSN_B),
        (dqpn,              _QPA_DQPN_B),
        (0x0E,              _QPA_QPACCFLAGS_B),     # remote_write|read|atomic
        (_CAP_VALUE,        _QPA_CAP_B),
        (0xFFFF,            _QPA_PKEY_B),
        (0,                 _QPA_SQDRAINING_B),
        (_MAX_QP_RD_ATOM,   _QPA_MAXREADATOMIC_B),
        (_MAX_QP_RD_ATOM,   _QPA_MAXDESTRD_B),
        (_DEFAULT_RNR_TIMER,_QPA_RNRTIMER_B),
        (_DEFAULT_TIMEOUT,  _QPA_TIMEOUT_B),
        (_DEFAULT_RETRY_NUM,_QPA_RETRYCNT_B),
        (_DEFAULT_RETRY_NUM,_QPA_RNRRETRY_B),
    )


def _encode_modify_qp(qpn:       int,
                      attr_mask: int,
                      qp_state:  int,
                      pmtu:      int = IBV_MTU_4096,
                      dqpn:      int = 0,
                      rq_psn:    int = 0,
                      sq_psn:    int = 0) -> int:
    """Build a 303-bit QP modify request."""
    bus_type    = METADATA_QP_T
    qp_req_type = REQ_QP_MODIFY
    qp_attr     = _make_qp_attr(qp_state, pmtu, dqpn, rq_psn, sq_psn)
    payload     = _pack(
        (qp_req_type, _QP_REQTYPE_B),
        (0,           _QP_PDHANDLER_B),   # not used for modify
        (qpn,         _QP_QPN_B),
        (attr_mask,   _QP_ATTRMASK_B),
        (qp_attr,     _QP_ATTR_B),
    )
    inner_bits = (_QP_REQTYPE_B + _QP_PDHANDLER_B +
                  _QP_QPN_B + _QP_ATTRMASK_B + _QP_ATTR_B)
    padding    = META_DATA_TX_BITS - 2 - inner_bits
    full       = _pack(
        (bus_type, 2),
        (payload,  inner_bits),
        (0,        padding),
    )
    return full


# ---------------------------------------------------------------------------
# MetaData response decoders
# ---------------------------------------------------------------------------

def _decode_resp_type(rx: int) -> int:
    """Return the 2-bit bus type from the MSB of the RX bus."""
    return (rx >> (META_DATA_RX_BITS - 2)) & 0x3


def _decode_pd_resp(rx: int) -> Tuple[bool, int]:
    """Return (success, pd_handler) from a PD response bus."""
    # Layout (MSB first after bus_type):
    #   successOrNot(1) | pdHandler(PD_HANDLER_B) | ...
    success    = bool((rx >> (META_DATA_RX_BITS - 3)) & 1)
    pd_handler = (rx >> (META_DATA_RX_BITS - 3 - _PD_HANDLER_B)) & ((1 << _PD_HANDLER_B) - 1)
    return success, pd_handler


def _decode_mr_resp(rx: int) -> Tuple[bool, int]:
    """Return (success, lkey) from an MR response bus."""
    success = bool((rx >> (META_DATA_RX_BITS - 3)) & 1)
    lkey    = (rx >> (META_DATA_RX_BITS - 3 - _MR_LKEY_B)) & ((1 << _MR_KEY_B) - 1)
    return success, lkey


# Field widths inside the QP response payload (after bus_type + success)
_MR_LKEY_B = _MR_KEY_B   # alias

def _decode_qp_resp(rx: int) -> Tuple[bool, int, int]:
    """Return (success, qpn, qp_state) from a QP response bus."""
    success   = bool((rx >> (META_DATA_RX_BITS - 3)) & 1)
    qpn       = (rx >> (META_DATA_RX_BITS - 3 - _QP_QPN_B))    & ((1 << _QP_QPN_B) - 1)
    qp_state  = (rx >> (META_DATA_RX_BITS - 3 - _QP_QPN_B - _QPA_QPSTATE_B)) & 0xF
    return success, qpn, qp_state


# ---------------------------------------------------------------------------
# pyrogue Device
# ---------------------------------------------------------------------------

class RoceEngine(pr.Device):
    """
    PyRogue Device for the FPGA RoCEv2 AXI-lite register block.

    Exposes the raw MetaData TX/RX channel and provides high-level helpers
    (_setup_connection) that are called by RoCEv2Server during _start().

    Parameters
    ----------
    **kwargs
        Forwarded to pr.Device.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        # ------------------------------------------------------------------
        # Raw AXI-lite registers (match firmware offsets exactly)
        # ------------------------------------------------------------------
        self.add(pr.RemoteVariable(
            name        = 'SendMetaData',
            description = 'Pulse 0→1→0 to send MetaDataTx to the RoCEv2 engine',
            offset      = 0xF00,
            bitSize     = 1,
            bitOffset   = 0,
            mode        = 'RW',
            hidden      = True,
        ))

        self.add(pr.RemoteVariable(
            name        = 'RecvMetaData',
            description = '1 = engine has a response ready in MetaDataRx',
            offset      = 0xF00,
            bitSize     = 1,
            bitOffset   = 1,
            mode        = 'RO',
            hidden      = True,
        ))

        self.add(pr.RemoteVariable(
            name        = 'MetaDataTx',
            description = '303-bit request bus to RoCEv2 engine',
            offset      = 0xF04,
            bitSize     = 303,
            mode        = 'RW',
            hidden      = True,
        ))

        self.add(pr.RemoteVariable(
            name        = 'MetaDataRx',
            description = '276-bit response bus from RoCEv2 engine',
            offset      = 0xF2C,
            bitSize     = 276,
            mode        = 'RO',
            hidden      = True,
        ))

        # ------------------------------------------------------------------
        # Status variables (populated by _setup_connection)
        # ------------------------------------------------------------------
        self.add(pr.LocalVariable(
            name        = 'FpgaQpn',
            description = "FPGA's QP number assigned by the RoCEv2 engine",
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt32',
        ))

        self.add(pr.LocalVariable(
            name        = 'FpgaPdHandler',
            description = "FPGA's PD handler",
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt32',
            hidden      = True,
        ))

        self.add(pr.LocalVariable(
            name        = 'FpgaLKey',
            description = "FPGA MR local key",
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt32',
            hidden      = True,
        ))

        self.add(pr.LocalVariable(
            name        = 'ConnectionState',
            description = 'Human-readable RC connection state',
            mode        = 'RO',
            value       = 'Disconnected',
        ))

    # ------------------------------------------------------------------
    # Low-level send / receive helpers
    # ------------------------------------------------------------------

    def _send_meta(self, bus_value: int) -> None:
        """Write bus_value into MetaDataTx then pulse SendMetaData."""
        self.MetaDataTx.set(bus_value)
        self.SendMetaData.set(0)
        self.SendMetaData.set(1)
        self.SendMetaData.set(0)

    def _wait_resp(self, timeout_s: float = 5.0) -> int:
        """
        Poll RecvMetaData until the engine signals a response, then
        read and return the MetaDataRx integer value.

        Raises RuntimeError on timeout.
        """
        deadline = time.monotonic() + timeout_s
        while self.RecvMetaData.get() != 1:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"RoceEngine '{self.name}': timeout waiting for MetaData response")
            time.sleep(0.05)
        return self.MetaDataRx.get()

    # ------------------------------------------------------------------
    # High-level connection setup  (called by RoCEv2Server._start)
    # ------------------------------------------------------------------

    def setup_connection(self,
                         host_qpn:   int,
                         host_rq_psn: int,
                         host_sq_psn: int,
                         mr_laddr:   int,
                         mr_len:     int,
                         pmtu:       int = IBV_MTU_4096) -> int:
        """
        Drive the FPGA RoCEv2 engine through the full connection sequence:
            1. Allocate PD
            2. Allocate MR  (pointing at the host-registered memory region)
            3. Create QP    (RC)
            4. QP → INIT
            5. QP → RTR     (using host QPN / PSN)
            6. QP → RTS

        Parameters
        ----------
        host_qpn    : Host RC QP number (from ibv_create_qp on the host side)
        host_rq_psn : Expected starting receive PSN on FPGA side (= host sq_psn)
        host_sq_psn : FPGA's own SQ PSN (= host rq_psn)
        mr_laddr    : Virtual address of host MR (from ibv_reg_mr)
        mr_len      : Length of host MR in bytes
        pmtu        : Path MTU enum value (default IBV_MTU_4096)

        Returns
        -------
        int : FPGA QP number (needed to complete the host-side RTR transition)
        """
        self.ConnectionState.set('Configuring')
        self._log.info("RoceEngine: starting connection setup sequence")

        # ---- 1. Allocate PD -------------------------------------------
        pd_key = random.getrandbits(_PD_KEY_B)
        self._send_meta(_encode_alloc_pd(pd_key))
        rx = self._wait_resp()

        assert _decode_resp_type(rx) == METADATA_PD_T, \
            f"Expected PD response (type {METADATA_PD_T}), got {_decode_resp_type(rx)}"
        ok, pd_handler = _decode_pd_resp(rx)
        assert ok, "FPGA PD allocation failed"
        self.FpgaPdHandler.set(pd_handler)
        self._log.info(f"RoceEngine: PD allocated, handler=0x{pd_handler:08x}")

        # ---- 2. Allocate MR -------------------------------------------
        lkey_part = random.getrandbits(_MR_LKEYPART_B)
        rkey_part = random.getrandbits(_MR_RKEYPART_B)
        self._send_meta(_encode_alloc_mr(
            pd_handler = pd_handler,
            laddr      = mr_laddr,
            length     = mr_len,
            lkey_part  = lkey_part,
            rkey_part  = rkey_part,
        ))
        rx = self._wait_resp()

        assert _decode_resp_type(rx) == METADATA_MR_T, \
            f"Expected MR response (type {METADATA_MR_T}), got {_decode_resp_type(rx)}"
        ok, lkey = _decode_mr_resp(rx)
        assert ok, "FPGA MR allocation failed"
        self.FpgaLKey.set(lkey)
        self._log.info(f"RoceEngine: MR allocated, lkey=0x{lkey:08x}")

        # ---- 3. Create QP (RC) ----------------------------------------
        self._send_meta(_encode_create_qp(pd_handler))
        rx = self._wait_resp()

        assert _decode_resp_type(rx) == METADATA_QP_T, \
            f"Expected QP response (type {METADATA_QP_T}), got {_decode_resp_type(rx)}"
        ok, fpga_qpn, _ = _decode_qp_resp(rx)
        assert ok, "FPGA QP creation failed"
        self.FpgaQpn.set(fpga_qpn)
        self._log.info(f"RoceEngine: QP created, fpga_qpn=0x{fpga_qpn:06x}")

        # ---- 4. QP → INIT ---------------------------------------------
        init_mask = IBV_QP_STATE | IBV_QP_PKEY_INDEX | IBV_QP_ACCESS_FLAGS
        self._send_meta(_encode_modify_qp(
            qpn       = fpga_qpn,
            attr_mask = init_mask,
            qp_state  = IBV_QPS_INIT,
            pmtu      = pmtu,
        ))
        rx = self._wait_resp()
        ok, _, state = _decode_qp_resp(rx)
        assert ok and state == IBV_QPS_INIT, \
            f"FPGA QP→INIT failed (state={state})"
        self._log.info("RoceEngine: QP → INIT")

        # ---- 5. QP → RTR  (needs host QPN and PSN) --------------------
        rtr_mask = (IBV_QP_STATE | IBV_QP_PATH_MTU | IBV_QP_DEST_QPN |
                    IBV_QP_RQ_PSN | IBV_QP_MAX_DEST_RD_ATOMIC | IBV_QP_MIN_RNR_TIMER)
        self._send_meta(_encode_modify_qp(
            qpn       = fpga_qpn,
            attr_mask = rtr_mask,
            qp_state  = IBV_QPS_RTR,
            pmtu      = pmtu,
            dqpn      = host_qpn,
            rq_psn    = host_rq_psn,
        ))
        rx = self._wait_resp()
        ok, _, state = _decode_qp_resp(rx)
        assert ok and state == IBV_QPS_RTR, \
            f"FPGA QP→RTR failed (state={state})"
        self._log.info(
            f"RoceEngine: QP → RTR  (targeting host qpn=0x{host_qpn:06x})")

        # ---- 6. QP → RTS ----------------------------------------------
        rts_mask = (IBV_QP_STATE | IBV_QP_SQ_PSN | IBV_QP_TIMEOUT |
                    IBV_QP_RETRY_CNT | IBV_QP_RNR_RETRY | IBV_QP_MAX_QP_RD_ATOMIC)
        self._send_meta(_encode_modify_qp(
            qpn       = fpga_qpn,
            attr_mask = rts_mask,
            qp_state  = IBV_QPS_RTS,
            pmtu      = pmtu,
            sq_psn    = host_sq_psn,
        ))
        rx = self._wait_resp()
        ok, _, state = _decode_qp_resp(rx)
        assert ok and state == IBV_QPS_RTS, \
            f"FPGA QP→RTS failed (state={state})"
        self._log.info("RoceEngine: QP → RTS  — FPGA ready to send RDMA WRITEs")

        self.ConnectionState.set('Connected')
        return fpga_qpn
