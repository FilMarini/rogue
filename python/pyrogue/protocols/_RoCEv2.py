#-----------------------------------------------------------------------------
# Company    : SLAC National Accelerator Laboratory
#-----------------------------------------------------------------------------
# Description:
#   PyRogue Device wrapper for the RoCEv2 RC receive server.
#
#   Accepts the same `ip` parameter as the existing UdpRssiPack / Root so
#   the two can be used interchangeably.  The FPGA GID is derived
#   deterministically from the IP address using the IPv4-mapped IPv6 format:
#
#       192.168.2.10  →  0000:0000:0000:0000:0000:ffff:c0a8:020a
#
#   Connection sequence (orchestrated in _start):
#
#     Host side (C++ Server)                FPGA side (RoceEngine / AXI-lite)
#     ──────────────────────                ──────────────────────────────────
#     ibv_reg_mr  → mrAddr, mrRkey  ──────→ used in FPGA MR alloc
#     ibv_create_qp → hostQpn       ──────→ used as dqpn in FPGA QP→RTR
#     ibv_query_gid → hostGid
#                                   ←────── fpgaQpn (from QP-create response)
#     setFpgaGid(gid from ip)
#     completeConnection(fpgaQpn)
#     RX thread starts               ←────── FPGA QP → RTS, WRITEs begin
#
#   Metadata bus encoding
#   ─────────────────────
#   All TX bus encoding mirrors BusStructs.py getBus() exactly:
#     - Fields appended MSB-first into metaBus
#     - fullBus = zeros(303 - len(metaBus)) + metaBus  (right-aligned)
#     - Top 2 bits overwritten with busType
#
#   All RX bus decoding mirrors BusStructs.py slice_vec/get_bool:
#     - bit 0 = LSB of the 276-bit response bus
#     - Fields extracted using (rx >> lsb) & mask
#
#   Constants from BSVSettings.py (firmware-specific):
#     MAX_PD = 1  → PD_INDEX_B = 0  → PD_KEY_B = 32
#     MAX_MR = 2  → MR_INDEX_B = 1  → MR_LKEYPART_B = MR_RKEYPART_B = 31
#-----------------------------------------------------------------------------
import socket
import struct
import random as _random
import time as _time
from bitstring import BitStream as _BS
from typing import Any

import pyrogue as pr
import rogue.protocols.rocev2
import rogue.interfaces.stream


# ---------------------------------------------------------------------------
# GID helpers
# ---------------------------------------------------------------------------

def _ip_to_gid_bytes(ip: str) -> bytes:
    """Convert IPv4 string to 16-byte IPv4-mapped IPv6 GID."""
    return b'\x00' * 10 + b'\xff\xff' + socket.inet_aton(ip)


def _gid_bytes_to_str(gid: bytes) -> str:
    """Format 16 GID bytes as colon-separated hex (ibv_devinfo format)."""
    words = struct.unpack('>8H', gid)
    return ':'.join(f'{w:04x}' for w in words)


# ---------------------------------------------------------------------------
# Constants — must match BSVSettings.py / BusStructs.py in the firmware repo
# ---------------------------------------------------------------------------

_META_DATA_TX_BITS   = 303
_META_DATA_RX_BITS   = 276   # informational only; actual bus may be shorter

# Bus type tags
_METADATA_PD_T = 0
_METADATA_MR_T = 1
_METADATA_QP_T = 2

# QP request types
_REQ_QP_CREATE = 0
_REQ_QP_MODIFY = 2

# QP types / states
_IBV_QPT_RC  = 2
_IBV_QPS_INIT = 1
_IBV_QPS_RTR  = 2
_IBV_QPS_RTS  = 3

# QP attribute mask bits
_IBV_QP_STATE             = 1
_IBV_QP_ACCESS_FLAGS      = 8
_IBV_QP_PKEY_INDEX        = 16
_IBV_QP_PATH_MTU          = 256
_IBV_QP_TIMEOUT           = 512
_IBV_QP_RETRY_CNT         = 1024
_IBV_QP_RNR_RETRY         = 2048
_IBV_QP_RQ_PSN            = 4096
_IBV_QP_MAX_QP_RD_ATOMIC  = 8192
_IBV_QP_MIN_RNR_TIMER     = 32768
_IBV_QP_SQ_PSN            = 65536
_IBV_QP_MAX_DEST_RD_ATOMIC = 131072
_IBV_QP_DEST_QPN          = 1048576

# Field widths — derived from BSVSettings.py with MAX_PD=1, MAX_MR=2
_PD_ALLOC_OR_NOT_B = 1
_PD_INDEX_B        = 0    # int(log2(MAX_PD=1)) = 0
_PD_HANDLER_B      = 32
_PD_KEY_B          = 32   # PD_HANDLER_B - PD_INDEX_B = 32

_MR_ALLOC_OR_NOT_B = 1
_MR_INDEX_B        = 1    # int(log2(MAX_MR/MAX_PD = 2)) = 1
_MR_LADDR_B        = 64
_MR_LEN_B          = 32
_MR_ACCFLAGS_B     = 8
_MR_PDHANDLER_B    = 32
_MR_KEY_B          = 32
_MR_LKEYPART_B     = 31   # MR_KEY_B - MR_INDEX_B = 31
_MR_RKEYPART_B     = 31   # MR_KEY_B - MR_INDEX_B = 31
_MR_LKEYORNOT_B    = 1

_QPI_TYPE_B        = 4
_QPI_SQSIGALL_B    = 1

_QPA_QPSTATE_B     = 4
_QPA_CURRQPSTATE_B = 4
_QPA_PMTU_B        = 3
_QPA_QKEY_B        = 32
_QPA_RQPSN_B       = 24
_QPA_SQPSN_B       = 24
_QPA_DQPN_B        = 24
_QPA_QPACCFLAGS_B  = 8
_QPA_CAP_B         = 40
_QPA_PKEY_B        = 16
_QPA_SQDRAINING_B  = 1
_QPA_MAXREADATOMIC_B  = 8
_QPA_MAXDESTRD_B      = 8
_QPA_RNRTIMER_B    = 5
_QPA_TIMEOUT_B     = 5
_QPA_RETRYCNT_B    = 3
_QPA_RNRRETRY_B    = 3

_QP_REQTYPE_B      = 2
_QP_PDHANDLER_B    = 32
_QP_QPN_B          = 24
_QP_ATTRMASK_B     = 26

# Default QP tuning values
_ACC_PERM          = 0x0F   # local_write | remote_write | remote_read | remote_atomic
_DEFAULT_RETRY_NUM = 3
_DEFAULT_RNR_TIMER = 1
_DEFAULT_TIMEOUT   = 14
_MAX_QP_RD_ATOM    = 16
_CAP_VALUE         = 0x2020010100   # from Utils4Test.bsv


# ---------------------------------------------------------------------------
# TX bus encoders — mirror BusStructs.py getBus() exactly
#
# Pattern:
#   metaBus = fields appended MSB-first
#   fullBus = BS(303 - len(metaBus)) + metaBus   ← right-aligned
#   fullBus.overwrite(busType, pos=0)             ← type in top 2 bits
# ---------------------------------------------------------------------------

def _mk_bus(bus_type, *fields):
    """Build a 303-bit TX metadata bus integer from (value, width) fields."""
    meta = _BS()
    for value, width in fields:
        meta.append(_BS(uint=int(value), length=width))
    full = _BS(_META_DATA_TX_BITS - meta.length)
    full.append(meta)
    full.overwrite(_BS(uint=bus_type, length=2), pos=0)
    return full.uint


def _encode_alloc_pd(pd_key):
    """reqPd.getBus(): allocOrNot(1) + pdKey(32) + pdHandler(32)"""
    return _mk_bus(_METADATA_PD_T,
        (1,      _PD_ALLOC_OR_NOT_B),
        (pd_key, _PD_KEY_B),
        (0,      _PD_HANDLER_B),       # pdHandler don't-care in request
    )


def _encode_alloc_mr(pd_handler, laddr, length, lkey_part, rkey_part):
    """reqMr.getBus(): allocOrNot + mrLAddr + mrLen + mrAccFlags + mrPdHandler
                       + mrLKeyPart + mrRKeyPart + lKeyOrNot + lKey + rKey"""
    return _mk_bus(_METADATA_MR_T,
        (1,           _MR_ALLOC_OR_NOT_B),
        (laddr,       _MR_LADDR_B),
        (length,      _MR_LEN_B),
        (_ACC_PERM,   _MR_ACCFLAGS_B),
        (pd_handler,  _MR_PDHANDLER_B),
        (lkey_part,   _MR_LKEYPART_B),
        (rkey_part,   _MR_RKEYPART_B),
        (0,           _MR_LKEYORNOT_B),
        (0,           _MR_KEY_B),       # lKey don't-care
        (0,           _MR_KEY_B),       # rKey don't-care
    )


def _encode_create_qp(pd_handler):
    """reqQp.getBus() for CREATE: all QPA fields + qpiType + qpiSqSigAll"""
    meta = _BS()
    meta.append(_BS(uint=_REQ_QP_CREATE, length=_QP_REQTYPE_B))
    meta.append(_BS(uint=pd_handler,     length=_QP_PDHANDLER_B))
    meta.append(_BS(uint=0,              length=_QP_QPN_B))
    meta.append(_BS(uint=0,              length=_QP_ATTRMASK_B))
    # All QPA fields don't-care for CREATE
    for w in [_QPA_QPSTATE_B, _QPA_CURRQPSTATE_B, _QPA_PMTU_B,
              _QPA_QKEY_B, _QPA_RQPSN_B, _QPA_SQPSN_B, _QPA_DQPN_B,
              _QPA_QPACCFLAGS_B, _QPA_CAP_B, _QPA_PKEY_B,
              _QPA_SQDRAINING_B, _QPA_MAXREADATOMIC_B, _QPA_MAXDESTRD_B,
              _QPA_RNRTIMER_B, _QPA_TIMEOUT_B, _QPA_RETRYCNT_B, _QPA_RNRRETRY_B]:
        meta.append(_BS(uint=0, length=w))
    meta.append(_BS(uint=_IBV_QPT_RC, length=_QPI_TYPE_B))
    meta.append(_BS(uint=0,           length=_QPI_SQSIGALL_B))
    full = _BS(_META_DATA_TX_BITS - meta.length)
    full.append(meta)
    full.overwrite(_BS(uint=_METADATA_QP_T, length=2), pos=0)
    return full.uint


def _encode_modify_qp(qpn, attr_mask, qp_state, pmtu,
                      dqpn=0, rq_psn=0, sq_psn=0):
    """reqQp.getBus() for MODIFY — same field order as CREATE."""
    meta = _BS()
    meta.append(_BS(uint=_REQ_QP_MODIFY, length=_QP_REQTYPE_B))
    meta.append(_BS(uint=0,              length=_QP_PDHANDLER_B))
    meta.append(_BS(uint=qpn,            length=_QP_QPN_B))
    meta.append(_BS(uint=attr_mask,      length=_QP_ATTRMASK_B))
    # QPA fields in reqQp.getBus() order
    for value, width in [
        (qp_state,           _QPA_QPSTATE_B),
        (0,                  _QPA_CURRQPSTATE_B),
        (pmtu,               _QPA_PMTU_B),
        (0,                  _QPA_QKEY_B),
        (rq_psn,             _QPA_RQPSN_B),
        (sq_psn,             _QPA_SQPSN_B),
        (dqpn,               _QPA_DQPN_B),
        (0x0E,               _QPA_QPACCFLAGS_B),
        (_CAP_VALUE,         _QPA_CAP_B),
        (0xFFFF,             _QPA_PKEY_B),
        (0,                  _QPA_SQDRAINING_B),
        (_MAX_QP_RD_ATOM,    _QPA_MAXREADATOMIC_B),
        (_MAX_QP_RD_ATOM,    _QPA_MAXDESTRD_B),
        (_DEFAULT_RNR_TIMER, _QPA_RNRTIMER_B),
        (_DEFAULT_TIMEOUT,   _QPA_TIMEOUT_B),
        (_DEFAULT_RETRY_NUM, _QPA_RETRYCNT_B),
        (_DEFAULT_RETRY_NUM, _QPA_RNRRETRY_B),
    ]:
        meta.append(_BS(uint=int(value), length=width))
    meta.append(_BS(uint=_IBV_QPT_RC, length=_QPI_TYPE_B))
    meta.append(_BS(uint=0,           length=_QPI_SQSIGALL_B))
    full = _BS(_META_DATA_TX_BITS - meta.length)
    full.append(meta)
    full.overwrite(_BS(uint=_METADATA_QP_T, length=2), pos=0)
    return full.uint


# ---------------------------------------------------------------------------
# RX bus decoders — mirror BusStructs.py slice_vec / get_bool exactly
#
# Convention: bit 0 = LSB of the response integer.
# Fields extracted as: (rx >> lsb) & ((1 << width) - 1)
#
# respPd layout (from BusStructs.py):
#   bits [PD_KEY_B-1 : 0]                = pdKey      [31:0]
#   bits [PD_KEY_B+PD_HANDLER_B-1 : 32]  = pdHandler  [63:32]
#   bit  [PD_KEY_B+PD_HANDLER_B]         = successOrNot  bit 64
#   bits [275:274]                        = busType
#
# respMr layout:
#   bits [31:0]   = rKey
#   bits [63:32]  = lKey
#   ...more fields...
#   bit  [success_bit]  = successOrNot
#   bits [275:274]      = busType
#
# respQp layout:
#   bit  273      = successOrNot
#   bits [272:249] = qpn
#   bits [216:213] = qpaQpState
#   bits [275:274] = busType
# ---------------------------------------------------------------------------

def _decode_resp_type(rx):
    """Extract busType from bits [275:274] (LSB convention)."""
    return (rx >> 274) & 0x3


def _decode_pd_resp(rx):
    """
    Decode PD allocation response.
    Returns (success: bool, pd_handler: int).
    """
    success    = bool((rx >> (_PD_KEY_B + _PD_HANDLER_B)) & 1)
    pd_handler = (rx >> _PD_KEY_B) & ((1 << _PD_HANDLER_B) - 1)
    return success, pd_handler


def _decode_mr_resp(rx):
    """
    Decode MR allocation response.
    Returns (success: bool, lkey: int).
    """
    # successOrNot bit position from respMr.__init__:
    # MR_RKEY_B + MR_LKEY_B + MR_RKEYPART_B + MR_LKEYPART_B
    # + MR_PDHANDLER_B + MR_ACCFLAGS_B + MR_LEN_B + MR_LADDR_B
    success_bit = (_MR_KEY_B + _MR_KEY_B + _MR_RKEYPART_B + _MR_LKEYPART_B +
                   _MR_PDHANDLER_B + _MR_ACCFLAGS_B + _MR_LEN_B + _MR_LADDR_B)
    success = bool((rx >> success_bit) & 1)
    lkey    = (rx >> _MR_KEY_B) & ((1 << _MR_KEY_B) - 1)
    return success, lkey


def _decode_qp_resp(rx):
    """
    Decode QP create/modify response.
    Returns (success: bool, qpn: int, qp_state: int).
    From respQp.__init__: successOrNot at bit 273, qpn at [272:249],
    qpaQpState at [216:213].
    """
    success  = bool((rx >> 273) & 1)
    qpn      = (rx >> 249) & ((1 << 24) - 1)
    qp_state = (rx >> 213) & 0xF
    return success, qpn, qp_state


# ---------------------------------------------------------------------------
# Metadata bus transport helpers
# ---------------------------------------------------------------------------

def _send_meta(engine, bus_value):
    """
    Send a metadata bus message via any RoceEngine-compatible object.

    The firmware triggers on the RISING EDGE of metaDataIsSet (0→1).
    Writing 0 first ensures a clean rising edge every time.
    """
    engine.SendMetaData.set(0)
    engine.MetaDataTx.set(bus_value)
    engine.SendMetaData.set(1)
    engine.SendMetaData.set(0)


def _wait_resp(engine, timeout_s=5.0):
    """
    Wait for firmware to process the request and return MetaDataRx value.

    The firmware clears metaDataIsReady on rising edge, sets it back when done.
    Processing takes microseconds so the 0 state may be too brief to observe.
    We try to catch the 0 within 200ms, then wait for the 1.
    """
    deadline     = _time.monotonic() + timeout_s
    zero_deadline = _time.monotonic() + 0.2

    # Try to observe RecvMetaData → 0 (firmware started)
    while engine.RecvMetaData.get() != 0:
        if _time.monotonic() > zero_deadline:
            break
        _time.sleep(0.005)

    # Wait for RecvMetaData → 1 (response ready)
    while engine.RecvMetaData.get() != 1:
        if _time.monotonic() > deadline:
            raise RuntimeError(
                'Timeout waiting for RoceEngine response '
                '(RecvMetaData never went to 1)')
        _time.sleep(0.05)

    return engine.MetaDataRx.get()


# ---------------------------------------------------------------------------
# Full FPGA connection sequence
# ---------------------------------------------------------------------------

def _roce_setup_connection(engine, host_qpn, host_rq_psn, host_sq_psn,
                           mr_laddr, mr_len, pmtu, log=None):
    """
    Drive any RoceEngine-compatible object (surf or our own) through:
    PD alloc → MR alloc → QP create → INIT → RTR → RTS.

    Works with both surf.ethernet.roce._RoceEngine.RoceEngine and
    pyrogue.protocols._RoceEngine.RoceEngine since both expose the same
    registers: SendMetaData, MetaDataTx, RecvMetaData, MetaDataRx.

    Returns the FPGA QPN.
    """
    def info(msg):
        if log:
            log.info(msg)

    # Ensure SendMetaData starts at 0 for a clean first rising edge
    engine.SendMetaData.set(0)
    _time.sleep(0.1)

    # 1. Alloc PD
    _send_meta(engine, _encode_alloc_pd(_random.getrandbits(_PD_KEY_B)))
    rx = _wait_resp(engine)
    assert _decode_resp_type(rx) == _METADATA_PD_T, \
        f"Expected PD response (type=0), got type={_decode_resp_type(rx)}"
    ok, pd_handler = _decode_pd_resp(rx)
    assert ok, "FPGA PD allocation failed"
    info(f"RoceEngine: PD allocated handler=0x{pd_handler:08x}")

    # 2. Alloc MR
    _send_meta(engine, _encode_alloc_mr(
        pd_handler = pd_handler,
        laddr      = mr_laddr,
        length     = mr_len,
        lkey_part  = _random.getrandbits(_MR_LKEYPART_B),
        rkey_part  = _random.getrandbits(_MR_RKEYPART_B),
    ))
    rx = _wait_resp(engine)
    assert _decode_resp_type(rx) == _METADATA_MR_T, \
        f"Expected MR response (type=1), got type={_decode_resp_type(rx)}"
    ok, lkey = _decode_mr_resp(rx)
    assert ok, "FPGA MR allocation failed"
    info(f"RoceEngine: MR allocated lkey=0x{lkey:08x}")

    # 3. Create QP (RC)
    _send_meta(engine, _encode_create_qp(pd_handler))
    rx = _wait_resp(engine)
    assert _decode_resp_type(rx) == _METADATA_QP_T, \
        f"Expected QP response (type=2), got type={_decode_resp_type(rx)}"
    ok, fpga_qpn, _ = _decode_qp_resp(rx)
    assert ok, "FPGA QP creation failed"
    info(f"RoceEngine: QP created fpga_qpn=0x{fpga_qpn:06x}")

    # 4. QP → INIT
    init_mask = _IBV_QP_STATE | _IBV_QP_PKEY_INDEX | _IBV_QP_ACCESS_FLAGS
    _send_meta(engine, _encode_modify_qp(fpga_qpn, init_mask, _IBV_QPS_INIT, pmtu))
    rx = _wait_resp(engine)
    ok, _, state = _decode_qp_resp(rx)
    assert ok and state == _IBV_QPS_INIT, \
        f"FPGA QP→INIT failed (ok={ok} state={state})"
    info("RoceEngine: QP → INIT")

    # 5. QP → RTR
    rtr_mask = (_IBV_QP_STATE | _IBV_QP_PATH_MTU | _IBV_QP_DEST_QPN |
                _IBV_QP_RQ_PSN | _IBV_QP_MAX_DEST_RD_ATOMIC | _IBV_QP_MIN_RNR_TIMER)
    _send_meta(engine, _encode_modify_qp(
        fpga_qpn, rtr_mask, _IBV_QPS_RTR, pmtu,
        dqpn=host_qpn, rq_psn=host_rq_psn))
    rx = _wait_resp(engine)
    ok, _, state = _decode_qp_resp(rx)
    assert ok and state == _IBV_QPS_RTR, \
        f"FPGA QP→RTR failed (ok={ok} state={state})"
    info(f"RoceEngine: QP → RTR targeting host qpn=0x{host_qpn:06x}")

    # 6. QP → RTS
    rts_mask = (_IBV_QP_STATE | _IBV_QP_SQ_PSN | _IBV_QP_TIMEOUT |
                _IBV_QP_RETRY_CNT | _IBV_QP_RNR_RETRY | _IBV_QP_MAX_QP_RD_ATOMIC)
    _send_meta(engine, _encode_modify_qp(
        fpga_qpn, rts_mask, _IBV_QPS_RTS, pmtu, sq_psn=host_sq_psn))
    rx = _wait_resp(engine)
    ok, _, state = _decode_qp_resp(rx)
    assert ok and state == _IBV_QPS_RTS, \
        f"FPGA QP→RTS failed (ok={ok} state={state})"
    info("RoceEngine: QP → RTS — FPGA ready to send RDMA WRITEs")

    if log:
        log.info("=" * 60)
        log.info("RoCEv2 FPGA connection summary")
        log.info(f"  FPGA QPN    : 0x{fpga_qpn:06x}")
        log.info(f"  FPGA lkey   : 0x{lkey:08x}")
        log.info(f"  FPGA state  : RTS (ready to send RDMA WRITEs)")
        log.info(f"  Host QPN    : 0x{host_qpn:06x}")
        log.info(f"  Host RQ PSN : 0x{host_rq_psn:06x}")
        log.info(f"  Host SQ PSN : 0x{host_sq_psn:06x}")
        log.info(f"  MR addr     : 0x{mr_laddr:016x}")
        log.info(f"  MR length   : {mr_len} bytes")
        log.info(f"  Path MTU    : {pmtu} ({[256,512,1024,2048,4096][pmtu-1]} bytes)")
        log.info("=" * 60)

    return fpga_qpn, lkey


# ---------------------------------------------------------------------------
# RoCEv2Server — pyrogue Device
# ---------------------------------------------------------------------------

class RoCEv2Server(pr.Device):
    """
    RoCEv2 RC receive server — pyrogue Device.

    Mirrors the interface of UdpRssiPack so the two can be used
    interchangeably inside a Root.

    Parameters
    ----------
    ip : str
        FPGA IP address. Used to derive the FPGA GID (IPv4-mapped IPv6).
    deviceName : str
        ibverbs device name (e.g. 'rxe0' for softRoCE, 'mlx5_0' for HW NIC).
    ibPort : int
        ibverbs port number (default: 1).
    gidIndex : int
        GID table index for the host NIC's RoCEv2 IPv4 address.
    maxPayload : int
        Maximum bytes per RDMA WRITE (default: 9000).
    rxQueueDepth : int
        Number of pre-posted receive slots (default: 256).
    roceEngineOffset : int
        AXI-lite offset of the RoCEv2 engine register block.
    roceMemBase : object
        memBase for the RoceEngine child device (the SRP object).
    roceEngine : object
        Pass an existing RoceEngine instance to avoid duplicate address mapping.
        When set, no child RoceEngine device is created.
    pmtu : int
        Path MTU enum: 1=256 2=512 3=1024 4=2048 5=4096 (default: 5).
    pollInterval : int
        Poll interval in seconds for status variables (default: 1).
    """

    def __init__(
        self,
        *,
        ip:               str,
        deviceName:       str,
        ibPort:           int    = 1,
        gidIndex:         int    = 0,
        maxPayload:       int    = rogue.protocols.rocev2.DefaultMaxPayload,
        rxQueueDepth:     int    = rogue.protocols.rocev2.DefaultRxQueueDepth,
        roceEngineOffset: int    = 0,
        roceMemBase:      object = None,
        roceEngine:       object = None,
        pmtu:             int    = 5,
        pollInterval:     int    = 1,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)

        self._ip            = ip
        self._deviceName    = deviceName
        self._ibPort        = ibPort
        self._gidIndex      = gidIndex
        self._maxPayload    = maxPayload
        self._rxQueueDepth  = rxQueueDepth
        self._pmtu          = pmtu
        self._extRoceEngine = roceEngine
        self._fpgaGidBytes  = _ip_to_gid_bytes(ip)

        # C++ RC server — ibverbs resources up to QP INIT
        self._server = rogue.protocols.rocev2.Server.create(
            deviceName, ibPort, gidIndex, maxPayload, rxQueueDepth)

        # Add RoceEngine child only if no external engine was provided
        if roceEngine is None:
            from surf.ethernet.roce._RoceEngine import RoceEngine as _RE
            self.add(_RE(
                name    = 'RoceEngine',
                offset  = roceEngineOffset,
                memBase = roceMemBase,
            ))

        # Status variables
        self.add(pr.LocalVariable(
            name='FpgaIp', mode='RO', value=ip,
            description='FPGA IP address used for GID derivation'))

        self.add(pr.LocalVariable(
            name='FpgaGid', mode='RO',
            value=_gid_bytes_to_str(self._fpgaGidBytes),
            description='FPGA GID (IPv4-mapped IPv6)'))

        self.add(pr.LocalVariable(
            name='HostQpn', mode='RO', value=0, typeStr='UInt32',
            localGet=lambda: self._server.getQpn(),
            description='Host RC QP number'))

        self.add(pr.LocalVariable(
            name='HostGid', mode='RO', value='',
            localGet=lambda: self._server.getGid(),
            description='Host GID (NIC RoCEv2 address)'))

        self.add(pr.LocalVariable(
            name='MrAddr', mode='RO', value=0, typeStr='UInt64',
            localGet=lambda: self._server.getMrAddr(),
            description='Host MR virtual address (FPGA writes here)'))

        self.add(pr.LocalVariable(
            name='MrRkey', mode='RO', value=0, typeStr='UInt32',
            localGet=lambda: self._server.getMrRkey(),
            description='Host MR rkey (given to FPGA)'))

        self.add(pr.LocalVariable(
            name='RxFrameCount', mode='RO', value=0, typeStr='UInt64',
            localGet=lambda: self._server.getFrameCount(),
            pollInterval=pollInterval,
            description='Total frames received'))

        self.add(pr.LocalVariable(
            name='RxByteCount', mode='RO', value=0, typeStr='UInt64',
            localGet=lambda: self._server.getByteCount(),
            pollInterval=pollInterval,
            description='Total bytes received'))

        self.add(pr.LocalVariable(
            name='ConnectionState', mode='RO', value='Disconnected',
            description='RC connection state'))

        self.add(pr.LocalVariable(
            name        = 'MaxPayload',
            description = 'Max payload bytes per RDMA WRITE slot',
            mode        = 'RO',
            value       = maxPayload,
            typeStr     = 'UInt32',
        ))

        self.add(pr.LocalVariable(
            name        = 'RxQueueDepth',
            description = 'Number of receive slots (rxQueueDepth)',
            mode        = 'RO',
            value       = rxQueueDepth,
            typeStr     = 'UInt32',
        ))

        self.add(pr.LocalVariable(
            name        = 'HostRqPsn',
            description = 'Host starting receive PSN — FPGA SQ PSN must match this',
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt32',
            localGet    = lambda: self._server.getRqPsn(),
        ))

        self.add(pr.LocalVariable(
            name        = 'HostSqPsn',
            description = 'Host starting send PSN — FPGA RQ PSN must match this',
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt32',
            localGet    = lambda: self._server.getSqPsn(),
        ))

        self.add(pr.LocalVariable(
            name        = 'FpgaQpn',
            description = 'FPGA QP number — set after RC connection is established',
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt32',
        ))

        self.add(pr.LocalVariable(
            name        = 'FpgaLkey',
            description = 'FPGA MR local key — set after RC connection is established',
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt32',
        ))

    @property
    def stream(self):
        """Direct access to the C++ stream master (mirrors rudp.application(0))."""
        return self._server

    def getChannel(self, channel: int):
        """Return a channel-filtered view (channel id from immediate value bits [7:0])."""
        filt = rogue.interfaces.stream.Filter(False, channel)
        self._server >> filt
        return filt

    def _start(self) -> None:
        self.ConnectionState.set('Connecting')

        # Push FPGA GID to C++ server
        self._server.setFpgaGid(bytes(self._fpgaGidBytes))

        host_qpn    = self._server.getQpn()
        host_rq_psn = self._server.getRqPsn()
        host_sq_psn = self._server.getSqPsn()
        mr_addr     = self._server.getMrAddr()
        mr_len      = self._maxPayload * self._rxQueueDepth

        self._log.info(
            f"RoCEv2 '{self.name}': QPN=0x{host_qpn:06x} "
            f"GID={self._server.getGid()} "
            f"MR=0x{mr_addr:016x} rkey=0x{self._server.getMrRkey():08x} "
            f"FPGA GID={_gid_bytes_to_str(self._fpgaGidBytes)}"
        )

        _engine  = self._extRoceEngine if self._extRoceEngine is not None \
                   else self.RoceEngine

        fpga_qpn, fpga_lkey = _roce_setup_connection(
            engine      = _engine,
            host_qpn    = host_qpn,
            host_rq_psn = host_rq_psn,
            host_sq_psn = host_sq_psn,
            mr_laddr    = mr_addr,
            mr_len      = mr_len,
            pmtu        = self._pmtu,
            log         = self._log,
        )

        self._server.completeConnection(
            fpgaQpn   = fpga_qpn,
            fpgaRqPsn = host_sq_psn,
            pmtu      = self._pmtu,
        )

        self.FpgaQpn.set(fpga_qpn)
        self.FpgaLkey.set(fpga_lkey)
        self.ConnectionState.set('Connected')
        self._log.info("=" * 60)
        self._log.info("RoCEv2 host RC connection summary")
        self._log.info(f"  Device      : {self._deviceName}  port={self._ibPort}  GID idx={self._gidIndex}")
        self._log.info(f"  Host QPN    : 0x{host_qpn:06x}")
        self._log.info(f"  Host GID    : {self._server.getGid()}")
        self._log.info(f"  Host state  : RTS")
        self._log.info(f"  MR addr     : 0x{mr_addr:016x}")
        self._log.info(f"  MR rkey     : 0x{self._server.getMrRkey():08x}")
        self._log.info(f"  MR size     : {mr_len} bytes  ({self._rxQueueDepth} slots x {self._maxPayload} bytes)")
        self._log.info(f"  FPGA QPN    : 0x{fpga_qpn:06x}")
        self._log.info(f"  FPGA lkey   : 0x{fpga_lkey:08x}")
        self._log.info(f"  FPGA GID    : {_gid_bytes_to_str(self._fpgaGidBytes)}")
        self._log.info(f"  Path MTU    : {self._pmtu} ({[256,512,1024,2048,4096][self._pmtu-1]} bytes)")
        self._log.info(f"  RC connection established — ready to receive RDMA WRITEs")
        self._log.info("=" * 60)

        super()._start()

    def _stop(self) -> None:
        self._server.stop()
        self.ConnectionState.set('Disconnected')
        super()._stop()
