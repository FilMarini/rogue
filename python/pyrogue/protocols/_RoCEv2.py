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
#       192.168.56.10  →  0000:0000:0000:0000:0000:ffff:c0a8:380a
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
#   Usage (mirrors existing Root pattern)
#   ──────────────────────────────────────
#     root.add(pr.protocols.RoCEv2Server(
#         name             = 'rdmaRx',
#         ip               = '192.168.2.10',
#         deviceName       = 'mlx5_0',
#         roceEngineOffset = 0x0000_0000,
#     ))
#     root.start()
#     root.rdmaRx.stream >> fileWriter.getChannel(0)
#-----------------------------------------------------------------------------
import socket
import struct
import pyrogue as pr
import rogue.protocols.rocev2
import rogue.interfaces.stream
from pyrogue.protocols._RoceEngine import RoceEngine
from typing import Any


def _ip_to_gid_bytes(ip: str) -> bytes:
    """
    Convert an IPv4 address string to a 16-byte IPv4-mapped IPv6 GID.

    Example:
        '192.168.56.10'  →  b'\\x00'*10 + b'\\xff\\xff' + b'\\xc0\\xa8\\x38\\x0a'

    This matches the GID format used by RoCEv2 engines on FPGA firmware
    (and by the Linux kernel's RXE / hardware RoCE drivers).
    """
    packed = socket.inet_aton(ip)          # 4 bytes, big-endian
    return b'\x00' * 10 + b'\xff\xff' + packed


def _gid_bytes_to_str(gid: bytes) -> str:
    """
    Format 16 GID bytes as the colon-separated hex string used by ibv_devinfo.

    Example:
        → '0000:0000:0000:0000:0000:ffff:c0a8:380a'
    """
    words = struct.unpack('>8H', gid)
    return ':'.join(f'{w:04x}' for w in words)


class RoCEv2Server(pr.Device):
    """
    RoCEv2 RC receive server — pyrogue Device.

    Mirrors the interface of :class:`pyrogue.protocols.UdpRssiPack` so the
    two can be used interchangeably inside a Root.

    Parameters
    ----------
    ip : str
        FPGA IP address (e.g. ``'192.168.2.10'``).  Used to derive the
        FPGA GID for the RC connection (IPv4-mapped IPv6 format).
    deviceName : str
        ibverbs device name, e.g. ``'rxe0'`` (softRoCE) or ``'mlx5_0'``.
        Run ``ibv_devinfo`` to list available devices.
    ibPort : int
        ibverbs port number (almost always ``1``).
    gidIndex : int
        GID table index selecting the host's RoCEv2 / IPv4 address.
        Run ``ibv_devinfo -v | grep GID`` to find the correct index.
        Typically ``0`` for softRoCE, ``3`` for hardware RoCEv2 over IPv4.
    maxPayload : int
        Maximum payload bytes per RDMA WRITE.  Must match the FPGA's
        configured maximum transfer size.  Default: 9000 bytes.
    rxQueueDepth : int
        Number of receive slots pre-posted to the RC QP.  Increase if
        completions are dropped at high data rates.
    roceEngineOffset : int
        AXI-lite byte offset of the RoCEv2 engine register block within
        the memory map inherited from the parent Device/Root.
    pmtu : int
        Path MTU enum (IBV_MTU_4096 = 5).  Must match FPGA firmware.
    pollInterval : int
        Poll interval in seconds for status variables.
    **kwargs
        Forwarded to :class:`pyrogue.Device` (name, description,
        memBase, offset, expand, …).

    Examples
    --------
    Drop-in replacement for RUDP streams in an existing Root::

        # Before (UDP/RSSI):
        self.rudp = pr.protocols.UdpRssiPack(
            name='SwRudpClient', host=ip, port=8193, packVer=2, jumbo=True)
        self.stream = self.rudp.application(0)

        # After (RoCEv2):
        self.rdmaRx = pr.protocols.RoCEv2Server(
            name='rdmaRx', ip=ip, deviceName='mlx5_0',
            roceEngineOffset=0x0000_0000)
        self.stream = self.rdmaRx.stream
    """

    def __init__(
        self,
        *,
        ip:                 str,
        deviceName:         str,
        ibPort:             int  = 1,
        gidIndex:           int  = 0,
        maxPayload:         int  = rogue.protocols.rocev2.DefaultMaxPayload,
        rxQueueDepth:       int  = rogue.protocols.rocev2.DefaultRxQueueDepth,
        roceEngineOffset:   int  = 0,
        pmtu:               int  = 5,     # IBV_MTU_4096
        pollInterval:       int  = 1,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)

        self._ip           = ip
        self._deviceName   = deviceName
        self._ibPort       = ibPort
        self._gidIndex     = gidIndex
        self._maxPayload   = maxPayload
        self._rxQueueDepth = rxQueueDepth
        self._pmtu         = pmtu

        # Derive FPGA GID from IP once — stored as bytes for setFpgaGid()
        self._fpgaGidBytes = _ip_to_gid_bytes(ip)

        # ------------------------------------------------------------------
        # C++ RC server — ibverbs resources up to QP INIT.
        # Created here (not in _start) so Python can read host QP parameters
        # for debugging before the connection is established.
        # ------------------------------------------------------------------
        self._server = rogue.protocols.rocev2.Server.create(
            deviceName,
            ibPort,
            gidIndex,
            maxPayload,
            rxQueueDepth,
        )

        # ------------------------------------------------------------------
        # FPGA RoCEv2 engine register block (child Device, inherits memBase)
        # ------------------------------------------------------------------
        self.add(RoceEngine(
            name   = 'RoceEngine',
            offset = roceEngineOffset,
        ))

        # ------------------------------------------------------------------
        # Status / info LocalVariables
        # ------------------------------------------------------------------
        self.add(pr.LocalVariable(
            name        = 'FpgaIp',
            description = 'FPGA IP address used for GID derivation',
            mode        = 'RO',
            value       = ip,
        ))

        self.add(pr.LocalVariable(
            name        = 'FpgaGid',
            description = 'FPGA GID derived from IP (IPv4-mapped IPv6)',
            mode        = 'RO',
            value       = _gid_bytes_to_str(self._fpgaGidBytes),
        ))

        self.add(pr.LocalVariable(
            name        = 'HostQpn',
            description = 'Host RC QP number',
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt32',
            localGet    = lambda: self._server.getQpn(),
        ))

        self.add(pr.LocalVariable(
            name        = 'HostGid',
            description = 'Host GID (NIC address used for RoCEv2)',
            mode        = 'RO',
            value       = '',
            localGet    = lambda: self._server.getGid(),
        ))

        self.add(pr.LocalVariable(
            name        = 'MrAddr',
            description = 'Host MR virtual address (FPGA writes here)',
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt64',
            localGet    = lambda: self._server.getMrAddr(),
        ))

        self.add(pr.LocalVariable(
            name        = 'MrRkey',
            description = 'Host MR rkey (given to FPGA engine)',
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt32',
            localGet    = lambda: self._server.getMrRkey(),
        ))

        self.add(pr.LocalVariable(
            name        = 'RxFrameCount',
            description = 'Total frames received',
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt64',
            localGet    = lambda: self._server.getFrameCount(),
            pollInterval= pollInterval,
        ))

        self.add(pr.LocalVariable(
            name        = 'RxByteCount',
            description = 'Total bytes received',
            mode        = 'RO',
            value       = 0,
            typeStr     = 'UInt64',
            localGet    = lambda: self._server.getByteCount(),
            pollInterval= pollInterval,
        ))

        self.add(pr.LocalVariable(
            name        = 'ConnectionState',
            description = 'RC connection state',
            mode        = 'RO',
            value       = 'Disconnected',
        ))

    # ------------------------------------------------------------------
    # Stream access — identical interface to UdpRssiPack
    # ------------------------------------------------------------------

    @property
    def stream(self):
        """
        Direct access to the underlying C++ stream master.

        Mirrors ``rudp.application(0)`` from the UDP/RSSI pattern::

            self.stream = self.rdmaRx.stream
        """
        return self._server

    def getChannel(self, channel: int):
        """
        Return a channel-filtered view of the stream.

        Inserts a rogue.interfaces.stream.Filter that forwards only frames
        whose channel id (immediate value bits [7:0]) matches `channel`.

        Parameters
        ----------
        channel : int
            Channel id 0–255.
        """
        filt = rogue.interfaces.stream.Filter(False, channel)
        self._server >> filt
        return filt

    # ------------------------------------------------------------------
    # pyrogue lifecycle
    # ------------------------------------------------------------------

    def _start(self) -> None:
        """
        Orchestrate the full RC connection handshake:

        1. Push FPGA GID to C++ server (derived from ``ip``).
        2. Read host QP parameters.
        3. Configure FPGA engine via AXI-lite → get FPGA QPN back.
        4. Complete host QP → RTR → RTS, start RX thread.
        """
        self.ConnectionState.set('Connecting')

        # Step 1 — tell the C++ server the FPGA's GID
        self._server.setFpgaGid(list(self._fpgaGidBytes))

        # Step 2 — read host RC parameters
        host_qpn    = self._server.getQpn()
        host_rq_psn = self._server.getRqPsn()
        host_sq_psn = self._server.getSqPsn()
        mr_addr     = self._server.getMrAddr()
        mr_len      = self._maxPayload * self._rxQueueDepth

        self._log.info(
            f"RoCEv2 '{self.name}': host QPN=0x{host_qpn:06x} "
            f"GID={self._server.getGid()}  "
            f"MR addr=0x{mr_addr:016x} rkey=0x{self._server.getMrRkey():08x}  "
            f"FPGA GID={_gid_bytes_to_str(self._fpgaGidBytes)}"
        )

        # Step 3 — configure FPGA engine via AXI-lite metadata bus
        fpga_qpn = self.RoceEngine.setup_connection(
            host_qpn    = host_qpn,
            host_rq_psn = host_rq_psn,
            host_sq_psn = host_sq_psn,
            mr_laddr    = mr_addr,
            mr_len      = mr_len,
            pmtu        = self._pmtu,
        )

        # Step 4 — finish host QP and start RX thread
        # fpgaRqPsn: the PSN the FPGA will start sending from.
        # We use host_sq_psn mirroring the existing script's sqpn4Write
        # convention.  Adjust if firmware returns a different initial PSN.
        self._server.completeConnection(
            fpgaQpn   = fpga_qpn,
            fpgaRqPsn = host_sq_psn,
        )

        self.ConnectionState.set('Connected')
        self._log.info(
            f"RoCEv2 '{self.name}': RC connection established — "
            f"FPGA QPN=0x{fpga_qpn:06x}"
        )

        super()._start()

    def _stop(self) -> None:
        self._server.stop()
        self.ConnectionState.set('Disconnected')
        super()._stop()
