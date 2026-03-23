#-----------------------------------------------------------------------------
# Company    : SLAC National Accelerator Laboratory
#-----------------------------------------------------------------------------
# Description:
#       PyRogue protocols / RoCEv2 wrapper
#
# Provides a pyrogue.Device wrapper around rogue.protocols.rocev2.Server
# so that RoCEv2 streams can be used identically to UDP streams inside a
# pyrogue Root/Device tree.
#
# Immediate-value format (agreed with FPGA firmware):
#
#   bits [7:0]  = channel id
#   bits [31:8] = reserved
#
# byte_len comes free from the CQ completion (wc.byte_len).
# firstUser is always 0x2 (SSI SOF) — each RDMA WRITE is one complete frame.
#-----------------------------------------------------------------------------
# This file is part of the rogue software platform. It is subject to
# the license terms in the LICENSE.txt file found in the top-level directory
# of this distribution and at:
#    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
# No part of the rogue software platform, including this file, may be
# copied, modified, propagated, or distributed except according to the terms
# contained in the LICENSE.txt file.
#-----------------------------------------------------------------------------
import pyrogue as pr
import rogue.protocols.rocev2
from typing import Any


class RoCEv2Server(pr.Device):
    """
    RoCEv2 receive server wrapped as a pyrogue Device.

    Receives RDMA WRITE-with-Immediate frames from an FPGA and forwards
    them as rogue stream frames.  The interface is intentionally analogous
    to :class:`pyrogue.protocols.UdpRssiPack` so that the two can be used
    interchangeably in application code.

    Parameters
    ----------
    deviceName : str
        ibverbs device name, e.g. ``"rxe0"`` (softRoCE) or ``"mlx5_0"``
        (hardware NIC).
    ibPort : int, optional
        ibverbs port number.  Almost always ``1``.
    gidIndex : int, optional
        GID table index selecting the RoCEv2 (UDP/IP) address family.
        Use ``ibv_devinfo -v`` to find the right index for your interface.
        Typically ``0`` for softRoCE, ``3`` for hardware RoCEv2 over IPv4.
    maxPayload : int, optional
        Maximum payload bytes per RDMA WRITE (excluding the 40-byte UD GRH
        that the HCA prepends).  Default 9000 bytes.
    rxQueueDepth : int, optional
        Number of receive work requests to keep posted at all times.
        Increase if completions are dropped at high data rates.
    pollInterval : int, optional
        Poll interval in seconds for status variables.
    **kwargs : Any
        Additional arguments forwarded to :class:`pyrogue.Device`.

    Examples
    --------
    Minimal usage — single channel, data written to file::

        import pyrogue as pr
        import pyrogue.protocols
        import rogue.utilities.fileio

        fileWriter = rogue.utilities.fileio.StreamWriter()
        fileWriter.open('data.bin')

        root = pr.Root()
        root.add(pr.protocols.RoCEv2Server(
            name       = 'rdmaRx',
            deviceName = 'rxe0',
            ibPort     = 1,
            gidIndex   = 0,
        ))
        root.start()

        # Connect the RoCEv2 server stream to the file writer
        root.rdmaRx.stream >> fileWriter.getChannel(0)

    Multi-channel usage::

        root.rdmaRx.getChannel(0) >> fileWriter.getChannel(0)
        root.rdmaRx.getChannel(1) >> myProcessor
    """

    def __init__(
        self,
        *,
        deviceName: str,
        ibPort:       int  = 1,
        gidIndex:     int  = 0,
        maxPayload:   int  = rogue.protocols.rocev2.DefaultMaxPayload,
        rxQueueDepth: int  = rogue.protocols.rocev2.DefaultRxQueueDepth,
        pollInterval: int  = 1,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)

        # Store config for _start / _stop
        self._deviceName   = deviceName
        self._ibPort       = ibPort
        self._gidIndex     = gidIndex
        self._maxPayload   = maxPayload
        self._rxQueueDepth = rxQueueDepth

        # Create the C++ server object
        self._server = rogue.protocols.rocev2.Server.create(
            deviceName,
            ibPort,
            gidIndex,
            maxPayload,
            rxQueueDepth,
        )

        # ---- Status variables -----------------------------------------------

        self.add(pr.LocalVariable(
            name         = 'deviceName',
            mode         = 'RO',
            value        = deviceName,
            description  = 'ibverbs device name',
        ))

        self.add(pr.LocalVariable(
            name         = 'ibPort',
            mode         = 'RO',
            value        = ibPort,
            typeStr      = 'UInt8',
            description  = 'ibverbs port number',
        ))

        self.add(pr.LocalVariable(
            name         = 'gidIndex',
            mode         = 'RO',
            value        = gidIndex,
            typeStr      = 'UInt8',
            description  = 'GID table index (RoCEv2 address family selector)',
        ))

        self.add(pr.LocalVariable(
            name         = 'maxPayload',
            mode         = 'RO',
            value        = maxPayload,
            typeStr      = 'UInt32',
            description  = 'Maximum payload bytes per RDMA WRITE',
        ))

        self.add(pr.LocalVariable(
            name         = 'rxQueueDepth',
            mode         = 'RO',
            value        = rxQueueDepth,
            typeStr      = 'UInt32',
            description  = 'Number of pre-posted receive work requests',
        ))

        self.add(pr.LocalVariable(
            name         = 'rxFrameCount',
            mode         = 'RO',
            value        = 0,
            typeStr      = 'UInt64',
            localGet     = lambda: self._server.getSlaveCount() \
                                   if hasattr(self._server, 'getSlaveCount') \
                                   else self._server.getFrameCount(),
            pollInterval = pollInterval,
            description  = 'Total frames received since start',
        ))

        self.add(pr.LocalVariable(
            name         = 'rxByteCount',
            mode         = 'RO',
            value        = 0,
            typeStr      = 'UInt64',
            localGet     = lambda: self._server.getByteCount(),
            pollInterval = pollInterval,
            description  = 'Total bytes received since start',
        ))

    # -------------------------------------------------------------------------
    # Stream access helpers
    # -------------------------------------------------------------------------

    @property
    def stream(self) -> rogue.protocols.rocev2.Server:
        """
        Direct access to the underlying rogue stream master.

        Use ``root.rdmaRx.stream >> downstream`` to connect a single-channel
        stream without caring about the channel number.
        """
        return self._server

    def getChannel(self, channel: int) -> rogue.interfaces.stream.Filter:
        """
        Return a channel-filtered view of the RoCEv2 stream.

        Inserts a :class:`rogue.interfaces.stream.Filter` between the server
        and the downstream consumer so that only frames carrying the requested
        channel id are forwarded.

        Parameters
        ----------
        channel : int
            Channel id (0–255) as encoded in the immediate value bits [7:0].

        Returns
        -------
        rogue.interfaces.stream.Filter
            A stream Filter already connected to the server output.
            Connect its output to your consumer with ``>>``.
        """
        import rogue.interfaces.stream
        filt = rogue.interfaces.stream.Filter(False, channel)
        self._server >> filt
        return filt

    # -------------------------------------------------------------------------
    # pyrogue lifecycle hooks
    # -------------------------------------------------------------------------

    def _start(self) -> None:
        # The C++ server starts its thread in the constructor, so nothing
        # extra to do here.  Override is present for symmetry with
        # UdpRssiPack and in case a future version adds lazy start.
        self._log.info(
            f"RoCEv2Server '{self.name}' active on device '{self._deviceName}' "
            f"port {self._ibPort} GID index {self._gidIndex}"
        )
        super()._start()

    def _stop(self) -> None:
        self._server.stop()
        super()._stop()
