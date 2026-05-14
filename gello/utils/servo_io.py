"""
Shared servo I/O utilities for xArm7.

Provides:
  - detect_port()          — auto-detect U2D2 / FTDI serial port
  - ServoIO                — low-level read/write via dynamixel_sdk
  - ServoReaderThread      — QThread that emits live servo positions at ~30 Hz
  - ScanWorker             — QThread that broadcasts a ping and returns found IDs
  - SetIDWorker            — QThread that reassigns a servo's ID
"""

import time
from pathlib import Path
from typing import List, Optional

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal

NUM_JOINTS = 7
BAUD_RATE = 57600


def detect_port() -> Optional[str]:
    """Auto-detect a Dynamixel-compatible USB serial port."""
    by_id = Path("/dev/serial/by-id")
    if by_id.exists():
        for p in sorted(by_id.iterdir()):
            if "FTDI" in p.name or "U2D2" in p.name:
                return str(p)
    for fb in ["/dev/ttyUSB0", "/dev/ttyUSB1"]:
        if Path(fb).exists():
            return fb
    try:
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            desc = (port.manufacturer or "") + (port.description or "")
            if "FTDI" in desc or "U2D2" in desc:
                return port.device
        ports = list(serial.tools.list_ports.comports())
        if ports:
            return ports[0].device
    except ImportError:
        pass
    return None


class ServoIO:
    """Low-level servo read/write via dynamixel_sdk."""

    def __init__(self, port: str, num_ids: int = NUM_JOINTS + 1):
        from dynamixel_sdk import GroupSyncRead, PacketHandler, PortHandler
        self._ph = PortHandler(port)
        self._pk = PacketHandler(2.0)
        self._ph.openPort()
        self._ph.setBaudRate(BAUD_RATE)
        self._num_ids = num_ids
        self._gsr = GroupSyncRead(self._ph, self._pk, 132, 4)
        for i in range(1, num_ids + 1):
            self._gsr.addParam(i)
        for _ in range(10):
            self._gsr.txRxPacket()
            time.sleep(0.01)

    def read_raw(self) -> List[float]:
        """Read all servos, return radians."""
        from dynamixel_sdk import COMM_SUCCESS
        if self._gsr.txRxPacket() != COMM_SUCCESS:
            return [0.0] * self._num_ids
        out = []
        for i in range(1, self._num_ids + 1):
            raw = self._gsr.getData(i, 132, 4)
            if raw > 0x7FFFFFFF:
                raw -= 0x100000000
            out.append(raw / 2048.0 * np.pi)
        return out

    def read_avg(self, n: int = 10) -> List[float]:
        """Average n readings."""
        samples = []
        for _ in range(n):
            samples.append(self.read_raw())
            time.sleep(0.02)
        return list(np.mean(samples, axis=0))

    def close(self):
        try:
            self._ph.closePort()
        except Exception:
            pass


class ServoReaderThread(QThread):
    """Continuously reads servo positions at ~30 Hz."""
    update = pyqtSignal(list)

    def __init__(self, port: str, num_ids: int = NUM_JOINTS + 1):
        super().__init__()
        self._port = port
        self._num_ids = num_ids
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        try:
            io = ServoIO(self._port, self._num_ids)
            while self._running:
                vals = io.read_raw()
                self.update.emit(vals)
                time.sleep(0.033)
            io.close()
        except Exception:
            pass


class ScanWorker(QThread):
    """Broadcast ping — returns dict of {id: model_info}."""
    result = pyqtSignal(dict)
    error = pyqtSignal(str)

    def __init__(self, port: str):
        super().__init__()
        self.port = port
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        try:
            from dynamixel_sdk import PacketHandler, PortHandler
            ph = PortHandler(self.port)
            pk = PacketHandler(2.0)
            if not ph.openPort():
                if not self._stop:
                    self.error.emit(f"Failed to open port {self.port}")
                    self.result.emit({})
                return
            if not ph.setBaudRate(BAUD_RATE):
                ph.closePort()
                if not self._stop:
                    self.error.emit(f"Failed to set baud rate {BAUD_RATE} on {self.port}")
                    self.result.emit({})
                return
            data, _ = pk.broadcastPing(ph)
            ph.closePort()
            if not self._stop:
                self.result.emit(dict(data) if data else {})
        except Exception as e:
            if not self._stop:
                self.error.emit(f"Scan error on {self.port}: {e}")
                self.result.emit({})


class SetIDWorker(QThread):
    """Write a new ID to a servo."""
    done = pyqtSignal(bool)

    def __init__(self, port: str, from_id: int, to_id: int):
        super().__init__()
        self.port = port
        self.from_id = from_id
        self.to_id = to_id

    def run(self):
        try:
            from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
            ph = PortHandler(self.port)
            pk = PacketHandler(2.0)
            ph.openPort()
            ph.setBaudRate(BAUD_RATE)
            r, _ = pk.write1ByteTxRx(ph, self.from_id, 7, self.to_id)
            ph.closePort()
            self.done.emit(r == COMM_SUCCESS)
        except Exception:
            self.done.emit(False)
