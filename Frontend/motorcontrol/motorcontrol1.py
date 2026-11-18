# This Python file uses the following encoding: utf-8
"""Motor control GUI for the ETH32 EPICS driver.

Manual control is handled entirely by the EPICS Qt (QE) widgets defined in
motorcontrol.ui. This file adds automatic control: the user enters a desired
position and the motor is driven there in a background thread.

The PV prefix defaults to "testETH32:motors:" and can be overridden with the
ETH32_PREFIX environment variable.
"""
import os
import subprocess
import sys
import time
from pathlib import Path

from PySide6.QtCore import QFile, QObject, QThread, Signal
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QLabel, QLineEdit, QPushButton, QWidget

PREFIX = os.environ.get("ETH32_PREFIX", "testETH32:motors:")
PV_RUN = PREFIX + "Run"
PV_FORWARD = PREFIX + "M1For"
PV_BACKWARD = PREFIX + "M1Back"
PV_POSITION = PREFIX + "M1Pos_RBV"
PV_RUN_RBV = PREFIX + "Run_RBV"
PV_FORWARD_RBV = PREFIX + "M1For_RBV"
PV_BACKWARD_RBV = PREFIX + "M1Back_RBV"

MIN_POS = 0.0          # Lowest valid target (driver clamps position to 0-10 V)
MAX_POS = 10.0         # Highest valid target
STEP = 0.05            # Targets must be a multiple of this
TOLERANCE = STEP / 2   # "Already there" if within this of the target
POLL_INTERVAL = 0.1    # Seconds between position reads
STALL_TIMEOUT = 5.0    # Abort if position does not change for this long (s)
CA_TIMEOUT = 2.0       # Channel Access timeout for caget/caput (s)


class EpicsError(RuntimeError):
    pass


def caget(pv):
    """Return the numeric value of a PV (enums are returned as their index)."""
    try:
        out = subprocess.run(
            ["caget", "-t", "-n", "-w", str(CA_TIMEOUT), pv],
            capture_output=True, text=True, timeout=CA_TIMEOUT + 2, check=True,
        ).stdout
        return float(out.split()[0])
    except (subprocess.SubprocessError, OSError, ValueError, IndexError) as exc:
        raise EpicsError(f"Could not read {pv}") from exc


def caput(pv, value):
    try:
        subprocess.run(
            ["caput", "-c", "-w", str(CA_TIMEOUT), pv, str(value)],
            capture_output=True, text=True, timeout=CA_TIMEOUT + 2, check=True,
        )
    except (subprocess.SubprocessError, OSError) as exc:
        raise EpicsError(f"Could not write {pv}") from exc


def stop_motor():
    """Turn off both directions and the run flag. Tries every PV even if one fails."""
    failed = []
    for pv in (PV_FORWARD, PV_BACKWARD, PV_RUN):
        try:
            caput(pv, 0)
        except EpicsError:
            failed.append(pv)
    if failed:
        raise EpicsError("Could not stop motor: failed to write " + ", ".join(failed))


def parse_target(text):
    """Return the target as a float, or None if it is not a valid position."""
    try:
        value = float(text)
    except ValueError:
        return None
    if not MIN_POS <= value <= MAX_POS:
        return None
    steps = value / STEP
    if abs(steps - round(steps)) > 1e-6:
        return None
    return value


class MoveWorker(QObject):
    """Drives the motor to a target position. Runs inside a QThread."""

    message = Signal(str)
    finished = Signal()

    def __init__(self, target):
        super().__init__()
        self.target = target
        self._abort = False

    def abort(self):
        self._abort = True

    def run(self):
        try:
            self.message.emit(self._move())
        except EpicsError as exc:
            self.message.emit(f"Error: {exc}")
        finally:
            # Always leave the motor stopped, whatever happened above.
            try:
                stop_motor()
            except EpicsError as exc:
                self.message.emit(f"Error: {exc}\nCheck the motor manually!")
            self.finished.emit()

    def _move(self):
        target = self.target
        pos = caget(PV_POSITION)
        if abs(pos - target) <= TOLERANCE:
            return f"Already at {pos:.3f}; target {target:.2f}."

        forward = pos < target
        direction_pv = PV_FORWARD if forward else PV_BACKWARD
        direction_rbv = PV_FORWARD_RBV if forward else PV_BACKWARD_RBV
        self.message.emit(f"Moving {'forward' if forward else 'backward'}\n"
                          f"from {pos:.3f} to {target:.2f}...")

        caput(PV_RUN, 1)
        caput(direction_pv, 1)

        last_pos = pos
        last_change = time.monotonic()
        while True:
            if self._abort:
                return "Move aborted."
            time.sleep(POLL_INTERVAL)
            pos = caget(PV_POSITION)

            # Stop once we reach or pass the target (never reverse to chase it).
            if (forward and pos >= target - TOLERANCE) or \
               (not forward and pos <= target + TOLERANCE):
                return f"Reached {pos:.3f} (target {target:.2f})."

            # Someone pressed a manual stop/direction/off button.
            if caget(direction_rbv) == 0 or caget(PV_RUN_RBV) == 0:
                return f"Move stopped externally at {pos:.3f}."

            now = time.monotonic()
            if abs(pos - last_pos) > 1e-6:
                last_pos, last_change = pos, now
            elif now - last_change > STALL_TIMEOUT:
                raise EpicsError(f"Motor stalled at {pos:.3f}\n"
                                 f"(no movement for {STALL_TIMEOUT:g} s)")


class Widget(QWidget):

    def __init__(self):
        super().__init__()
        self.load_ui()
        self.thread = None
        self.worker = None

        self.automatic_pos = self.findChild(QPushButton, "automatic_control")
        self.desired_pos = self.findChild(QLineEdit, "desired_pos")
        self.error_status = self.findChild(QLabel, "error_status")
        if not (self.automatic_pos and self.desired_pos and self.error_status):
            raise RuntimeError("motorcontrol.ui is missing an automatic-control widget")

        self.automatic_pos.clicked.connect(self.update_position)
        self.desired_pos.returnPressed.connect(self.update_position)
        self.error_status.setText("The most recent automatic control message\n"
                                  "will be placed here!")

    def load_ui(self):
        loader = QUiLoader()
        path = os.fspath(Path(__file__).resolve().parent / "motorcontrol.ui")
        ui_file = QFile(path)
        if not ui_file.open(QFile.ReadOnly):
            raise RuntimeError(f"Cannot open {path}: {ui_file.errorString()}")
        ui = loader.load(ui_file, self)
        ui_file.close()
        if ui is None:
            raise RuntimeError(f"Cannot load {path}: {loader.errorString()}")
        self.resize(ui.size())

    def update_position(self):
        if self.thread is not None:
            self.error_status.setText("A move is already in progress.")
            return

        text = self.desired_pos.text().strip()
        target = parse_target(text)
        if target is None:
            self.error_status.setText(
                f"{text!r} is not a valid input!\n"
                f"Valid inputs are between {MIN_POS:g} and {MAX_POS:g}\n"
                f"and divisible by {STEP:g}.")
            return

        self.automatic_pos.setEnabled(False)
        self.thread = QThread(self)
        self.worker = MoveWorker(target)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.message.connect(self.show_message)
        self.worker.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.move_done)
        self.thread.start()

    def show_message(self, text):
        print(text)
        self.error_status.setText(text)

    def move_done(self):
        self.thread.wait()
        self.worker.deleteLater()
        self.thread.deleteLater()
        self.worker = None
        self.thread = None
        self.automatic_pos.setEnabled(True)

    def closeEvent(self, event):
        # Stop any move in progress before the window goes away.
        if self.thread is not None:
            self.worker.abort()
            self.thread.quit()
            self.thread.wait()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = Widget()
    widget.show()
    sys.exit(app.exec())
