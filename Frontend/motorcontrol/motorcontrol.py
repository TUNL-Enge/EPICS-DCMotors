# This Python file uses the following encoding: utf-8
"""Motor control GUI for the ETH32 EPICS driver.

The user picks one of the driver's motors (1-8) from the selector. Manual
control is handled by the EPICS Qt (QE) widgets defined in motorcontrol.ui,
whose PV names use the macros $(P) (prefix) and $(M) (motor number). This
file adds automatic control: the user enters a desired position and the
selected motor is driven there in a background thread.

The PV prefix defaults to "testETH32:motors:" and can be overridden with the
ETH32_PREFIX environment variable.
"""
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

from PySide6.QtCore import QDir, QFile, QObject, QThread, Signal
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import (QApplication, QComboBox, QGroupBox, QLabel, QLineEdit,
                               QPushButton, QVBoxLayout, QWidget)

PREFIX = os.environ.get("ETH32_PREFIX", "testETH32:motors:")
NUM_MOTORS = 8         # Motors 1-8 (one per ETH32 analog input)

MIN_POS = 0.0          # Lowest valid target (driver clamps position to 0-10 V)
MAX_POS = 10.0         # Highest valid target
STEP = 0.05            # Targets must be a multiple of this
TOLERANCE = STEP / 2   # "Already there" if within this of the target
POLL_INTERVAL = 0.1    # Seconds between position reads
STALL_TIMEOUT = 5.0    # Abort if position does not change for this long (s)
CA_TIMEOUT = 2.0       # Channel Access timeout for caget/caput (s)


class EpicsError(RuntimeError):
    pass


class MotorPVs:
    """The PV names for one motor (numbered from 1)."""

    def __init__(self, motor):
        base = f"{PREFIX}M{motor}"
        self.motor = motor
        self.run = base + "Run"
        self.forward = base + "For"
        self.backward = base + "Back"
        self.position = base + "Pos_RBV"
        self.run_rbv = base + "Run_RBV"
        self.forward_rbv = base + "For_RBV"
        self.backward_rbv = base + "Back_RBV"


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


def stop_motor(pvs, run_off=True):
    """Turn off both directions (and the run flag). Tries every PV even if one fails."""
    failed = []
    for pv in (pvs.forward, pvs.backward) + ((pvs.run,) if run_off else ()):
        try:
            caput(pv, 0)
        except EpicsError:
            failed.append(pv)
    if failed:
        raise EpicsError(f"Could not stop motor {pvs.motor}: failed to write "
                         + ", ".join(failed))


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
    """Drives one motor to a target position. Runs inside a QThread."""

    message = Signal(str)
    finished = Signal()

    def __init__(self, motor, target):
        super().__init__()
        self.pvs = MotorPVs(motor)
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
                stop_motor(self.pvs)
            except EpicsError as exc:
                self.message.emit(f"Error: {exc}. Check the motor manually!")
            self.finished.emit()

    def _move(self):
        target, pvs = self.target, self.pvs
        name = f"Motor {pvs.motor}"
        pos = caget(pvs.position)
        if abs(pos - target) <= TOLERANCE:
            return f"{name} already at {pos:.3f}; target {target:.2f}."

        forward = pos < target
        direction_pv = pvs.forward if forward else pvs.backward
        direction_rbv = pvs.forward_rbv if forward else pvs.backward_rbv
        self.message.emit(f"{name}: moving {'forward' if forward else 'backward'} "
                          f"from {pos:.3f} to {target:.2f}...")

        caput(pvs.run, 1)
        caput(direction_pv, 1)

        last_pos = pos
        last_change = time.monotonic()
        while True:
            if self._abort:
                return f"{name}: move aborted."
            time.sleep(POLL_INTERVAL)
            pos = caget(pvs.position)

            # Stop once we reach or pass the target (never reverse to chase it).
            if (forward and pos >= target - TOLERANCE) or \
               (not forward and pos <= target + TOLERANCE):
                return f"{name} reached {pos:.3f} (target {target:.2f})."

            # Someone pressed a manual stop/direction/off button.
            if caget(direction_rbv) == 0 or caget(pvs.run_rbv) == 0:
                return f"{name}: move stopped externally at {pos:.3f}."

            now = time.monotonic()
            if abs(pos - last_pos) > 1e-6:
                last_pos, last_change = pos, now
            elif now - last_change > STALL_TIMEOUT:
                raise EpicsError(f"{name} stalled at {pos:.3f} "
                                 f"(no movement for {STALL_TIMEOUT:g} s)")


class Widget(QWidget):

    stop_message = Signal(str)  # Emitted from the Stop helper thread

    def __init__(self):
        super().__init__()
        self.load_ui()
        self.thread = None
        self.worker = None

        self.move_button = self.findChild(QPushButton, "move_button")
        self.stop_button = self.findChild(QPushButton, "stop_button")
        self.desired_pos = self.findChild(QLineEdit, "desired_pos")
        self.status_message = self.findChild(QLabel, "status_message")
        self.prefix_label = self.findChild(QLabel, "prefix_label")
        self.motor_select = self.findChild(QComboBox, "motor_select")
        self.plot_group = self.findChild(QGroupBox, "plot_group")
        if not all((self.move_button, self.stop_button, self.desired_pos,
                    self.status_message, self.prefix_label, self.motor_select,
                    self.plot_group)):
            raise RuntimeError("motorcontrol.ui is missing a required widget")
        self.qe_widgets = [w for w in self.findChildren(QWidget)
                           if w.property("variableSubstitutions") is not None]
        self.prefix_label.setText(f"PV prefix: {PREFIX}")

        self.motor_select.clear()
        for motor in range(1, NUM_MOTORS + 1):
            self.motor_select.addItem(f"Motor {motor}", motor)
        self.motor_select.currentIndexChanged.connect(self.select_motor)
        self.select_motor()

        self.move_button.clicked.connect(self.update_position)
        self.desired_pos.returnPressed.connect(self.update_position)
        self.stop_button.clicked.connect(self.stop)
        self.stop_message.connect(self.show_message)

    def load_ui(self):
        here = Path(__file__).resolve().parent
        loader = QUiLoader()
        loader.setWorkingDirectory(QDir(os.fspath(here)))  # For relative image paths
        path = os.fspath(here / "motorcontrol.ui")
        ui_file = QFile(path)
        if not ui_file.open(QFile.ReadOnly):
            raise RuntimeError(f"Cannot open {path}: {ui_file.errorString()}")
        ui = loader.load(ui_file, self)
        ui_file.close()
        if ui is None:
            raise RuntimeError(f"Cannot load {path}: {loader.errorString()}")
        # Put the loaded form in a layout so it resizes with the window.
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(ui)
        self.setWindowTitle(ui.windowTitle())
        self.setMinimumSize(ui.minimumSize())
        self.resize(ui.size())

    @property
    def motor(self):
        """The currently selected motor number (1 to NUM_MOTORS)."""
        return self.motor_select.currentData()

    def select_motor(self):
        """Point every QE widget at the selected motor's PVs."""
        for widget in self.qe_widgets:
            widget.setProperty("variableSubstitutions", f"P={PREFIX},M={self.motor}")
        self.plot_group.setTitle(f"Motor {self.motor} Position History (last 60 s)")
        self.show_message(f"Motor {self.motor} selected. "
                          "Enter a target position and press Move.")

    def update_position(self):
        if self.thread is not None:
            self.show_message("A move is already in progress.")
            return

        text = self.desired_pos.text().strip()
        target = parse_target(text)
        if target is None:
            self.show_message(
                f"\"{text}\" is not a valid position. Enter a number from "
                f"{MIN_POS:g} to {MAX_POS:g} in steps of {STEP:g}.")
            return

        # Keep the selector on the moving motor until the move is over.
        self.move_button.setEnabled(False)
        self.motor_select.setEnabled(False)
        self.thread = QThread(self)
        self.worker = MoveWorker(self.motor, target)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.message.connect(self.show_message)
        self.worker.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.move_done)
        self.thread.start()

    def stop(self):
        """Stop the selected motor: abort any automatic move and clear both directions."""
        if self.worker is not None:
            self.worker.abort()
        # Run the writes off the GUI thread so a slow IOC can't freeze the window.
        threading.Thread(target=self._stop_directions, args=(MotorPVs(self.motor),),
                         daemon=True).start()

    def _stop_directions(self, pvs):
        try:
            stop_motor(pvs, run_off=False)
        except EpicsError as exc:
            self.stop_message.emit(f"Error: {exc}. Check the motor manually!")
            return
        if self.worker is None:
            self.stop_message.emit(f"Motor {pvs.motor} stopped.")

    def show_message(self, text):
        print(text)
        self.status_message.setText(text)

    def move_done(self):
        self.thread.wait()
        self.worker.deleteLater()
        self.thread.deleteLater()
        self.worker = None
        self.thread = None
        self.move_button.setEnabled(True)
        self.motor_select.setEnabled(True)

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
