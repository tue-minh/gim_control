#!/usr/bin/env python3
import sys
import subprocess
import glob
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFormLayout, QComboBox, QLineEdit, QPushButton, QTextEdit,
    QGroupBox, QLabel, QMessageBox
)
from PyQt6.QtCore import QThread, pyqtSignal


class CanSetupThread(QThread):
    output = pyqtSignal(str)
    finished_ok = pyqtSignal()
    finished_err = pyqtSignal(str)

    def __init__(self, device, can_if):
        super().__init__()
        self.device = device
        self.can_if = can_if

    def run(self):
        # NOTE: -x matches process name only, avoids pkill killing its own `bash -c` wrapper
        # (pkill -f would match the script text itself which contains "slcand ... canX")
        script = f"""#!/bin/bash
exec 2>&1
set -x
pkill -x slcand 2>/dev/null || true
modprobe slcan || exit 1
slcan_attach -o -s8 {self.device} || exit 2
slcand {self.device} {self.can_if} || exit 3
sleep 0.5
ip link set {self.can_if} down || exit 4
ip link set {self.can_if} type can bitrate 1000000 || exit 5
ip link set {self.can_if} up || exit 6
ip link set up {self.can_if} || exit 7
ip link show {self.can_if}
"""
        self.output.emit("$ pkexec bash -c <setup_script>")
        try:
            result = subprocess.run(
                ["pkexec", "bash", "-c", script],
                capture_output=True, text=True, timeout=30
            )
            if result.stdout.strip():
                self.output.emit(result.stdout.strip())
            if result.stderr.strip():
                self.output.emit(result.stderr.strip())
            if result.returncode != 0:
                self.finished_err.emit(f"Setup failed (exit {result.returncode})")
                return
        except Exception as e:
            self.finished_err.emit(str(e))
            return
        self.finished_ok.emit()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN Interface Setup")
        self.setMinimumSize(550, 450)
        self.thread = None

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        form_group = QGroupBox("Configuration")
        form = QFormLayout()

        self.device_combo = QComboBox()
        self.device_combo.setEditable(True)
        self.refresh_devices()
        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self.refresh_devices)
        dev_layout = QHBoxLayout()
        dev_layout.addWidget(self.device_combo)
        dev_layout.addWidget(btn_refresh)
        form.addRow("Device:", dev_layout)

        self.if_edit = QLineEdit("can0")
        form.addRow("CAN Interface:", self.if_edit)

        form_group.setLayout(form)
        layout.addWidget(form_group)

        btn_layout = QHBoxLayout()
        self.btn_up = QPushButton("Bring Up CAN")
        self.btn_up.clicked.connect(self.bring_up)
        self.btn_down = QPushButton("Bring Down CAN")
        self.btn_down.clicked.connect(self.bring_down)
        btn_layout.addWidget(self.btn_up)
        btn_layout.addWidget(self.btn_down)
        layout.addLayout(btn_layout)

        layout.addWidget(QLabel("Log:"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)

    def refresh_devices(self):
        self.device_combo.clear()
        devs = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        self.device_combo.addItems(devs if devs else [])

    def log_msg(self, msg):
        self.log.append(msg)

    def bring_up(self):
        device = self.device_combo.currentText().strip()
        can_if = self.if_edit.text().strip()
        if not device or not can_if:
            QMessageBox.warning(self, "Error", "Device and interface required.")
            return
        self.btn_up.setEnabled(False)
        self.log.clear()
        self.log_msg(f"[INFO] Bringing up CAN on {device} -> {can_if}")
        self.thread = CanSetupThread(device, can_if)
        self.thread.output.connect(self.log_msg)
        self.thread.finished_ok.connect(self.on_success)
        self.thread.finished_err.connect(self.on_error)
        self.thread.start()

    def bring_down(self):
        can_if = self.if_edit.text().strip()
        if not can_if:
            return
        subprocess.run(["pkexec", "bash", "-c", f"ip link set {can_if} down; pkill -x slcand"], capture_output=True)
        self.log_msg(f"[INFO] {can_if} brought down.")

    def on_success(self):
        self.btn_up.setEnabled(True)
        self.log_msg("[INFO] CAN interface is up.")

    def on_error(self, err):
        self.btn_up.setEnabled(True)
        self.log_msg(f"[FAIL] {err}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())