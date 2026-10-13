# CAN Bus (`can0`) Setup Guide

Based on `lsusb`, your CAN adapter device is identified as:
- **Device**: OpenMoko, Inc. Geschwister Schneider CAN adapter (gs_usb firmware / Candlelight / CANable)
- **Vendor ID (`idVendor`)**: `1d50`
- **Product ID (`idProduct`)**: `606f`
- **Serial Number**: `0037004B4233570920303936`

---

## 1. Quick Manual Setup (Temporary)

To bring up the interface manually (e.g. at 1 Mbps bitrate):

```bash
# Bring the interface down first (if active)
sudo ip link set can0 down

# Configure the bitrate (adjust 1000000 to your desired bitrate, e.g., 500000 or 1000000)
sudo ip link set can0 type can bitrate 500000

# Bring the interface up
sudo ip link set can0 up
```

### Verify Status
```bash
ip -details link show can0
```

### Test Communication
Install CAN utility tools if not already installed:
```bash
sudo apt update && sudo apt install -y can-utils
```

- Monitor packets:
  ```bash
  candump can0
  ```
- Send a test frame:
  ```bash
  cansend can0 123#DEADBEEF
  ```

---

## 2. Persistent Setup: Bind Device via Udev Rule

If you want this exact adapter to always be assigned the interface name `can0`:

1. Create a udev rule:
   ```bash
   sudo nano /etc/udev/rules.d/80-can.rules
   ```

2. Add the following rule:
   ```udev
   SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="0037004B4233570920303936", NAME="can0"
   ```

3. Reload and apply udev rules:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

---

## 3. Auto-bringup on Boot

Choose one of the following methods to automatically bring `can0` up at startup.

### Option A: Using `systemd-networkd` (Recommended)

1. Create a network configuration file:
   ```bash
   sudo nano /etc/systemd/network/80-can0.network
   ```

2. Add:
   ```ini
   [Match]
   Name=can0

   [CAN]
   BitRate=1000000
   ```
   *(Change `BitRate` to `500000` or `1000000` as required by your CAN bus.)*

3. Enable and restart `systemd-networkd`:
   ```bash
   sudo systemctl enable systemd-networkd
   sudo systemctl restart systemd-networkd
   ```

---

### Option B: Using a Systemd Service

1. Create a service file:
   ```bash
   sudo nano /etc/systemd/system/can0-bringup.service
   ```

2. Add:
   ```ini
   [Unit]
   Description=Setup and bring up can0 interface
   After=network.target

   [Service]
   Type=oneshot
   RemainAfterExit=yes
   ExecStart=/sbin/ip link set can0 type can bitrate 1000000
   ExecStart=/sbin/ip link set can0 up
   ExecStop=/sbin/ip link set can0 down

   [Install]
   WantedBy=multi-user.target
   ```

3. Enable and start the service:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable can0-bringup.service
   sudo systemctl start can0-bringup.service
   ```
