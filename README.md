# Robotic Control System

![Python LOC badge](docs/badges/python_loc.svg)

Inspired by a Star Wars droid named Loadlifter

Python software to control the Hiwonder ArmPi Pro robot.

---

## Features

- Autonom driving in a warehouse using Mecanum wheels and a LiDAR
- Use robotic arm to pick up and place objects
- Camera for object detection with machine learning
- Demo videos available on my YouTube https://www.youtube.com/@danielrmv

---

👨🏽‍💻 Created as a student project

---

## Installation on the Raspberry Pi

1. **Prepare the system**
   - Raspberry Pi OS or Ubuntu recommended, then install all updates:\
     `sudo apt update && sudo apt full-upgrade -y`
   - Install basic tools:\
     `sudo apt install -y git python3 python3-pip python3-venv libgpiod2`
   - Optional: Enable additional tools for camera, I2C, and serial interfaces (`sudo raspi-config`) and install `libatlas-base-dev` if you need faster NumPy operations.

2. **Clone the repository**
   ```bash
   cd ~
   git clone https://github.com/daniel-rmv/Loadlifter.git
   cd Loadlifter
   ```

3. **Create the Python environment**
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip
   ```

4. **Install Python dependencies**
   ```bash
   pip install -r requirements.txt
   ```
   - For camera functionality, make sure the Raspberry Pi camera is enabled.
   - For the `gpiod` path, run `sudo adduser $USER gpio` and log in again if needed.

5. **Prepare the hardware**
   - Enable I2C for the motor controller (`sudo raspi-config` → Interface Options → I2C).
   - Configure the serial interface for servos/LiDAR (enable UART, disable console login).
   - Check the wiring for the buzzer, LiDAR, motor controller, and camera.

6. **Test the software**
   - Example: start the control via CLI\
     ```bash
     python -m src --help
     ```
   - For live visualization, use a PC/laptop and start the camera/LiDAR.
   
---

## Overlay-Client auf dem Mac starten

1. **Repository klonen**
   ```bash
   cd ~
   git clone https://github.com/daniel-rmv/Loadlifter.git
   cd Loadlifter
   ```

2. **Virtuelle Umgebung und Abhängigkeiten**
   - Python 3 ist auf macOS bereits vorhanden (bei Bedarf über Homebrew aktualisieren).
   - Virtuelle Umgebung anlegen und Pakete installieren:
     ```bash
     python3 -m venv .venv
     source .venv/bin/activate
     python -m pip install --upgrade pip
     pip install -r requirements.txt
     ```

3. **Overlay-Client ausführen**
   - Sicherstellen, dass auf dem Pi der MJPEG-Stream läuft (`python -m src.low_level.camera_bridge`).
   - Overlay-Client starten und die Pi-IP einsetzen:
     ```bash
     python -m src.visualization.overlay_client --pi <pi-ip>:5000
     ```
   - Optional das Modell oder Parameter wie `--conf` (Konfidenz) oder `--fps` anpassen. Standardmodell liegt unter `models/best.pt`.
