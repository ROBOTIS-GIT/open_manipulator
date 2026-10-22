# OMX Port Identifier

A GUI tool that automatically identifies Leader and Follower arms connected via USB, assigns camera devices, and generates a ready-to-paste lerobot-record command.

## Requirements

```bash
sudo apt install python3-tk
pip install opencv-python --break-system-packages
```

## Usage

Run on the HOST (not inside Docker):

```bash
python3 omx_port_identifier.py
```

## Features

- Auto-identifies arms via hardware serial registry (one-time setup)
- Supports 0, 1, or 2 arms plugged in at startup
- Camera detection with live preview
- Generates lerobot-record command with all ports pre-filled
