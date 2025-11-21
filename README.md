# Auto-Driving Robot Simulation

This repository implements an auto-driving robot system with navigation, server communication, and a browser-based simulator.  
It follows a unified structure to ensure ease of testing and evaluation.

---

## Purpose

This project demonstrates autonomous robot navigation in a simulated environment.  
The robot connects to a backend Python server, communicates over WebSockets, executes a planned path visiting all required endpoints, and finishes at the goal with a clean exit status.

---

## ⚙️ Installation

### 1. Clone Repository

```bash
git clone https://github.com/virkhanna20/RL-agent-for-Path-planning
```

### 2. Install Dependencies

Python ≥ 3.8 is required.

---

## 🚀 Run Instructions

# Install

pip install Pillow
pip install numpy
pip install opencv-python
pip install requests

Follow the steps below in separate terminals:

### 1. Start the Server (Terminal 1)

```bash
python server.py
```

### 2. Launch the Simulator (Terminal 2)

Open the file `simulator.html` in a browser.

### 3. Run the Robot Navigator (Terminal 3)

```bash
python .\vision_navigator.py
```

The robot will connect to the server, communicate with the simulator, execute navigation, and exit with **Success/Fail** depending on task completion.

---

## ⚡ Configuration

Default settings are defined inside the scripts:

- **Server Host:** `localhost`
- **Server Port:** `8000`
- **WebSocket Path:** `/ws`

To modify, update parameters inside `server.py` and `robot_navigator.py`.

---

---

## 🔧 System Requirements

- Python ≥ 3.8
- Web browser (for `simulator.html`)
- OS: Linux / macOS / Windows

---
