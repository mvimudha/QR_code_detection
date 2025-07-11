# 🤖 QR Code-Based Navigation System

> A ROS2-based system for detecting QR codes using a camera and navigating a robot to predefined locations based on the scanned codes.

---

## 🔗 Table of Contents
- 📍 [Overview](#overview)
- 🚀 [Getting Started](#getting-started)
  - ☑️ [Prerequisites](#prerequisites)
  - ⚙️ [Installation](#installation)
  - 🤖 [Usage](#usage)
  - 🧠 [Code Structure](#code-structure)
- 📁 [Project Structure](#project-structure)
- 📸 [Screenshots](#screenshots)
- 🙌 [Acknowledgments](#acknowledgments)

---

## 📍 Overview

This project combines two ROS2 nodes to implement an autonomous QR code-based navigation system.  
It works in two parts:

1. **QR Code Detection Node**: Captures live camera feed, detects QR codes, and publishes the decoded text.  
2. **QR Code Navigator Node**: Subscribes to the QR code topic, interprets the location from the QR code, and sends navigation goals to the robot accordingly.

---

## 🚀 Getting Started

### ☑️ Prerequisites

- TurtleBot4  
- Python 3.8+  
- ROS2 Humble or Foxy  
- OpenCV  
- `pyzbar`  
- `cv_bridge`  
- A camera publishing to a ROS topic (e.g., `/oakd/rgb/preview/image_raw`)

### ⚙️ Installation

1. **Clone this repository:**
   ```bash
   git clone https://github.com/mvimudha/QR_code_detection.git
   cd QR_code_detection
   ```

2. **Make sure your ROS2 workspace is sourced:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Install dependencies (if not already installed):**
   ```bash
   pip install opencv-python pyzbar
   ```

---

### 🤖 Usage

1. **Launch the QR Code Detection node:**
   ```bash
   ros2 run qr_code_detection qr_code_detection
   ```

2. **Launch the QR Code Navigation node:**
   ```bash
   ros2 run qr_code_navigation qr_code_navigation
   ```

> ✅ Ensure your robot's navigation stack is running and is able to receive goals on the `/goal_pose` topic.

---

## 🧠 Code Structure

### `qr_code_detection.py`

- Subscribes to camera feed  
- Uses OpenCV + `pyzbar` to detect QR codes  
- Publishes decoded text to `/qr_code_data`

### `qr_code_navigation.py`

- Subscribes to `/qr_code_data`  
- Maps the QR code data to real-world `(x, y)` positions  
- Publishes goal as a `PoseStamped` message on `/goal_pose`

---

## 📁 Project Structure

```
QR_code_detection/
├── QR_Codes/               # Screenshots of QR codes
├── qr_code_detection.py    # Node to detect and decode QR codes
├── qr_code_navigation.py   # Node to navigate robot based on QR codes
├── README.md
└── requirements.txt
```

---

## 📸 Screenshots

Here are some images from the working system:

<p float="left">
  <img src="QR_Codes/qr_code_1.png" width="300" alt="QR code 1"/>
  <img src="QR_Codes/qr_code_5.png" width="300" alt="QR code 5"/>
</p>

---

## 🙌 Acknowledgments

- [TurtleBot4](https://turtlebot.github.io/turtlebot4-user-manual/) – main mobile robot platform used  
- [ROS2](https://docs.ros.org/en/foxy/index.html)  
- [OpenCV](https://opencv.org/)  
- [pyzbar](https://github.com/NaturalHistoryMuseum/pyzbar)  
- Inspired by real-world warehouse navigation challenges.
