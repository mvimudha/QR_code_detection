🚀 TurtleBot4 Autonomous QR Code Navigation

This project focuses on enabling autonomous indoor navigation for a TurtleBot4 using QR code-based waypoints. Instead of relying on GPS or manual path planning, the robot identifies and responds to QR codes placed in the environment to dynamically decide its next movement.
🧠 Core Concept

    QR codes act as navigation markers—each one encodes specific location tags

    The TurtleBot4 uses its RGB camera to detect QR codes in real-time.

    Upon detection, it decodes the QR content and performs appropriate movement actions using ROS2.

📦 QR Code Detection Module (qr_code_detection.py)

This Python script is responsible for:

    Accessing camera feed from the TurtleBot4.

    Detecting QR codes in each frame using OpenCV and pyzbar.

    Decoding the content of the QR code and publishing it to a ROS2 topic.

    Filtering repeated reads and maintaining a short-term memory of recent QR detections to avoid redundancy.

🔍 Key Features

    Uses pyzbar for fast and reliable QR detection.

    Publishes decoded QR data to a ROS2 topic (e.g., /qr_code_data) for further navigation decision-making.

    Includes visual debugging with bounding boxes and text overlays in the camera window (optional).
