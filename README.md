# drawing-semB — V-Plotter Drawing Robot

A cable-driven V-plotter that draws images on a vertical whiteboard. Two motors mounted at the top corners of the board pull strings attached to a pen carriage; by controlling the two string lengths together, the carriage can be moved to any point on the board and traced along a path.

## How it works

```
image file  --[image_processing.py]-->  path.txt  --[main.py]-->  Arduino (Driver.ino)  -->  physical drawing
```

1. **`image_processing.py`** takes a source image, converts it to a line-art skeleton, extracts and orders the drawing paths, maps them into real-world coordinates (meters) within a safe drawing zone, and writes the result to `path.txt` as a sequence of `PEN_UP` / `PEN_DOWN` / `G:x,y` commands.
2. **`main.py`** runs on the host computer. It connects to the Arduino over serial, walks the operator through a calibration step (measuring the two string lengths to establish the pen's starting position), then streams the commands from `path.txt` to the Arduino with flow control and buffering.
3. **`Driver/Driver.ino`** runs on the Arduino. It converts XY coordinates to motor encoder targets (and back), drives the two motors with PID position control, and handles motion profiling — including smooth cornering during continuous drawing and full trapezoidal accel/decel profiles for pen-up travel moves.

## Hardware

- Two motors with encoders, each driving a spool/string, mounted at a fixed base width apart
- A pen-holding carriage suspended by the two strings (V/polargraph-style configuration)
- A servo to lift/lower the pen
- An Arduino board running the `ArduinoMotorCarrier` library

## Project structure

```
image_processing.py   Image → drawing path (generates path.txt)
main.py                Host controller — serial communication with the Arduino
path.txt               Generated drawing commands (auto-created, not hand-edited)
Driver/Driver.ino       Arduino firmware — kinematics, motor control, motion planning
images/                 Sample source images for drawing
```

## Setup

Install the Python dependencies:

```
pip install pyserial opencv-python numpy scikit-image matplotlib
```

Upload `Driver/Driver.ino` to the Arduino (requires the `ArduinoMotorCarrier` library).

## Usage

1. Generate a drawing path from an image:
   ```
   python image_processing.py images/mona.png
   ```
   This writes `path.txt` and shows preview plots of the extracted path.

2. Run the host controller:
   ```
   python main.py
   ```
   - On startup, you'll be asked to calibrate: measure the string length from each motor to the pen carriage and enter both values.
   - From the menu, choose **[1]** to load and draw `path.txt`.
   - Press **`S`** at any time to trigger an emergency stop.
   - Other menu options let you release the motors, recalibrate, or query the current pen position.

## Configuration

Physical constants (motor spacing, whiteboard dimensions, safe drawing zone, speeds, PID gains) are defined at the top of `image_processing.py`, `main.py`, and `Driver/Driver.ino`, and are tuned to the specific physical rig. Adjust them if you change the hardware dimensions or motor setup.
