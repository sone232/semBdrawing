import serial
import time
import math
import sys
import msvcrt

COM_PORT = 'COM4'
BAUD_RATE = 115200
BASE_WIDTH = 0.59
R_SPOOL = 0.0045
COUNTS_PER_REV = 1200
L_ARM = 0.075
MOTOR1_IS_LEFT = False
MOVE_TIMEOUT = 35
PEN_UP = 44
PEN_DOWN = 9

ser = None
stopped = False
motors_engaged = False


def check_stop():
    global stopped
    while msvcrt.kbhit():
        if msvcrt.getch().lower() == b's':
            print("\n!!! EMERGENCY STOP !!!")
            stopped = True
            try:
                for _ in range(15):
                    ser.write(b'!')
                    time.sleep(0.02)
                ser.flush()
            except Exception:
                pass
            return True
    return False


def connect():
    global ser
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    ser.setDTR(False)
    time.sleep(0.1)
    ser.setDTR(True)
    print("Waiting for Arduino...")
    deadline = time.time() + 10
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"  {line}")
            if line in ("READY", "RESUMED"):
                ser.reset_input_buffer()
                print("Connected!\n")
                return True
    print("ERROR: No READY!")
    return False


def send_command(cmd, ack, timeout=5):
    ser.reset_input_buffer()
    ser.write(f"{cmd}\n".encode())
    ser.flush()
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if line == ack:
                return True
        time.sleep(0.01)
    return False


def calibrate():
    global motors_engaged
    print("=== Calibration ===")
    print("Measure string from each motor to its pulley.")
    l1 = float(input("  L1 - left string (meters):  "))
    l2 = float(input("  L2 - right string (meters): "))
    z_i = (l1 + L_ARM, l2 + L_ARM)
    x_init = (z_i[0]**2 - z_i[1]**2 + BASE_WIDTH**2) / (2 * BASE_WIDTH)
    y_init = math.sqrt(max(0, z_i[0]**2 - x_init**2))
    print(f"  Z_i = ({z_i[0]:.4f}, {z_i[1]:.4f})")
    print(f"  Initial position: ({x_init:.4f}, {y_init:.4f})")
    if send_command("CALIBRATE", "CALIBRATED"):
        motors_engaged = True
        print("  Encoders zeroed on Arduino.")
    else:
        print("  WARNING: CALIBRATE not acknowledged.")
    print()
    return z_i


def release_motors():
    global motors_engaged
    print("  Releasing motors...")
    if send_command("RELEASE", "RELEASED"):
        motors_engaged = False
        print("  Motors released (silent).")
    else:
        print("  WARNING: No RELEASE confirmation.")


def ensure_engaged():
    global motors_engaged, stopped
    if motors_engaged and not stopped:
        return True
    if send_command("RESUME", "RESUMED"):
        motors_engaged = True
        stopped = False
        return True
    print("ERROR: Could not engage motors.")
    return False


def set_pen(angle):
    label = 'UP' if angle == PEN_UP else 'DOWN'
    print(f"  PEN: {label}")
    if send_command(f"P:{angle}", "P_OK", timeout=3):
        return True
    print("  WARNING: No servo confirmation")
    return False


def xy_to_counts(x, y, z_i):
    z_l = math.sqrt(x**2 + y**2)
    z_r = math.sqrt((BASE_WIDTH - x)**2 + y**2)
    theta_l = 2.0 * (z_l - z_i[0]) / R_SPOOL
    theta_r = 2.0 * (z_r - z_i[1]) / R_SPOOL
    c_l = int(theta_l * COUNTS_PER_REV / (2 * math.pi))
    c_r = int(theta_r * COUNTS_PER_REV / (2 * math.pi))
    return (c_l, c_r) if MOTOR1_IS_LEFT else (c_r, c_l)


def move(c1, c2):
    if stopped:
        return False
    cmd = f"G:{c1},{c2}\n"
    print(f"  >> {cmd.strip()}", end="", flush=True)
    ser.write(cmd.encode())
    ser.flush()
    deadline = time.time() + MOVE_TIMEOUT
    while time.time() < deadline:
        if check_stop():
            return False
        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if line == "D":
                print("  OK")
                return True
            elif line.startswith("DBG:"):
                pass
            elif "STOPPED" in line or line.startswith("ERROR:"):
                print(f"  {line}")
                return False
        time.sleep(0.01)
    print("  TIMEOUT!")
    return False


# Load waypoints from file — supports comments with #
def load_path(filename="path.txt"):
    try:
        waypoints = []
        with open(filename, 'r') as f:
            for i, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                if len(parts) != 2:
                    print(f"  WARNING: Skipping line {i}: '{line}'")
                    continue
                x, y = float(parts[0].strip()), float(parts[1].strip())
                waypoints.append((x, y))
        if not waypoints:
            print("  ERROR: No valid waypoints found.")
            return None
        print(f"  Loaded {len(waypoints)} waypoints from {filename}")
        return waypoints
    except FileNotFoundError:
        print(f"  ERROR: {filename} not found.")
        return None
    except ValueError as e:
        print(f"  ERROR: Bad number format — {e}")
        return None


def run_waypoints(waypoints, z_i, start_idx=0):
    if not ensure_engaged():
        return start_idx

    if start_idx == 0 and len(waypoints) > 0:
        set_pen(PEN_UP)
        x, y = waypoints[0]
        c1, c2 = xy_to_counts(x, y, z_i)
        print(f"[1/{len(waypoints)}] Move to start ({x},{y}) -> M1={c1}, M2={c2}")
        if not move(c1, c2):
            set_pen(PEN_UP)
            return 0
        time.sleep(0.1)
        set_pen(PEN_DOWN)
        start_idx = 1

    for i in range(start_idx, len(waypoints)):
        if stopped or check_stop():
            set_pen(PEN_UP)
            return i
        x, y = waypoints[i]
        c1, c2 = xy_to_counts(x, y, z_i)
        print(f"[{i+1}/{len(waypoints)}] ({x},{y}) -> M1={c1}, M2={c2}")
        if not move(c1, c2):
            set_pen(PEN_UP)
            return i
        time.sleep(0.1)

    set_pen(PEN_UP)
    return -1


def go_to_center(z_i):
    cx, cy = BASE_WIDTH / 2, BASE_WIDTH / 2
    print(f"Going to center ({cx:.3f}, {cy:.3f})...")
    if not ensure_engaged():
        return
    set_pen(PEN_UP)
    c1, c2 = xy_to_counts(cx, cy, z_i)
    move(c1, c2)
    release_motors()


if __name__ == "__main__":
    try:
        if not connect():
            sys.exit(1)

        z_i = calibrate()

        while True:
            print("\n" + "=" * 36)
            print("  [1] Run Waypoints (from path.txt)")
            print("  [2] Go to Center")
            print("  [3] Release Motors (stop noise)")
            print("  [4] Recalibrate")
            print("  [Q] Quit")
            print("=" * 36)
            choice = input("> ").strip().upper()

            if choice == '1':
                waypoints = load_path()
                if waypoints is None:
                    continue
                print("Press 'S' at any time for EMERGENCY STOP\n")
                idx = run_waypoints(waypoints, z_i)
                while stopped and 0 <= idx < len(waypoints):
                    print(f"\nStopped at waypoint {idx+1}/{len(waypoints)}.")
                    r = input("[R] Resume  [Q] Cancel > ").strip().upper()
                    if r == 'R':
                        stopped = False
                        idx = run_waypoints(waypoints, z_i, idx)
                    else:
                        break
                if idx == -1:
                    print("\nAll waypoints reached!")
                release_motors()

            elif choice == '2':
                go_to_center(z_i)

            elif choice == '3':
                release_motors()

            elif choice == '4':
                z_i = calibrate()

            elif choice == 'Q':
                break

    finally:
        if ser and ser.is_open:
            try:
                release_motors()
            except Exception:
                pass
            ser.close()
            print("Serial closed.")