import serial
import time
import math
import sys
import msvcrt

# === הגדרות פיזיות ===
COM_PORT = 'COM4'
BAUD_RATE = 115200
BASE_WIDTH = 0.59
R_SPOOL = 0.0045
COUNTS_PER_REV = 1200
L_ARM = 0.075
MOTOR1_IS_LEFT = False
MOVE_TIMEOUT = 50
PEN_UP_ANGLE = 44
PEN_DOWN_ANGLE = 9
MIN_COUNT_DELTA = 30

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
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        ser.setDTR(False)
        time.sleep(0.1)
        ser.setDTR(True)
        print("Waiting for Arduino...")
        deadline = time.time() + 10
        while time.time() < deadline:
            if ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                if line in ("READY", "RESUMED"):
                    ser.reset_input_buffer()
                    print("Connected!\n")
                    return True
        print("ERROR: No READY!")
        return False
    except Exception as e:
        print(f"Connection Error: {e}")
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
    try:
        l1 = float(input("  L1 - left string (meters):  "))
        l2 = float(input("  L2 - right string (meters): "))
    except ValueError:
        return None
    z_i = (l1 + L_ARM, l2 + L_ARM)
    try:
        x_init = (z_i[0]**2 - z_i[1]**2 + BASE_WIDTH**2) / (2 * BASE_WIDTH)
        y_sq = z_i[0]**2 - x_init**2
        if y_sq < 0:
            print("ERROR: Impossible dimensions!")
            return None
        y_init = math.sqrt(y_sq)
        print(f"  Z_i = ({z_i[0]:.4f}, {z_i[1]:.4f})")
        print(f"  Initial position: ({x_init:.4f}, {y_init:.4f})")
        if send_command("CALIBRATE", "CALIBRATED"):
            motors_engaged = True
            print("  Encoders zeroed.")
        return z_i
    except Exception as e:
        print(f"Calc Error: {e}")
        return None


def set_pen(angle):
    label = 'UP' if angle == PEN_UP_ANGLE else 'DOWN'
    ser.reset_input_buffer()

    for attempt in range(3):
        ser.write(f"P:{angle}\n".encode())
        ser.flush()

        deadline = time.time() + 3
        while time.time() < deadline:
            if ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                if line == "P_OK":
                    return True
            time.sleep(0.01)

        print(f"\n  Pen retry {attempt+1}...")
        ser.reset_input_buffer()

    print(f"\n  WARNING: Pen {label} failed after 3 attempts!")
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

    ser.reset_input_buffer()

    cmd = f"G:{c1},{c2}"
    ser.write(f"{cmd}\n".encode())
    ser.flush()

    deadline = time.time() + MOVE_TIMEOUT
    while time.time() < deadline:
        if check_stop():
            return False

        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            if line == "D":
                return True
            elif "ERROR" in line or "STOPPED" in line:
                print(f"\n  Arduino error: {line}")
                return False
            elif line.startswith("DBG:") or line.startswith("TIMEOUT_DBG:"):
                print(f"\n    {line}", end="", flush=True)
            else:
                print(f"\n  Unexpected: {line}")

        time.sleep(0.01)

    print(f"\n  TIMEOUT waiting for move {cmd}")
    return False


def ensure_engaged():
    global motors_engaged, stopped
    if motors_engaged and not stopped:
        return True
    if send_command("RESUME", "RESUMED"):
        motors_engaged = True
        stopped = False
        return True
    return False


def load_path(filename="path.txt"):
    """קורא קובץ פקודות: G:x,y או PEN_UP או PEN_DOWN"""
    commands = []
    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                if line == "PEN_UP":
                    commands.append(("PEN", PEN_UP_ANGLE))
                elif line == "PEN_DOWN":
                    commands.append(("PEN", PEN_DOWN_ANGLE))
                elif line.startswith("G:"):
                    parts = line[2:].split(',')
                    if len(parts) == 2:
                        commands.append(("MOVE", float(parts[0]), float(parts[1])))
        print(f"  Loaded {len(commands)} commands.")
        return commands
    except Exception as e:
        print(f"Error loading path: {e}")
        return None


def run_draw_sequence(commands, z_i):
    global stopped
    if not ensure_engaged():
        return

    total = len(commands)
    move_count = sum(1 for c in commands if c[0] == "MOVE")
    print(f"Starting drawing: {total} commands, {move_count} moves")

    set_pen(PEN_UP_ANGLE)

    done_moves = 0
    skipped = 0
    last_c1, last_c2 = None, None

    for i, cmd in enumerate(commands):
        if stopped or check_stop():
            set_pen(PEN_UP_ANGLE)
            print("\nStopped by user.")
            return

        cmd_type, val1, *val2 = cmd

        if cmd_type == "PEN":
            label = "UP" if val1 == PEN_UP_ANGLE else "DOWN"
            print(f"\n  [{done_moves}/{move_count}] PEN {label}", flush=True)

            if not set_pen(val1):
                print("\nPen command failed! Aborting.")
                set_pen(PEN_UP_ANGLE)
                return
            time.sleep(0.3)

            # Reset last position on pen change to force first move
            if val1 == PEN_DOWN_ANGLE:
                last_c1, last_c2 = None, None

        elif cmd_type == "MOVE":
            x, y = val1, val2[0]
            c1, c2 = xy_to_counts(x, y, z_i)
            done_moves += 1

            # Skip if too close to previous position
            if last_c1 is not None:
                delta = abs(c1 - last_c1) + abs(c2 - last_c2)
                if delta < MIN_COUNT_DELTA:
                    skipped += 1
                    continue

            print(f"\r  [{done_moves}/{move_count}] G:{c1},{c2} (skip:{skipped})    ", end="", flush=True)

            if not move(c1, c2):
                print(f"\n  FAILED at move {done_moves}/{move_count}")
                print(f"  Target: ({x:.4f}, {y:.4f}) → counts ({c1}, {c2})")
                if last_c1 is not None:
                    print(f"  Previous counts: ({last_c1}, {last_c2})")
                    print(f"  Delta: ({c1 - last_c1}, {c2 - last_c2})")
                set_pen(PEN_UP_ANGLE)
                return

            last_c1, last_c2 = c1, c2
            time.sleep(0.05)

    set_pen(PEN_UP_ANGLE)
    print(f"\n\n  ✅ Drawing Complete! ({done_moves} moves, {skipped} skipped)")


def release_motors():
    send_command("RELEASE", "RELEASED")


# === Main Loop ===
if __name__ == "__main__":
    if not connect():
        sys.exit(1)
    z_i = calibrate()

    while True:
        print("\n[1] Load & Draw 'path.txt'")
        print("[2] Manual Release")
        print("[3] Recalibrate")
        print("[Q] Quit")
        c = input("> ").upper().strip()

        if c == '1':
            cmds = load_path("path.txt")
            if cmds and z_i:
                run_draw_sequence(cmds, z_i)
                release_motors()
        elif c == '2':
            release_motors()
        elif c == '3':
            z_i = calibrate()
        elif c == 'Q':
            break

    if ser:
        ser.close()