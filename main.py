import serial
import time
import math
import sys
import msvcrt

# ============================================================
# Physical Constants
# ============================================================
COM_PORT       = 'COM4'
BAUD_RATE      = 115200
BASE_WIDTH     = 0.59
L_ARM          = 0.075
PEN_UP_ANGLE   = 44
PEN_DOWN_ANGLE = 9

MOVE_TIMEOUT   = 60

ser = None
stopped = False
motors_engaged = False
arduino_buf_count = 0


# ============================================================
# Emergency Stop
# ============================================================
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


# ============================================================
# Connection
# ============================================================
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
        print("ERROR: No READY from Arduino!")
        return False
    except Exception as e:
        print(f"Connection Error: {e}")
        return False


# ============================================================
# Send command and wait for acknowledgment
# ============================================================
def send_command(cmd, ack, timeout=5):
    ser.reset_input_buffer()
    ser.write(f"{cmd}\n".encode())
    ser.flush()
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if line.startswith(ack):
                return line
        time.sleep(0.01)
    return None


# ============================================================
# Drain serial — read all pending lines, update buffer count
# ============================================================
def drain_serial():
    global arduino_buf_count
    lines = []
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue
        if line.startswith("BUF:"):
            try:
                arduino_buf_count = int(line[4:])
            except ValueError:
                pass
        elif line.startswith("Q_OK:"):
            try:
                arduino_buf_count = int(line[5:])
            except ValueError:
                pass
        lines.append(line)
    return lines


# ============================================================
# Calibration
# ============================================================
def calibrate():
    global motors_engaged
    print("=== Calibration ===")
    print("Measure string length from each motor spool to the pulley.")
    try:
        l1 = float(input("  L1 - left string (meters):  "))
        l2 = float(input("  L2 - right string (meters): "))
    except ValueError:
        print("Invalid input!")
        return None

    z1 = l1 + L_ARM
    z2 = l2 + L_ARM

    x_init = (z1**2 - z2**2 + BASE_WIDTH**2) / (2 * BASE_WIDTH)
    y_sq = z1**2 - x_init**2
    if y_sq < 0:
        print("ERROR: Impossible string lengths!")
        return None
    y_init = math.sqrt(y_sq)

    print(f"  Cable lengths: Z1={z1:.4f}m, Z2={z2:.4f}m")
    print(f"  Computed position: X={x_init:.4f}m, Y={y_init:.4f}m")

    response = send_command(f"CAL:{x_init:.5f},{y_init:.5f}", "CALIBRATED", timeout=5)
    if response:
        motors_engaged = True
        print(f"  Arduino: {response}")
        print("  Calibration complete!\n")
        return (x_init, y_init)
    else:
        print("  WARNING: No calibration acknowledgment!")
        return None


# ============================================================
# Pen Control
# ============================================================
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
                elif line.startswith("Q_OK"):
                    pass
            time.sleep(0.01)
        print(f"  Pen {label} retry {attempt+1}...")
        ser.reset_input_buffer()
    print(f"  WARNING: Pen {label} failed!")
    return False


# ============================================================
# Ensure motors engaged
# ============================================================
def ensure_engaged():
    global motors_engaged, stopped
    if motors_engaged and not stopped:
        return True
    response = send_command("RESUME", "RESUMED")
    if response:
        motors_engaged = True
        stopped = False
        return True
    print("ERROR: Could not engage motors.")
    return False


# ============================================================
# Send a move command with flow control
# ============================================================
def send_move(x, y):
    global arduino_buf_count

    if stopped:
        return False

    cmd = f"G:{x:.5f},{y:.5f}\n"
    max_retries = 50
    retry_count = 0
    cmd_sent = False

    while retry_count < max_retries:
        if check_stop():
            return False

        # Wait if buffer is full
        if not cmd_sent and arduino_buf_count >= 20:
            if ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                if line.startswith("BUF:") or line.startswith("Q_OK:"):
                    try:
                        arduino_buf_count = int(line.split(":")[1])
                    except (ValueError, IndexError):
                        pass
                elif line == "D":
                    arduino_buf_count = max(0, arduino_buf_count - 1)
            time.sleep(0.02)
            continue

        if not cmd_sent:
            ser.write(cmd.encode())
            ser.flush()
            cmd_sent = True

        # Wait for Q_OK
        deadline = time.time() + 10
        while time.time() < deadline:
            if check_stop():
                return False
            if ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                if line.startswith("Q_OK:"):
                    try:
                        arduino_buf_count = int(line[5:])
                    except ValueError:
                        pass
                    return True
                elif line == "ERROR:BUF_FULL":
                    retry_count += 1
                    cmd_sent = False
                    time.sleep(0.1)
                    break
                elif line == "D":
                    arduino_buf_count = max(0, arduino_buf_count - 1)
                elif line == "P_OK":
                    pass
                elif line.startswith("BUF:"):
                    try:
                        arduino_buf_count = int(line[4:])
                    except ValueError:
                        pass
                elif "ERROR" in line or "STOPPED" in line:
                    print(f"\n  Arduino error: {line}")
                    return False
            time.sleep(0.005)
        else:
            print(f"\n  TIMEOUT sending G:{x:.4f},{y:.4f}")
            return False

    print(f"\n  FAILED after {max_retries} retries")
    return False


# ============================================================
# Wait for all queued moves to finish
# ============================================================
def wait_all_done(timeout=120):
    global arduino_buf_count

    drain_serial()
    if arduino_buf_count == 0 and not stopped:
        return True

    deadline = time.time() + timeout
    while time.time() < deadline:
        if check_stop():
            return False
        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            if line == "D":
                drain_serial()
                if arduino_buf_count == 0:
                    return True
            elif line.startswith("BUF:"):
                try:
                    arduino_buf_count = int(line[4:])
                except ValueError:
                    pass
                if arduino_buf_count == 0:
                    return True
            elif "ERROR" in line or "STOPPED" in line:
                print(f"\n  Arduino: {line}")
                return False
        time.sleep(0.01)

    print("  WARNING: wait_all_done timeout!")
    return False


# ============================================================
# Load path file
# ============================================================
def load_path(filename="path.txt"):
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
                else:
                    # Try plain x,y format
                    parts = line.split(',')
                    if len(parts) == 2:
                        try:
                            commands.append(("MOVE", float(parts[0]), float(parts[1])))
                        except ValueError:
                            pass
        print(f"  Loaded {len(commands)} commands from {filename}")
        return commands
    except Exception as e:
        print(f"Error loading path: {e}")
        return None


# ============================================================
# Main Drawing Sequence
# ============================================================
def run_draw_sequence(commands):
    global stopped, arduino_buf_count

    if not ensure_engaged():
        return

    total = len(commands)
    move_count = sum(1 for c in commands if c[0] == "MOVE")
    print(f"Starting drawing: {total} commands, {move_count} moves")
    print("Press 'S' at any time for EMERGENCY STOP\n")

    done_moves = 0
    i = 0

    while i < total:
        if stopped or check_stop():
            set_pen(PEN_UP_ANGLE)
            print("\nStopped by user.")
            return

        cmd_type = commands[i][0]

        if cmd_type == "PEN":
            # Wait for all moves to finish before pen change
            wait_all_done(timeout=60)

            angle = commands[i][1]
            label = "UP" if angle == PEN_UP_ANGLE else "DOWN"
            print(f"\n  PEN {label}", flush=True)

            ser.write(f"P:{angle}\n".encode())
            ser.flush()

            deadline = time.time() + 5
            got_ok = False
            while time.time() < deadline:
                if ser.in_waiting:
                    line = ser.readline().decode(errors='ignore').strip()
                    if line == "P_OK":
                        got_ok = True
                        break
                    elif line.startswith("Q_OK"):
                        pass
                time.sleep(0.01)

            if not got_ok:
                print("  WARNING: No pen confirmation!")

            time.sleep(0.2)
            i += 1

        elif cmd_type == "MOVE":
            x, y = commands[i][1], commands[i][2]
            done_moves += 1

            if done_moves % 20 == 0 or done_moves <= 3:
                print(f"\r  [{done_moves}/{move_count}] ({x:.4f},{y:.4f})     ", end="", flush=True)

            if not send_move(x, y):
                print(f"\n  FAILED at move {done_moves}")
                set_pen(PEN_UP_ANGLE)
                return

            i += 1
            drain_serial()

    # Wait for remaining moves
    print("\n\n  Waiting for remaining moves...")
    wait_all_done(timeout=120)

    set_pen(PEN_UP_ANGLE)
    print(f"\n  Drawing Complete! ({done_moves} moves)")


# ============================================================
# Release Motors
# ============================================================
def release_motors():
    global motors_engaged
    response = send_command("RELEASE", "RELEASED")
    if response:
        motors_engaged = False
        print("  Motors released.")
    else:
        print("  WARNING: No release confirmation.")


# ============================================================
# Main Menu
# ============================================================
if __name__ == "__main__":
    if not connect():
        sys.exit(1)

    init_pos = calibrate()

    while True:
        print("\n" + "=" * 40)
        print("  [1] Load & Draw 'path.txt'")
        print("  [2] Release Motors")
        print("  [3] Recalibrate")
        print("  [4] Query Position")
        print("  [Q] Quit")
        print("=" * 40)
        c = input("> ").upper().strip()

        if c == '1':
            cmds = load_path("path.txt")
            if cmds and init_pos:
                run_draw_sequence(cmds)
                release_motors()
        elif c == '2':
            release_motors()
        elif c == '3':
            init_pos = calibrate()
        elif c == '4':
            response = send_command("POS", "POS:", timeout=3)
            if response:
                print(f"  Arduino reports: {response}")
            else:
                print("  No response.")
        elif c == 'Q':
            break

    if ser and ser.is_open:
        try:
            release_motors()
        except Exception:
            pass
        ser.close()
        print("Serial closed.")