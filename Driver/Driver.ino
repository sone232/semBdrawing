#include <ArduinoMotorCarrier.h>
#include <math.h>

// ============================================================
// Physical Constants
// ============================================================
const float BASE_WIDTH     = 0.59;
const float R_SPOOL        = 0.0045;
const float COUNTS_PER_REV = 1200.0;

const float COUNTS_PER_METER = COUNTS_PER_REV / (PI * R_SPOOL);

// Motion Profile
const float MAX_SPEED_DRAW   = 0.025;   // m/s pen down
const float MAX_SPEED_TRAVEL = 0.07;   // m/s pen up
const float MAX_ACCEL        = 1.0;    // m/s^2

// PID
const float Kp = 0.18;
const float Ki = 0.0;
const float Kd = 0.015;

// Timing
const unsigned long CONTROL_INTERVAL_US = 10000; // was 20ms

// Servo
const int PEN_UP_ANGLE   = 44;
const int PEN_DOWN_ANGLE = 9;

// Safety
const long MAX_SPEED_COUNTS = 5000;

// Settling
const long SETTLE_THRESHOLD    = 120;  // was 150
const int  SETTLE_NEEDED       = 5;
const unsigned long SETTLE_TIMEOUT_US = 5000000; // 5s

// Corner Detection
const float CORNER_DOT_THRESHOLD = 0.7;  // dot product below this = sharp corner

// ============================================================
// Motion States
// ============================================================
enum MoveState { IDLE, TRAJECTORY, SETTLING, DRAWING };
MoveState moveState = IDLE;

int settleCount = 0;
unsigned long settleStartTime = 0;
long settleTargetC1 = 0, settleTargetC2 = 0;

// After settling from a sharp corner, should we load next segment?
bool cornerPending = false;
float cornerNextX = 0.0, cornerNextY = 0.0;

// ============================================================
// Command Buffer
// ============================================================
enum CmdType { CMD_MOVE, CMD_PEN };

struct MotionCommand {
  CmdType type;
  float x;
  float y;
  int penAngle;
};

const int BUF_SIZE = 24;
MotionCommand cmdBuf[BUF_SIZE];
int bufHead  = 0;
int bufTail  = 0;
int bufCount = 0;

bool bufferFull()  { return bufCount >= BUF_SIZE; }
bool bufferEmpty() { return bufCount == 0; }

bool pushCmd(MotionCommand cmd) {
  if (bufferFull()) return false;
  cmdBuf[bufHead] = cmd;
  bufHead = (bufHead + 1) % BUF_SIZE;
  bufCount++;
  return true;
}

MotionCommand peekCmd() {
  return cmdBuf[bufTail];
}

MotionCommand popCmd() {
  MotionCommand cmd = cmdBuf[bufTail];
  bufTail = (bufTail + 1) % BUF_SIZE;
  bufCount--;
  return cmd;
}

// ============================================================
// System State
// ============================================================
float currentX = 0.0, currentY = 0.0;
float startX   = 0.0, startY   = 0.0;
float moveTargetX = 0.0, moveTargetY = 0.0;

// Single-segment trajectory vars
float moveTotalDist = 0.0;
float tAccel = 0.0, tCruise = 0.0, tDecel = 0.0, tTotal = 0.0;
float dAccel = 0.0, dCruise = 0.0, vPeak = 0.0;
float moveAccel = 0.0;

// Drawing mode vars
float drawSpeed     = 0.0;
float drawDist      = 0.0;
float drawSegDist   = 0.0;
float drawDirX      = 0.0, drawDirY = 0.0;

float calibL1 = 0.0;
float calibL2 = 0.0;

bool isMoving     = false;
bool isStopped    = false;
bool isReleased   = false;
bool isCalibrated = false;
bool penIsDown    = false;

unsigned long moveStartTime   = 0;
unsigned long lastControlTime = 0;

long prevEnc1 = 0, prevEnc2 = 0;
String inputBuffer = "";

// ============================================================
// Inverse Kinematics: XY → encoder counts
// ============================================================
long xyToCount1(float x, float y) {
  float L = sqrt(x * x + y * y);
  return (long)((L - calibL1) * COUNTS_PER_METER);
}

long xyToCount2(float x, float y) {
  float dx = BASE_WIDTH - x;
  float L = sqrt(dx * dx + y * y);
  return (long)((L - calibL2) * COUNTS_PER_METER);
}

// ============================================================
// Forward Kinematics: encoder counts → XY
// ============================================================
void updatePositionFromEncoders() {
  long enc1 = encoder1.getRawCount();
  long enc2 = encoder2.getRawCount();

  float L_left  = calibL1 + (float)enc2 / COUNTS_PER_METER;
  float L_right = calibL2 + (float)enc1 / COUNTS_PER_METER;

  float x = (L_left * L_left - L_right * L_right + BASE_WIDTH * BASE_WIDTH)
            / (2.0 * BASE_WIDTH);
  float y_sq = L_left * L_left - x * x;
  if (y_sq > 0) {
    currentX = x;
    currentY = sqrt(y_sq);
  }
}

// ============================================================
// Motor Control
// ============================================================
void setPidTargets(long c1, long c2) {
  pid1.setSetpoint(TARGET_POSITION, c2);
  pid2.setSetpoint(TARGET_POSITION, c1);
}

void engageMotors() {
  pid1.setControlMode(CL_POSITION);
  pid1.setGains(Kp, Ki, Kd);
  pid2.setControlMode(CL_POSITION);
  pid2.setGains(Kp, Ki, Kd);

  long c1 = encoder1.getRawCount();
  long c2 = encoder2.getRawCount();
  pid1.setSetpoint(TARGET_POSITION, c1);
  pid2.setSetpoint(TARGET_POSITION, c2);

  prevEnc1 = c1;
  prevEnc2 = c2;
  isReleased = false;
  isStopped  = false;
  moveState  = IDLE;
  isMoving   = false;
}

void releaseMotors() {
  pid1.setControlMode(CL_OPEN_LOOP);
  pid2.setControlMode(CL_OPEN_LOOP);
  M1.setDuty(0);
  M2.setDuty(0);
  isReleased = true;
  isMoving   = false;
  moveState  = IDLE;
  Serial.println("RELEASED");
}

void emergencyStop() {
  servo3.setAngle(PEN_UP_ANGLE);
  M1.setDuty(0);
  M2.setDuty(0);
  isMoving   = false;
  isStopped  = true;
  penIsDown  = false;
  moveState  = IDLE;
  cornerPending = false;
  bufHead = 0; bufTail = 0; bufCount = 0;
  Serial.println("STOPPED");
}

bool checkOverspeed() {
  long cur1 = encoder1.getRawCount();
  long cur2 = encoder2.getRawCount();
  long spd1 = labs(cur1 - prevEnc1);
  long spd2 = labs(cur2 - prevEnc2);
  prevEnc1 = cur1;
  prevEnc2 = cur2;
  if (spd1 > MAX_SPEED_COUNTS || spd2 > MAX_SPEED_COUNTS) {
    emergencyStop();
    Serial.println("ERROR:OVERSPEED");
    return true;
  }
  return false;
}

// ============================================================
// TRAVEL mode: full trapezoidal profile (pen up)
// ============================================================
void beginTravel(float tx, float ty) {
  startX = currentX; startY = currentY;
  moveTargetX = tx; moveTargetY = ty;
  float dx = tx - startX;
  float dy = ty - startY;
  moveTotalDist = sqrt(dx * dx + dy * dy);

  if (moveTotalDist < 0.0002) {
    currentX = tx; currentY = ty;
    moveState = IDLE;
    isMoving = false;
    Serial.println("D");
    return;
  }

  float vmax = penIsDown ? MAX_SPEED_DRAW : MAX_SPEED_TRAVEL;
  moveAccel = MAX_ACCEL;
  tAccel = vmax / moveAccel;
  dAccel = 0.5 * moveAccel * tAccel * tAccel;

  if (2.0 * dAccel > moveTotalDist) {
    tAccel = sqrt(moveTotalDist / moveAccel);
    vPeak  = moveAccel * tAccel;
    dAccel = 0.5 * moveAccel * tAccel * tAccel;
    tCruise = 0.0;
    dCruise = 0.0;
    tDecel  = tAccel;
    tTotal  = tAccel + tDecel;
  } else {
    vPeak   = vmax;
    dCruise = moveTotalDist - 2.0 * dAccel;
    tCruise = dCruise / vPeak;
    tDecel  = tAccel;
    tTotal  = tAccel + tCruise + tDecel;
  }

  moveStartTime = micros();
  moveState = TRAJECTORY;
  isMoving = true;
}

void updateTrajectory() {
  float t = (float)(micros() - moveStartTime) / 1000000.0;
  if (t >= tTotal) t = tTotal;

  float d = 0.0;
  if (t < tAccel) {
    d = 0.5 * moveAccel * t * t;
  } else if (t < (tAccel + tCruise)) {
    d = dAccel + vPeak * (t - tAccel);
  } else {
    float tr = tTotal - t;
    d = moveTotalDist - 0.5 * moveAccel * tr * tr;
  }
  if (d > moveTotalDist) d = moveTotalDist;

  float ratio = d / moveTotalDist;
  float x = startX + ratio * (moveTargetX - startX);
  float y = startY + ratio * (moveTargetY - startY);

  long c1 = xyToCount1(x, y);
  long c2 = xyToCount2(x, y);
  setPidTargets(c1, c2);

  if (t >= tTotal) {
    settleTargetC1 = c1;
    settleTargetC2 = c2;
    settleCount = 0;
    settleStartTime = micros();
    moveState = SETTLING;
  }
}

// ============================================================
// SETTLING: wait for motors to physically arrive
// ============================================================
void updateSettling() {
  long enc1 = encoder1.getRawCount();
  long enc2 = encoder2.getRawCount();

  long err1 = labs(enc1 - settleTargetC2);
  long err2 = labs(enc2 - settleTargetC1);

  if (err1 < SETTLE_THRESHOLD && err2 < SETTLE_THRESHOLD) {
    settleCount++;
    if (settleCount >= SETTLE_NEEDED) {
      updatePositionFromEncoders();
      isMoving = false;
      moveState = IDLE;
      Serial.println("D");

      // If we got here from a sharp corner, start the pending next segment
      if (cornerPending) {
        cornerPending = false;
        Serial.print("BUF:"); Serial.println(bufCount);
        beginDrawSegment(cornerNextX, cornerNextY);
        return;
      }

      loadNextCommand();
      return;
    }
  } else {
    settleCount = 0;
  }

  if (micros() - settleStartTime > SETTLE_TIMEOUT_US) {
    updatePositionFromEncoders();
    isMoving = false;
    moveState = IDLE;
    Serial.println("ERROR:SETTLE_TIMEOUT");
    Serial.print("SETTLE_ERR:");
    Serial.print(err1); Serial.print(","); Serial.println(err2);

    // Even on timeout, try to continue if corner was pending
    if (cornerPending) {
      cornerPending = false;
      beginDrawSegment(cornerNextX, cornerNextY);
      return;
    }

    loadNextCommand();
  }
}

// ============================================================
// DRAWING mode: continuous motion with smart cornering
// ============================================================
void beginDrawSegment(float tx, float ty) {
  startX = currentX; startY = currentY;
  moveTargetX = tx; moveTargetY = ty;

  float dx = tx - startX;
  float dy = ty - startY;
  drawSegDist = sqrt(dx * dx + dy * dy);

  if (drawSegDist < 0.0001) {
    currentX = tx; currentY = ty;
    Serial.println("D");
    Serial.print("BUF:"); Serial.println(bufCount);
    loadNextCommand();
    return;
  }

  drawDirX = dx / drawSegDist;
  drawDirY = dy / drawSegDist;
  drawDist = 0.0;
  moveStartTime = micros();
  moveState = DRAWING;
  isMoving = true;
}

bool nextCmdIsMove() {
  if (bufferEmpty()) return false;
  return peekCmd().type == CMD_MOVE;
}

// ============================================================
// Look-ahead: compute dot product with next segment
// Returns 1.0 if no next move (treat as "straight ahead" for decel)
// ============================================================
float getNextSegmentDot() {
  if (!nextCmdIsMove()) return -1.0;  // no next move → force stop

  MotionCommand next = peekCmd();
  float dx = next.x - moveTargetX;
  float dy = next.y - moveTargetY;
  float len = sqrt(dx * dx + dy * dy);

  if (len < 0.0001) return 1.0;  // zero-length next segment, treat as straight

  float ndx = dx / len;
  float ndy = dy / len;

  // Dot product: current direction · next direction
  return drawDirX * ndx + drawDirY * ndy;
}

// ============================================================
// Compute remaining distance for deceleration planning
// SMART: stops counting at sharp corners
// ============================================================
float computeRemainingDist() {
  float remaining = drawSegDist - drawDist;

  // Look ahead in buffer, but STOP at sharp corners
  float prevDirX = drawDirX;
  float prevDirY = drawDirY;
  float prevX = moveTargetX;
  float prevY = moveTargetY;

  int idx = bufTail;
  int cnt = bufCount;

  while (cnt > 0) {
    MotionCommand cmd = cmdBuf[idx];
    if (cmd.type != CMD_MOVE) break;  // stop at pen command

    float dx = cmd.x - prevX;
    float dy = cmd.y - prevY;
    float segLen = sqrt(dx * dx + dy * dy);

    if (segLen < 0.0001) {
      // Skip zero-length segments
      prevX = cmd.x;
      prevY = cmd.y;
      idx = (idx + 1) % BUF_SIZE;
      cnt--;
      continue;
    }

    // Direction of this upcoming segment
    float ndx = dx / segLen;
    float ndy = dy / segLen;

    // Dot product with previous segment direction
    float dot = prevDirX * ndx + prevDirY * ndy;

    if (dot < CORNER_DOT_THRESHOLD) {
      // Sharp corner ahead — don't count distance beyond it
      // The robot must decelerate to 0 by the end of current remaining
      break;
    }

    // Shallow angle — include this segment in remaining distance
    remaining += segLen;

    prevDirX = ndx;
    prevDirY = ndy;
    prevX = cmd.x;
    prevY = cmd.y;
    idx = (idx + 1) % BUF_SIZE;
    cnt--;
  }

  return remaining;
}

// ============================================================
// Main drawing update — called every control cycle
// ============================================================
void updateDrawing() {
  float dt = (float)(micros() - moveStartTime) / 1000000.0;
  moveStartTime = micros();

  // Compute remaining distance (smart — stops at sharp corners)
  float remaining = computeRemainingDist();

  // Speed limit from deceleration constraint: v² = 2·a·d
  float vMaxForStop = sqrt(2.0 * MAX_ACCEL * remaining);

  // Target speed
  float vTarget = MAX_SPEED_DRAW;
  if (vMaxForStop < vTarget) vTarget = vMaxForStop;

  // Accelerate or decelerate toward target speed
  if (drawSpeed < vTarget) {
    drawSpeed += MAX_ACCEL * dt;
    if (drawSpeed > vTarget) drawSpeed = vTarget;
  } else if (drawSpeed > vTarget) {
    drawSpeed -= MAX_ACCEL * dt;
    if (drawSpeed < 0.0) drawSpeed = 0.0;
  }

  // Minimum speed to avoid stalling (only if we still have distance)
  if (drawSpeed < 0.002 && remaining > 0.001) drawSpeed = 0.002;

  // Advance along current segment
  float step = drawSpeed * dt;
  drawDist += step;

  // ---- Segment transition ----
  if (drawDist >= drawSegDist) {
    currentX = moveTargetX;
    currentY = moveTargetY;

    long c1 = xyToCount1(currentX, currentY);
    long c2 = xyToCount2(currentX, currentY);
    setPidTargets(c1, c2);

    Serial.println("D");
    Serial.print("BUF:"); Serial.println(bufCount);

    float overshoot = drawDist - drawSegDist;

    // Is next command a MOVE?
    if (nextCmdIsMove()) {
      // Calculate corner angle via dot product
      float dot = getNextSegmentDot();

      if (dot >= CORNER_DOT_THRESHOLD) {
        // ---- SHALLOW ANGLE: blend seamlessly ----
        MotionCommand next = popCmd();
        Serial.print("BUF:"); Serial.println(bufCount);

        startX = currentX; startY = currentY;
        moveTargetX = next.x; moveTargetY = next.y;
        float dx = next.x - startX;
        float dy = next.y - startY;
        drawSegDist = sqrt(dx * dx + dy * dy);

        if (drawSegDist < 0.0001) {
          currentX = next.x; currentY = next.y;
          Serial.println("D");
          Serial.print("BUF:"); Serial.println(bufCount);
          loadNextCommand();
          return;
        }

        drawDirX = dx / drawSegDist;
        drawDirY = dy / drawSegDist;
        drawDist = overshoot;
        // drawSpeed carries over — smooth blending!
        return;

      } else {
        // ---- SHARP CORNER: stop, settle, then continue ----
        MotionCommand next = popCmd();
        Serial.print("BUF:"); Serial.println(bufCount);

        // Save next segment target for after settling
        cornerPending = true;
        cornerNextX = next.x;
        cornerNextY = next.y;

        // Enter settling to stabilize at the corner point
        settleTargetC1 = c1;
        settleTargetC2 = c2;
        settleCount = 0;
        settleStartTime = micros();
        drawSpeed = 0.0;
        moveState = SETTLING;
        return;
      }
    }

    // No more moves in buffer — final point, settle
    settleTargetC1 = xyToCount1(currentX, currentY);
    settleTargetC2 = xyToCount2(currentX, currentY);
    settleCount = 0;
    settleStartTime = micros();
    drawSpeed = 0.0;
    moveState = SETTLING;
    cornerPending = false;
    return;
  }

  // ---- Normal mid-segment: interpolate position ----
  float x = startX + drawDist * drawDirX;
  float y = startY + drawDist * drawDirY;

  long c1 = xyToCount1(x, y);
  long c2 = xyToCount2(x, y);
  setPidTargets(c1, c2);
}

// ============================================================
// Command Loading
// ============================================================
void loadNextCommand() {
  while (!bufferEmpty()) {
    MotionCommand cmd = popCmd();
    Serial.print("BUF:"); Serial.println(bufCount);

    if (cmd.type == CMD_PEN) {
      servo3.setAngle(cmd.penAngle);
      bool wasDown = penIsDown;
      penIsDown = (cmd.penAngle == PEN_DOWN_ANGLE);
      delay(250);
      lastControlTime = micros();
      prevEnc1 = encoder1.getRawCount();
      prevEnc2 = encoder2.getRawCount();

      updatePositionFromEncoders();
      drawSpeed = 0.0;

      Serial.println("P_OK");
    }
    else if (cmd.type == CMD_MOVE) {
      if (penIsDown) {
        beginDrawSegment(cmd.x, cmd.y);
      } else {
        beginTravel(cmd.x, cmd.y);
      }
      if (moveState != IDLE) return;
    }
  }
}

// ============================================================
// Serial Parser
// ============================================================
void parseSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '!') {
      emergencyStop();
      inputBuffer = "";
      continue;
    }
    if (c == '\r') continue;
    if (c == '\n') {
      inputBuffer.trim();

      if (inputBuffer == "RESUME") {
        engageMotors();
        Serial.println("RESUMED");
      }
      else if (inputBuffer == "RELEASE") {
        releaseMotors();
      }
      else if (inputBuffer.startsWith("CAL:")) {
        int comma = inputBuffer.indexOf(',', 4);
        if (comma > 4) {
          float xi = inputBuffer.substring(4, comma).toFloat();
          float yi = inputBuffer.substring(comma + 1).toFloat();
          calibL1 = sqrt(xi * xi + yi * yi);
          calibL2 = sqrt((BASE_WIDTH - xi) * (BASE_WIDTH - xi) + yi * yi);

          pid1.setControlMode(CL_OPEN_LOOP);
          pid2.setControlMode(CL_OPEN_LOOP);
          M1.setDuty(0);
          M2.setDuty(0);
          encoder1.resetCounter(0);
          encoder2.resetCounter(0);
          currentX = xi; currentY = yi;
          engageMotors();
          isCalibrated = true;
          drawSpeed = 0.0;
          cornerPending = false;

          Serial.print("CALIBRATED:");
          Serial.print(xi, 4);
          Serial.print(",");
          Serial.println(yi, 4);
        }
      }
      else if (inputBuffer.startsWith("G:")) {
        if (!isCalibrated) {
          Serial.println("ERROR:NOT_READY");
        } else if (isStopped) {
          Serial.println("ERROR:STOPPED");
        } else if (bufferFull()) {
          Serial.println("ERROR:BUF_FULL");
        } else {
          int comma = inputBuffer.indexOf(',', 2);
          if (comma > 2) {
            MotionCommand cmd;
            cmd.type = CMD_MOVE;
            cmd.x = inputBuffer.substring(2, comma).toFloat();
            cmd.y = inputBuffer.substring(comma + 1).toFloat();
            pushCmd(cmd);
            Serial.print("Q_OK:");
            Serial.println(bufCount);
            if (moveState == IDLE && !isMoving) loadNextCommand();
          }
        }
      }
      else if (inputBuffer.startsWith("P:")) {
        if (bufferFull()) {
          Serial.println("ERROR:BUF_FULL");
        } else {
          MotionCommand cmd;
          cmd.type = CMD_PEN;
          cmd.penAngle = inputBuffer.substring(2).toInt();
          pushCmd(cmd);
          Serial.print("Q_OK:");
          Serial.println(bufCount);
          if (moveState == IDLE && !isMoving) loadNextCommand();
        }
      }
      else if (inputBuffer == "POS") {
        Serial.print("POS:");
        Serial.print(currentX, 5);
        Serial.print(",");
        Serial.println(currentY, 5);
      }

      inputBuffer = "";
    } else if (inputBuffer.length() < 60) {
      inputBuffer += c;
    }
  }
}

// ============================================================
// Setup & Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!controller.begin()) {
    Serial.println("ERROR:CARRIER_FAIL");
    while (1);
  }

  M1.setDuty(0);
  M2.setDuty(0);
  delay(100);
  encoder1.resetCounter(0);
  encoder2.resetCounter(0);
  engageMotors();
  servo3.setAngle(PEN_UP_ANGLE);
  Serial.println("READY");
}

void loop() {
  parseSerial();

  unsigned long now = micros();
  if (now - lastControlTime >= CONTROL_INTERVAL_US) {
    lastControlTime = now;

    if (!isReleased && !isStopped) {
      if (checkOverspeed()) return;
    }

    if (!isStopped && !isReleased && isCalibrated) {
      switch (moveState) {
        case TRAJECTORY:
          updateTrajectory();
          break;
        case SETTLING:
          updateSettling();
          break;
        case DRAWING:
          updateDrawing();
          break;
        case IDLE:
          break;
      }
    }
  }
}