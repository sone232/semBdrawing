#include <ArduinoMotorCarrier.h>
#include <math.h>

// ============================================================
// Physical Constants
// ============================================================
const float BASE_WIDTH       = 0.59;
const float R_SPOOL          = 0.0045;
const float COUNTS_PER_REV   = 1200.0;

const float COUNTS_PER_METER = COUNTS_PER_REV / (PI * R_SPOOL);

// Motion Profile
const float MAX_SPEED_DRAW   = 0.05;   // m/s when pen is down
const float MAX_SPEED_TRAVEL = 0.10;   // m/s when pen is up
const float MAX_ACCEL        = 2.0;    // m/s^2

// PID gains
const float Kp = 0.18;
const float Ki = 0.0;
const float Kd = 0.01;

// Timing - הורדנו עומס ל-20ms (50Hz)
const unsigned long CONTROL_INTERVAL_US = 20000; 

// Servo angles
const int PEN_UP_ANGLE   = 44;
const int PEN_DOWN_ANGLE = 9;

// Overspeed protection
const long MAX_SPEED_COUNTS = 5000; // סף גבוה כדי למנוע עצירות שווא

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

// Trajectory vars
float moveTotalDist = 0.0;
float moveVmax      = 0.0;
float moveAccel     = 0.0;
float tAccel = 0.0, tCruise = 0.0, tDecel = 0.0, tTotal = 0.0;
float dAccel = 0.0, dCruise = 0.0, vPeak = 0.0;

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
// Inverse Kinematics
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
// Motor Control
// ============================================================
void setPidTargets(long c1, long c2) {
  // Motor 1 = Right Cable (c2), Motor 2 = Left Cable (c1)
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
}

void releaseMotors() {
  pid1.setControlMode(CL_OPEN_LOOP);
  pid2.setControlMode(CL_OPEN_LOOP);
  M1.setDuty(0); M2.setDuty(0);
  isReleased = true;
  isMoving   = false;
  Serial.println("RELEASED");
}

void emergencyStop() {
  servo3.setAngle(PEN_UP_ANGLE);
  M1.setDuty(0); M2.setDuty(0);
  isMoving  = false;
  isStopped = true;
  penIsDown = false;
  bufHead = 0; bufTail = 0; bufCount = 0;
  Serial.println("STOPPED");
}

bool checkOverspeed() {
  long cur1 = encoder1.getRawCount();
  long cur2 = encoder2.getRawCount();
  long spd1 = labs(cur1 - prevEnc1);
  long spd2 = labs(cur2 - prevEnc2);
  prevEnc1 = cur1; prevEnc2 = cur2;
  if (spd1 > MAX_SPEED_COUNTS || spd2 > MAX_SPEED_COUNTS) {
    emergencyStop();
    Serial.println("ERROR:OVERSPEED");
    return true;
  }
  return false;
}

// ============================================================
// Motion Logic
// ============================================================
void beginMove(float tx, float ty) {
  startX = currentX; startY = currentY;
  moveTargetX = tx; moveTargetY = ty;
  float dx = tx - startX; float dy = ty - startY;
  moveTotalDist = sqrt(dx * dx + dy * dy);

  if (moveTotalDist < 0.0002) {
    currentX = tx; currentY = ty;
    isMoving = false;
    Serial.println("D");
    return;
  }

  moveVmax  = penIsDown ? MAX_SPEED_DRAW : MAX_SPEED_TRAVEL;
  moveAccel = MAX_ACCEL;
  tAccel = moveVmax / moveAccel;
  dAccel = 0.5 * moveAccel * tAccel * tAccel;

  if (2.0 * dAccel > moveTotalDist) {
    tAccel = sqrt(moveTotalDist / moveAccel);
    vPeak  = moveAccel * tAccel;
    dAccel = 0.5 * moveAccel * tAccel * tAccel;
    tCruise = 0.0; dCruise = 0.0; tDecel = tAccel;
    tTotal  = tAccel + tDecel;
  } else {
    vPeak   = moveVmax;
    dCruise = moveTotalDist - 2.0 * dAccel;
    tCruise = dCruise / vPeak;
    tDecel  = tAccel;
    tTotal  = tAccel + tCruise + tDecel;
  }
  moveStartTime = micros();
  isMoving = true;
}

void loadNextCommand() {
  while (!bufferEmpty()) {
    MotionCommand cmd = popCmd();
    Serial.print("BUF:"); Serial.println(bufCount);

    if (cmd.type == CMD_PEN) {
      servo3.setAngle(cmd.penAngle);
      penIsDown = (cmd.penAngle == PEN_DOWN_ANGLE);
      delay(250);
      lastControlTime = micros(); 
      prevEnc1 = encoder1.getRawCount();
      prevEnc2 = encoder2.getRawCount();
      Serial.println("P_OK");
    }
    else if (cmd.type == CMD_MOVE) {
      beginMove(cmd.x, cmd.y);
      if (isMoving) return;
    }
  }
}

void updateTrajectory() {
  if (!isMoving) return;
  float t = (float)(micros() - moveStartTime) / 1000000.0;
  if (t >= tTotal) t = tTotal;

  float d = 0.0;
  if (t < tAccel) d = 0.5 * moveAccel * t * t;
  else if (t < (tAccel + tCruise)) d = dAccel + vPeak * (t - tAccel);
  else {
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
    currentX = moveTargetX; currentY = moveTargetY;
    isMoving = false;
    Serial.println("D");
    loadNextCommand();
  }
}

// ============================================================
// Setup & Loop
// ============================================================
void parseSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '!') { emergencyStop(); inputBuffer = ""; continue; }
    if (c == '\r') continue;
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer == "RESUME") { engageMotors(); Serial.println("RESUMED"); }
      else if (inputBuffer == "RELEASE") releaseMotors();
      else if (inputBuffer.startsWith("CAL:")) {
        int comma = inputBuffer.indexOf(',', 4);
        if (comma > 4) {
          float xi = inputBuffer.substring(4, comma).toFloat();
          float yi = inputBuffer.substring(comma + 1).toFloat();
          calibL1 = sqrt(xi * xi + yi * yi);
          calibL2 = sqrt((BASE_WIDTH - xi) * (BASE_WIDTH - xi) + yi * yi);
          
          pid1.setControlMode(CL_OPEN_LOOP); pid2.setControlMode(CL_OPEN_LOOP);
          M1.setDuty(0); M2.setDuty(0);
          encoder1.resetCounter(0); encoder2.resetCounter(0);
          currentX = xi; currentY = yi;
          engageMotors(); isCalibrated = true;
          Serial.print("CALIBRATED:"); Serial.print(xi, 4); Serial.print(","); Serial.println(yi, 4);
        }
      }
      else if (inputBuffer.startsWith("G:")) {
        if (!isCalibrated) Serial.println("ERROR:NOT_READY");
        else if (bufferFull()) Serial.println("ERROR:BUF_FULL");
        else {
          int comma = inputBuffer.indexOf(',', 2);
          if (comma > 2) {
            MotionCommand cmd; cmd.type = CMD_MOVE;
            cmd.x = inputBuffer.substring(2, comma).toFloat();
            cmd.y = inputBuffer.substring(comma + 1).toFloat();
            pushCmd(cmd);
            Serial.print("Q_OK:"); Serial.println(bufCount);
            if (!isMoving) loadNextCommand();
          }
        }
      }
      else if (inputBuffer.startsWith("P:")) {
        if (bufferFull()) Serial.println("ERROR:BUF_FULL");
        else {
          MotionCommand cmd; cmd.type = CMD_PEN;
          cmd.penAngle = inputBuffer.substring(2).toInt();
          pushCmd(cmd);
          Serial.print("Q_OK:"); Serial.println(bufCount);
          if (!isMoving) loadNextCommand();
        }
      }
      inputBuffer = "";
    } else if (inputBuffer.length() < 60) inputBuffer += c;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (!controller.begin()) { Serial.println("ERROR:CARRIER_FAIL"); while (1); }
  M1.setDuty(0); M2.setDuty(0);
  delay(100);
  encoder1.resetCounter(0); encoder2.resetCounter(0);
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
      updateTrajectory();
    }
  }
}