#include <ArduinoMotorCarrier.h>

long target1 = 0, target2 = 0;
bool targetReached = true, stopped = false, released = false;
String inputBuffer = "";
int stableCount = 0;
unsigned long moveStartTime = 0, lastDebugTime = 0;

const int ERROR_THRESHOLD = 200;
const int STABLE_NEEDED = 2;
const unsigned long MOVE_TIMEOUT = 45000;
const long MAX_SPEED = 250;

long prevPos1 = 0, prevPos2 = 0;

void setTargets(long t1, long t2) {
  target1 = t1; target2 = t2;
  pid1.setSetpoint(TARGET_POSITION, t1);
  pid2.setSetpoint(TARGET_POSITION, t2);
}

void stopHere() {
  setTargets(encoder1.getRawCount(), encoder2.getRawCount());
}

void emergencyStop() {
  servo3.setAngle(44);
  M1.setDuty(0);
  M2.setDuty(0);
  stopHere();
  targetReached = true;
  stopped = true;
  stableCount = 0;
  Serial.println("STOPPED");
}

void releaseMotors() {
  pid1.setControlMode(CL_OPEN_LOOP);
  pid2.setControlMode(CL_OPEN_LOOP);
  M1.setDuty(0);
  M2.setDuty(0);
  released = true;
  targetReached = true;
  Serial.println("RELEASED");
}

void engageMotors() {
  pid1.setControlMode(CL_POSITION);
  pid1.setGains(0.18, 0.0, 0.01);
  pid2.setControlMode(CL_POSITION);
  pid2.setGains(0.18, 0.0, 0.01);
  stopHere();
  prevPos1 = encoder1.getRawCount();
  prevPos2 = encoder2.getRawCount();
  released = false;
  stopped = false;
  targetReached = true;
}

void resetEncoders() {
  encoder1.resetCounter(0);
  encoder2.resetCounter(0);
  target1 = 0; target2 = 0;
  prevPos1 = 0; prevPos2 = 0;
}

bool checkOverspeed() {
  long cur1 = encoder1.getRawCount();
  long cur2 = encoder2.getRawCount();
  long speed1 = labs(cur1 - prevPos1);
  long speed2 = labs(cur2 - prevPos2);
  prevPos1 = cur1;
  prevPos2 = cur2;
  if (speed1 > MAX_SPEED || speed2 > MAX_SPEED) {
    emergencyStop();
    Serial.println("ERROR:OVERSPEED");
    return true;
  }
  return false;
}

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

  resetEncoders();

  pid1.setControlMode(CL_POSITION);
  pid1.setGains(0.18, 0.0, 0.01);
  pid2.setControlMode(CL_POSITION);
  pid2.setGains(0.18, 0.0, 0.01);
  setTargets(0, 0);

  servo3.setAngle(44);
  Serial.println("READY");
}

void loop() {
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
      else if (inputBuffer == "CALIBRATE") {
        pid1.setControlMode(CL_OPEN_LOOP);
        pid2.setControlMode(CL_OPEN_LOOP);
        M1.setDuty(0);
        M2.setDuty(0);
        resetEncoders();
        engageMotors();
        Serial.println("CALIBRATED");
      }
      else if (inputBuffer == "POS") {
        Serial.print("POS:");
        Serial.print(encoder1.getRawCount());
        Serial.print(",");
        Serial.println(encoder2.getRawCount());
      }
      else if (inputBuffer.startsWith("G:")) {
        if (stopped || released) {
          Serial.println("ERROR:STOPPED");
        } else {
          int comma = inputBuffer.indexOf(',');
          if (comma > 2 && comma < (int)inputBuffer.length() - 1) {
            setTargets(
              inputBuffer.substring(2, comma).toInt(),
              inputBuffer.substring(comma + 1).toInt()
            );
            targetReached = false;
            stableCount = 0;
            moveStartTime = millis();
          }
        }
      }
      else if (inputBuffer.startsWith("P:")) {
        int angle = inputBuffer.substring(2).toInt();
        if (angle >= 0 && angle <= 180) {
          servo3.setAngle(angle);
          delay(300);
          Serial.println("P_OK");
        }
      }

      inputBuffer = "";
    }
    else if (inputBuffer.length() < 50) {
      inputBuffer += c;
    }
  }

  if (!released && !stopped) {
    if (checkOverspeed()) return;
  }

  if (!targetReached && !stopped && !released) {
    if (millis() - moveStartTime > MOVE_TIMEOUT) {
      long cur1 = encoder1.getRawCount();
      long cur2 = encoder2.getRawCount();
      Serial.print("TIMEOUT_DBG:");
      Serial.print(cur1); Serial.print(","); Serial.print(cur2);
      Serial.print(" T:");
      Serial.print(target1); Serial.print(","); Serial.print(target2);
      Serial.print(" E:");
      Serial.print(labs(target1 - cur1)); Serial.print(","); Serial.println(labs(target2 - cur2));

      stopHere();
      targetReached = true;
      Serial.println("ERROR:TIMEOUT");
      return;
    }

    long cur1 = encoder1.getRawCount();
    long cur2 = encoder2.getRawCount();

    if (millis() - lastDebugTime > 500) {
      Serial.print("DBG:");
      Serial.print(cur1); Serial.print(","); Serial.print(cur2);
      Serial.print(" T:");
      Serial.print(target1); Serial.print(","); Serial.print(target2);
      Serial.print(" E:");
      Serial.print(labs(target1 - cur1)); Serial.print(","); Serial.println(labs(target2 - cur2));
      lastDebugTime = millis();
    }

    if (labs(target1 - cur1) < ERROR_THRESHOLD &&
        labs(target2 - cur2) < ERROR_THRESHOLD) {
      if (++stableCount >= STABLE_NEEDED) {
        stopHere();
        targetReached = true;
        stableCount = 0;
        Serial.println("D");
      }
    } else {
      stableCount = 0;
    }
  }

  delay(10);
}