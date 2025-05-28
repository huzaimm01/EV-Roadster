#include <Servo.h>
#include <Wire.h>

const int ACCEL_PIN = A0;
const int BRAKE_PIN = A1;
const int CLUTCH_PIN = 2;

const int MODE_ENCODER_A = 3;
const int MODE_ENCODER_B = 4;
const int MANUAL_HALL_AUTO = 5;
const int MANUAL_HALL_R = 6;
const int MANUAL_HALL_1 = 7;
const int MANUAL_HALL_2 = 8;
const int MANUAL_HALL_3 = 9;
const int MANUAL_HALL_4 = 10;
const int MANUAL_HALL_5 = 11;

const int PADDLE_UP = 12;
const int PADDLE_DOWN = 13;

const int DRIVE_MODE_UP = 14;
const int DRIVE_MODE_DOWN = 15;

const int AS5600_ADDRESS = 0x36;
const int STEERING_MOTOR_PWM = 16;
const int STEERING_MOTOR_DIR = 17;

const int EPB_SWITCH = 18;

const int DOOR1_SERVO_PIN = 19;
const int DOOR2_SERVO_PIN = 20;
const int TAILGATE_SERVO_PIN = 21;
const int HOOD_SERVO_PIN = 22;

const int DOOR1_LOCK_BTN = 23;
const int DOOR2_LOCK_BTN = 24;
const int TAILGATE_LOCK_BTN = 25;
const int HOOD_LOCK_BTN = 26;

const int START_STOP_BTN = 27;
const int HAZARD_BTN = 28;
const int K1_BTN = 29;

const int VOLUME_ENCODER_A = 30;
const int VOLUME_ENCODER_B = 31;

const int MOTOR_FL_PWM = 32;
const int MOTOR_FL_DIR = 33;
const int MOTOR_FR_PWM = 34;
const int MOTOR_FR_DIR = 35;
const int MOTOR_RL_PWM = 36;
const int MOTOR_RL_DIR = 37;
const int MOTOR_RR_PWM = 38;
const int MOTOR_RR_DIR = 39;

const int HEADLIGHT_LEFT = 40;
const int HEADLIGHT_RIGHT = 41;
const int TAILLIGHT_LEFT = 42;
const int TAILLIGHT_RIGHT = 43;
const int BRAKE_LIGHT = 44;
const int HAZARD_LIGHT = 45;

const int HORN_PIN = 46;
const int WIPER_MOTOR = 47;
const int WIPER_SWITCH = 48;

const int BATTERY_VOLTAGE_PIN = A2;
const int MOTOR_TEMP_PIN = A3;
const int COOLANT_TEMP_PIN = A4;

enum TransmissionMode {
  MODE_P = 0,
  MODE_R = 1,
  MODE_N = 2,
  MODE_D = 3,
  MODE_S = 4,
  MODE_M = 5
};

enum DriveMode {
  DRIVE_AWD = 0,
  DRIVE_FWD = 1,
  DRIVE_RWD = 2
};

enum ManualGear {
  GEAR_AUTO = 0,
  GEAR_R = 1,
  GEAR_1 = 2,
  GEAR_2 = 3,
  GEAR_3 = 4,
  GEAR_4 = 5,
  GEAR_5 = 6
};

TransmissionMode currentMode = MODE_P;
TransmissionMode previousMode = MODE_P;
DriveMode currentDriveMode = DRIVE_AWD;
ManualGear currentManualGear = GEAR_AUTO;
int currentGear = 1;
int targetGear = 1;
bool engineRunning = false;
bool epbActive = true;
bool hazardActive = false;
bool k1Active = false;
bool lightsOn = false;
bool highBeams = false;
bool wipersActive = false;

unsigned long k1StartTime = 0;
int k1PreviousGear = 1;
unsigned long lastGearChange = 0;
unsigned long lastModeChange = 0;
unsigned long hazardBlinkTime = 0;
bool hazardState = false;

volatile int modeEncoderPos = 0;
volatile int volumeEncoderPos = 0;
int lastModeEncoderPos = 0;
int lastVolumeEncoderPos = 0;

Servo doorServo1;
Servo doorServo2;
Servo tailgateServo;
Servo hoodServo;

bool door1Locked = true;
bool door2Locked = true;
bool tailgateLocked = true;
bool hoodLocked = true;

int accelValue = 0;
int brakeValue = 0;
bool clutchPressed = false;
int accelFiltered = 0;
int brakeFiltered = 0;

float steeringAngle = 0;
float targetSteeringAngle = 0;
int steeringPWM = 0;

int Motor_FL = 0;
int Motor_FR = 0;
int Motor_RL = 0;
int Motor_RR = 0;
int SteeringMotor = 0;

bool Motor_FL_Dir = true;
bool Motor_FR_Dir = true;
bool Motor_RL_Dir = true;
bool Motor_RR_Dir = true;

int batteryVoltage = 0;
int motorTemp = 0;
int coolantTemp = 0;

unsigned long lastSensorRead = 0;
unsigned long lastMotorUpdate = 0;
unsigned long lastServoCheck = 0;
unsigned long lastBrakeUpdate = 0;

bool regenBraking = false;
int regenLevel = 0;
bool tcsActive = false;
bool absActive = false;

struct PIDController {
  float kp, ki, kd;
  float lastError;
  float integral;
};

PIDController epbPID = {2.0, 0.1, 0.05, 0, 0};
PIDController steeringPID = {1.5, 0.05, 0.1, 0, 0};

void setup() {
  initializePins();
  initializeServos();
  initializeInterrupts();
  initializeI2C();
  runStartupSequence();
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorRead >= 10) {
    handlePedals();
    readSensors();
    lastSensorRead = currentTime;
  }
  
  if (currentTime - lastMotorUpdate >= 20) {
    handleTransmission();
    handleDriveMode();
    handleMotors();
    lastMotorUpdate = currentTime;
  }
  
  handleSteering();
  handleDashboard();
  
  if (currentTime - lastServoCheck >= 100) {
    handleServos();
    lastServoCheck = currentTime;
  }
  
  if (currentTime - lastBrakeUpdate >= 15) {
    handleBraking();
    handleEPB();
    lastBrakeUpdate = currentTime;
  }
  
  handleLighting();
  handleK1Timer();
  handleSafetySystems();
  handleWipers();
  processEncoders();
}

void initializePins() {
  pinMode(CLUTCH_PIN, INPUT_PULLUP);
  
  pinMode(MODE_ENCODER_A, INPUT_PULLUP);
  pinMode(MODE_ENCODER_B, INPUT_PULLUP);
  
  pinMode(MANUAL_HALL_AUTO, INPUT_PULLUP);
  pinMode(MANUAL_HALL_R, INPUT_PULLUP);
  pinMode(MANUAL_HALL_1, INPUT_PULLUP);
  pinMode(MANUAL_HALL_2, INPUT_PULLUP);
  pinMode(MANUAL_HALL_3, INPUT_PULLUP);
  pinMode(MANUAL_HALL_4, INPUT_PULLUP);
  pinMode(MANUAL_HALL_5, INPUT_PULLUP);
  
  pinMode(PADDLE_UP, INPUT_PULLUP);
  pinMode(PADDLE_DOWN, INPUT_PULLUP);
  
  pinMode(DRIVE_MODE_UP, INPUT_PULLUP);
  pinMode(DRIVE_MODE_DOWN, INPUT_PULLUP);
  
  pinMode(STEERING_MOTOR_PWM, OUTPUT);
  pinMode(STEERING_MOTOR_DIR, OUTPUT);
  
  pinMode(EPB_SWITCH, INPUT_PULLUP);
  
  pinMode(DOOR1_LOCK_BTN, INPUT_PULLUP);
  pinMode(DOOR2_LOCK_BTN, INPUT_PULLUP);
  pinMode(TAILGATE_LOCK_BTN, INPUT_PULLUP);
  pinMode(HOOD_LOCK_BTN, INPUT_PULLUP);
  
  pinMode(START_STOP_BTN, INPUT_PULLUP);
  pinMode(HAZARD_BTN, INPUT_PULLUP);
  pinMode(K1_BTN, INPUT_PULLUP);
  
  pinMode(VOLUME_ENCODER_A, INPUT_PULLUP);
  pinMode(VOLUME_ENCODER_B, INPUT_PULLUP);
  
  pinMode(MOTOR_FL_PWM, OUTPUT);
  pinMode(MOTOR_FL_DIR, OUTPUT);
  pinMode(MOTOR_FR_PWM, OUTPUT);
  pinMode(MOTOR_FR_DIR, OUTPUT);
  pinMode(MOTOR_RL_PWM, OUTPUT);
  pinMode(MOTOR_RL_DIR, OUTPUT);
  pinMode(MOTOR_RR_PWM, OUTPUT);
  pinMode(MOTOR_RR_DIR, OUTPUT);
  
  pinMode(HEADLIGHT_LEFT, OUTPUT);
  pinMode(HEADLIGHT_RIGHT, OUTPUT);
  pinMode(TAILLIGHT_LEFT, OUTPUT);
  pinMode(TAILLIGHT_RIGHT, OUTPUT);
  pinMode(BRAKE_LIGHT, OUTPUT);
  pinMode(HAZARD_LIGHT, OUTPUT);
  
  pinMode(HORN_PIN, OUTPUT);
  pinMode(WIPER_MOTOR, OUTPUT);
  pinMode(WIPER_SWITCH, INPUT_PULLUP);
}

void initializeServos() {
  doorServo1.attach(DOOR1_SERVO_PIN);
  doorServo2.attach(DOOR2_SERVO_PIN);
  tailgateServo.attach(TAILGATE_SERVO_PIN);
  hoodServo.attach(HOOD_SERVO_PIN);
  
  doorServo1.write(0);
  doorServo2.write(0);
  tailgateServo.write(0);
  hoodServo.write(0);
}

void initializeInterrupts() {
  attachInterrupt(digitalPinToInterrupt(MODE_ENCODER_A), modeEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(VOLUME_ENCODER_A), volumeEncoderISR, CHANGE);
}

void initializeI2C() {
  Wire.begin();
  Wire.setClock(400000);
}

void runStartupSequence() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(HEADLIGHT_LEFT, HIGH);
    digitalWrite(HEADLIGHT_RIGHT, HIGH);
    delay(200);
    digitalWrite(HEADLIGHT_LEFT, LOW);
    digitalWrite(HEADLIGHT_RIGHT, LOW);
    delay(200);
  }
}

void handlePedals() {
  int rawAccel = analogRead(ACCEL_PIN);
  int rawBrake = analogRead(BRAKE_PIN);
  
  accelFiltered = (accelFiltered * 9 + rawAccel) / 10;
  brakeFiltered = (brakeFiltered * 9 + rawBrake) / 10;
  
  accelValue = constrain(accelFiltered, 0, 1023);
  brakeValue = constrain(brakeFiltered, 0, 1023);
  
  clutchPressed = !digitalRead(CLUTCH_PIN);
  
  if (brakeValue > 50) {
    regenBraking = true;
    regenLevel = map(brakeValue, 50, 1023, 1, 5);
  } else {
    regenBraking = false;
    regenLevel = 0;
  }
}

void readSensors() {
  batteryVoltage = analogRead(BATTERY_VOLTAGE_PIN);
  motorTemp = analogRead(MOTOR_TEMP_PIN);
  coolantTemp = analogRead(COOLANT_TEMP_PIN);
  
  if (motorTemp > 800) {
    tcsActive = true;
  } else if (motorTemp < 700) {
    tcsActive = false;
  }
  
  if (brakeValue > 500 && (abs(Motor_FL) > 200 || abs(Motor_FR) > 200)) {
    absActive = true;
  } else {
    absActive = false;
  }
}

void processEncoders() {
  if (modeEncoderPos != lastModeEncoderPos) {
    if (millis() - lastModeChange > 200) {
      int diff = modeEncoderPos - lastModeEncoderPos;
      int modeIndex = (int)currentMode;
      
      if (diff > 0) {
        modeIndex++;
        if (modeIndex > MODE_M) modeIndex = MODE_P;
      } else {
        modeIndex--;
        if (modeIndex < MODE_P) modeIndex = MODE_M;
      }
      
      previousMode = currentMode;
      currentMode = (TransmissionMode)modeIndex;
      lastModeChange = millis();
    }
    lastModeEncoderPos = modeEncoderPos;
  }
  
  if (volumeEncoderPos != lastVolumeEncoderPos) {
    lastVolumeEncoderPos = volumeEncoderPos;
  }
}

void handleTransmission() {
  if (currentMode == MODE_M) {
    handleManualShifter();
  } else if (currentMode == MODE_S) {
    handlePaddleShifters();
  } else if (currentMode == MODE_D) {
    handleAutomaticShifting();
  }
  
  if (currentMode == MODE_P) {
    epbActive = true;
    currentGear = 0;
  }
  
  if (currentMode == MODE_N) {
    currentGear = 0;
  }
  
  smoothGearTransition();
}

void handleManualShifter() {
  if (currentMode != MODE_M) return;
  
  static bool clutchWasPressed = false;
  
  if (clutchPressed && !clutchWasPressed) {
    if (!digitalRead(MANUAL_HALL_AUTO)) {
      currentManualGear = GEAR_AUTO;
      currentGear = calculateAutoGear();
    }
    else if (!digitalRead(MANUAL_HALL_R)) {
      currentManualGear = GEAR_R;
      currentGear = -1;
    }
    else if (!digitalRead(MANUAL_HALL_1)) {
      currentManualGear = GEAR_1;
      currentGear = 1;
    }
    else if (!digitalRead(MANUAL_HALL_2)) {
      currentManualGear = GEAR_2;
      currentGear = 2;
    }
    else if (!digitalRead(MANUAL_HALL_3)) {
      currentManualGear = GEAR_3;
      currentGear = 3;
    }
    else if (!digitalRead(MANUAL_HALL_4)) {
      currentManualGear = GEAR_4;
      currentGear = 4;
    }
    else if (!digitalRead(MANUAL_HALL_5)) {
      currentManualGear = GEAR_5;
      currentGear = 5;
    }
    
    lastGearChange = millis();
  }
  
  clutchWasPressed = clutchPressed;
}

void handlePaddleShifters() {
  static bool lastPaddleUp = HIGH;
  static bool lastPaddleDown = HIGH;
  
  bool paddleUp = digitalRead(PADDLE_UP);
  bool paddleDown = digitalRead(PADDLE_DOWN);
  
  if (paddleUp == LOW && lastPaddleUp == HIGH) {
    if (currentGear < 5 && millis() - lastGearChange > 500) {
      currentGear++;
      lastGearChange = millis();
    }
  }
  
  if (paddleDown == LOW && lastPaddleDown == HIGH) {
    if (currentGear > 1 && millis() - lastGearChange > 500) {
      currentGear--;
      lastGearChange = millis();
    }
  }
  
  lastPaddleUp = paddleUp;
  lastPaddleDown = paddleDown;
}

void handleAutomaticShifting() {
  int optimalGear = calculateAutoGear();
  
  if (optimalGear != currentGear && millis() - lastGearChange > 1000) {
    if (optimalGear > currentGear && accelValue > 200) {
      currentGear++;
      lastGearChange = millis();
    } else if (optimalGear < currentGear && accelValue < 100) {
      currentGear--;
      lastGearChange = millis();
    }
  }
}

int calculateAutoGear() {
  int speed = (abs(Motor_FL) + abs(Motor_FR)) / 2;
  
  if (speed < 50) return 1;
  else if (speed < 100) return 2;
  else if (speed < 150) return 3;
  else if (speed < 200) return 4;
  else return 5;
}

void smoothGearTransition() {
  if (targetGear != currentGear) {
    if (millis() - lastGearChange > 200) {
      if (targetGear < currentGear) {
        targetGear++;
      } else if (targetGear > currentGear) {
        targetGear--;
      }
    }
  } else {
    targetGear = currentGear;
  }
}

void handleDriveMode() {
  static bool lastDriveModeUp = HIGH;
  static bool lastDriveModeDown = HIGH;
  static unsigned long lastDriveModeChange = 0;
  
  bool driveModeUp = digitalRead(DRIVE_MODE_UP);
  bool driveModeDown = digitalRead(DRIVE_MODE_DOWN);
  
  if ((driveModeUp == LOW && lastDriveModeUp == HIGH) || 
      (driveModeDown == LOW && lastDriveModeDown == HIGH)) {
    
    if (millis() - lastDriveModeChange > 300) {
      int modeIndex = (int)currentDriveMode;
      modeIndex++;
      if (modeIndex > DRIVE_RWD) modeIndex = DRIVE_AWD;
      currentDriveMode = (DriveMode)modeIndex;
      lastDriveModeChange = millis();
    }
  }
  
  lastDriveModeUp = driveModeUp;
  lastDriveModeDown = driveModeDown;
}

void handleSteering() {
  readSteeringAngle();
  
  float error = targetSteeringAngle - steeringAngle;
  steeringPID.integral += error;
  steeringPID.integral = constrain(steeringPID.integral, -100, 100);
  
  float derivative = error - steeringPID.lastError;
  float output = (steeringPID.kp * error) + 
                 (steeringPID.ki * steeringPID.integral) + 
                 (steeringPID.kd * derivative);
  
  steeringPID.lastError = error;
  
  steeringPWM = constrain(output, -255, 255);
  
  if (abs(steeringPWM) > 10) {
    digitalWrite(STEERING_MOTOR_DIR, steeringPWM > 0);
    analogWrite(STEERING_MOTOR_PWM, abs(steeringPWM));
  } else {
    analogWrite(STEERING_MOTOR_PWM, 0);
  }
  
  SteeringMotor = steeringPWM;
}

void readSteeringAngle() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int rawAngle = (highByte << 8) | lowByte;
    
    float newAngle = (rawAngle * 360.0) / 4096.0;
    if (newAngle > 180) newAngle -= 360;
    
    steeringAngle = (steeringAngle * 0.8) + (newAngle * 0.2);
    steeringAngle = constrain(steeringAngle, -60, 60);
  }
}

void handleDashboard() {
  static bool lastStartStop = HIGH;
  static bool lastHazard = HIGH;
  static bool lastK1 = HIGH;
  static bool lastEPB = HIGH;
  static unsigned long lastButtonPress = 0;
  
  bool startStop = digitalRead(START_STOP_BTN);
  bool hazard = digitalRead(HAZARD_BTN);
  bool k1 = digitalRead(K1_BTN);
  bool epb = digitalRead(EPB_SWITCH);
  
  if (startStop == LOW && lastStartStop == HIGH && millis() - lastButtonPress > 300) {
    if (currentMode == MODE_P || brakeValue > 200) {
      engineRunning = !engineRunning;
      lastButtonPress = millis();
    }
  }
  
  if (hazard == LOW && lastHazard == HIGH && millis() - lastButtonPress > 300) {
    hazardActive = !hazardActive;
    lastButtonPress = millis();
  }
  
  if (k1 == LOW && lastK1 == HIGH && millis() - lastButtonPress > 300) {
    if ((currentMode == MODE_S || currentMode == MODE_M) && engineRunning && !k1Active) {
      k1Active = true;
      k1StartTime = millis();
      k1PreviousGear = currentGear;
      if (currentGear < 5) currentGear++;
      lastButtonPress = millis();
    }
  }
  
  if (epb == LOW && lastEPB == HIGH && millis() - lastButtonPress > 300) {
    if (currentMode == MODE_P || brakeValue > 500) {
      epbActive = !epbActive;
      lastButtonPress = millis();
    }
  }
  
  lastStartStop = startStop;
  lastHazard = hazard;
  lastK1 = k1;
  lastEPB = epb;
}

void handleServos() {
  static bool lastDoor1 = HIGH;
  static bool lastDoor2 = HIGH;
  static bool lastTailgate = HIGH;
  static bool lastHood = HIGH;
  
  bool door1Btn = digitalRead(DOOR1_LOCK_BTN);
  bool door2Btn = digitalRead(DOOR2_LOCK_BTN);
  bool tailgateBtn = digitalRead(TAILGATE_LOCK_BTN);
  bool hoodBtn = digitalRead(HOOD_LOCK_BTN);
  
  if (door1Btn == LOW && lastDoor1 == HIGH) {
    door1Locked = !door1Locked;
    doorServo1.write(door1Locked ? 0 : 90);
  }
  
  if (door2Btn == LOW && lastDoor2 == HIGH) {
    door2Locked = !door2Locked;
    doorServo2.write(door2Locked ? 0 : 90);
  }
  
  if (tailgateBtn == LOW && lastTailgate == HIGH) {
    tailgateLocked = !tailgateLocked;
    tailgateServo.write(tailgateLocked ? 0 : 90);
  }
  
  if (hoodBtn == LOW && lastHood == HIGH) {
    hoodLocked = !hoodLocked;
    hoodServo.write(hoodLocked ? 0 : 90);
  }
  
  lastDoor1 = door1Btn;
  lastDoor2 = door2Btn;
  lastTailgate = tailgateBtn;
  lastHood = hoodBtn;
}

void handleBraking() {
  if (brakeValue > 100) {
    digitalWrite(BRAKE_LIGHT, HIGH);
    
    if (regenBraking && engineRunning) {
      int regenPower = map(brakeValue, 100, 1023, 50, 200);
      Motor_FL = max(0, Motor_FL - regenPower);
      Motor_FR = max(0, Motor_FR - regenPower);
      Motor_RL = max(0, Motor_RL - regenPower);
      Motor_RR = max(0, Motor_RR - regenPower);
    }
  } else {
    digitalWrite(BRAKE_LIGHT, LOW);
  }
}

void handleEPB() {
  if (epbActive) {
    float error = 0 - ((Motor_FL + Motor_FR + Motor_RL + Motor_RR) / 4);
    epbPID.integral += error;
    epbPID.integral = constrain(epbPID.integral, -50, 50);
    
    float derivative = error - epbPID.lastError;
    float output = (epbPID.kp * error) + 
                   (epbPID.ki * epbPID.integral) + 
                   (epbPID.kd * derivative);
    
    epbPID.lastError = error;
    
    int brakingForce = constrain(abs(output), 0, 255);
    
    Motor_FL = constrain(Motor_FL - brakingForce, 0, 255);
    Motor_FR = constrain(Motor_FR - brakingForce, 0, 255);
    Motor_RL = constrain(Motor_RL - brakingForce, 0, 255);
    Motor_RR = constrain(Motor_RR - brakingForce, 0, 255);
  }
}

void handleMotors() {
  int baseMotorSpeed = 0;
  
  if (engineRunning && !epbActive && currentMode != MODE_P && currentMode != MODE_N) {
    if (currentMode == MODE_R || currentGear == -1) {
      baseMotorSpeed = -map(accelValue, 0, 1023, 0, 200);
    } else if (currentGear > 0) {
      int maxSpeed = map(currentGear, 1, 5, 150, 255);
      baseMotorSpeed = map(accelValue, 0, 1023, 0, maxSpeed);
      
      if (k1Active) {
        baseMotorSpeed = min(255, baseMotorSpeed * 1.2);
      }
    }
    
    if (tcsActive) {
      baseMotorSpeed = constrain(baseMotorSpeed, -180, 180);
    }
  }
  
  Motor_FL = baseMotorSpeed;
  Motor_FR = baseMotorSpeed;
  Motor_RL = baseMotorSpeed;
  Motor_RR = baseMotorSpeed;
  
  applyDifferential();
  applyDriveMode();
  outputMotorSignals();
}

void applyDifferential() {
  int steerAdjustment = map(abs(steeringAngle), 0, 60, 0, 50);
  
  if (steeringAngle > 5) {
    Motor_FL = constrain(Motor_FL - steerAdjustment, -255, 255);
    Motor_RL = constrain(Motor_RL - steerAdjustment, -255, 255);
  } else if (steeringAngle < -5) {
    Motor_FR = constrain(Motor_FR - steerAdjustment, -255, 255);
    Motor_RR = constrain(Motor_RR - steerAdjustment, -255, 255);
  }
}

void applyDriveMode() {
  switch (currentDriveMode) {
    case DRIVE_FWD:
      Motor_RL = 0;
      Motor_RR = 0;
      break;
    case DRIVE_RWD:
      Motor_FL = 0;
      Motor_FR = 0;
      break;
    case DRIVE_AWD:
    default:
      break;
  }
}

void outputMotorSignals() {
  Motor_FL_Dir = Motor_FL >= 0;
  Motor_FR_Dir = Motor_FR >= 0;
  Motor_RL_Dir = Motor_RL >= 0;
  Motor_RR_Dir = Motor_RR >= 0;
  
  digitalWrite(MOTOR_FL_DIR, Motor_FL_Dir);
  digitalWrite(MOTOR_FR_DIR, Motor_FR_Dir);
  digitalWrite(MOTOR_RL_DIR, Motor_RL_Dir);
  digitalWrite(MOTOR_RR_DIR, Motor_RR_Dir);
  
  analogWrite(MOTOR_FL_PWM, abs(Motor_FL));
  analogWrite(MOTOR_FR_PWM, abs(Motor_FR));
  analogWrite(MOTOR_RL_PWM, abs(Motor_RL));
  analogWrite(MOTOR_RR_PWM, abs(Motor_RR));
}

void handleLighting() {
  if (engineRunning) {
    digitalWrite(HEADLIGHT_LEFT, HIGH);
    digitalWrite(HEADLIGHT_RIGHT, HIGH);
    digitalWrite(TAILLIGHT_LEFT, HIGH);
    digitalWrite(TAILLIGHT_RIGHT, HIGH);
  } else {
    digitalWrite(HEADLIGHT_LEFT, LOW);
    digitalWrite(HEADLIGHT_RIGHT, LOW);
    digitalWrite(TAILLIGHT_LEFT, LOW);
    digitalWrite(TAILLIGHT_RIGHT, LOW);
  }
  
  if (hazardActive) {
    if (millis() - hazardBlinkTime > 500) {
      hazardState = !hazardState;
      digitalWrite(HAZARD_LIGHT, hazardState);
      hazardBlinkTime = millis();
    }
  } else {
    digitalWrite(HAZARD_LIGHT, LOW);
  }
}

void handleK1Timer() {
  if (k1Active && millis() - k1StartTime >= 10000) {
    currentGear = k1PreviousGear;
    k1Active = false;
  }
}

void handleSafetySystems() {
  if (batteryVoltage < 300) {
    Motor_FL = constrain(Motor_FL, -128, 128);
    Motor_FR = constrain(Motor_FR, -128, 128);
    Motor_RL = constrain(Motor_RL, -128, 128);
    Motor_RR = constrain(Motor_RR, -128, 128);
  }
  
  if (motorTemp > 900) {
    Motor_FL = 0;
    Motor_FR = 0;
    Motor_RL = 0;
    Motor_RR = 0;
    engineRunning = false;
  }
  
  if (coolantTemp > 850) {
    Motor_FL = constrain(Motor_FL, -180, 180);
    Motor_FR = constrain(Motor_FR, -180, 180);
    Motor_RL = constrain(Motor_RL, -180, 180);
    Motor_RR = constrain(Motor_RR, -180, 180);
  }
  
  if (currentMode == MODE_P && (abs(Motor_FL) > 10 || abs(Motor_FR) > 10 || abs(Motor_RL) > 10 || abs(Motor_RR) > 10)) {
    Motor_FL = 0;
    Motor_FR = 0;
    Motor_RL = 0;
    Motor_RR = 0;
  }
}

void handleWipers() {
  static bool lastWiperSwitch = HIGH;
  static bool wipersRunning = false;
  static unsigned long lastWiperCycle = 0;
  
  bool wiperSwitch = digitalRead(WIPER_SWITCH);
  
  if (wiperSwitch == LOW && lastWiperSwitch == HIGH) {
    wipersRunning = !wipersRunning;
  }
  
  if (wipersRunning && millis() - lastWiperCycle > 2000) {
    digitalWrite(WIPER_MOTOR, HIGH);
    delay(500);
    digitalWrite(WIPER_MOTOR, LOW);
    lastWiperCycle = millis();
  }
  
  lastWiperSwitch = wiperSwitch;
}

void emergencyStop() {
  Motor_FL = 0;
  Motor_FR = 0;
  Motor_RL = 0;
  Motor_RR = 0;
  SteeringMotor = 0;
  
  analogWrite(MOTOR_FL_PWM, 0);
  analogWrite(MOTOR_FR_PWM, 0);
  analogWrite(MOTOR_RL_PWM, 0);
  analogWrite(MOTOR_RR_PWM, 0);
  analogWrite(STEERING_MOTOR_PWM, 0);
  
  epbActive = true;
  engineRunning = false;
  
  for (int i = 0; i < 10; i++) {
    digitalWrite(HAZARD_LIGHT, HIGH);
    delay(100);
    digitalWrite(HAZARD_LIGHT, LOW);
    delay(100);
  }
}

void diagnosticCheck() {
  bool systemOK = true;
  
  if (batteryVoltage < 250) systemOK = false;
  if (motorTemp > 950) systemOK = false;
  if (coolantTemp > 900) systemOK = false;
  
  Wire.beginTransmission(AS5600_ADDRESS);
  if (Wire.endTransmission() != 0) systemOK = false;
  
  if (!systemOK) {
    emergencyStop();
  }
}

void updateDashboard() {
  static unsigned long lastDashUpdate = 0;
  
  if (millis() - lastDashUpdate > 100) {
    int speedKmh = map((abs(Motor_FL) + abs(Motor_FR)) / 2, 0, 255, 0, 120);
    int batteryPercent = map(batteryVoltage, 300, 1023, 0, 100);
    int tempCelsius = map(motorTemp, 0, 1023, 20, 120);
    
    lastDashUpdate = millis();
  }
}

void handleReverseLogic() {
  static bool reverseActive = false;
  static unsigned long reverseStartTime = 0;
  
  if (currentMode == MODE_R && !reverseActive) {
    reverseActive = true;
    reverseStartTime = millis();
    
    for (int i = 0; i < 3; i++) {
      digitalWrite(TAILLIGHT_LEFT, HIGH);
      digitalWrite(TAILLIGHT_RIGHT, HIGH);
      delay(200);
      digitalWrite(TAILLIGHT_LEFT, LOW);
      digitalWrite(TAILLIGHT_RIGHT, LOW);
      delay(200);
    }
  } else if (currentMode != MODE_R) {
    reverseActive = false;
  }
  
  if (reverseActive) {
    if (millis() - reverseStartTime > 500) {
      digitalWrite(TAILLIGHT_LEFT, HIGH);
      digitalWrite(TAILLIGHT_RIGHT, HIGH);
    }
  }
}

void handleTorqueVectoring() {
  if (currentDriveMode == DRIVE_AWD && abs(steeringAngle) > 10) {
    int torqueAdjust = map(abs(steeringAngle), 10, 60, 0, 30);
    
    if (steeringAngle > 0) {
      Motor_FL = constrain(Motor_FL + torqueAdjust, -255, 255);
      Motor_RL = constrain(Motor_RL + torqueAdjust, -255, 255);
      Motor_FR = constrain(Motor_FR - torqueAdjust, -255, 255);
      Motor_RR = constrain(Motor_RR - torqueAdjust, -255, 255);
    } else {
      Motor_FR = constrain(Motor_FR + torqueAdjust, -255, 255);
      Motor_RR = constrain(Motor_RR + torqueAdjust, -255, 255);
      Motor_FL = constrain(Motor_FL - torqueAdjust, -255, 255);
      Motor_RL = constrain(Motor_RL - torqueAdjust, -255, 255);
    }
  }
}

void handleCruiseControl() {
  static bool cruiseActive = false;
  static int cruiseSpeed = 0;
  static unsigned long lastCruiseUpdate = 0;
  
  if (currentMode == MODE_D && accelValue < 50 && millis() - lastCruiseUpdate > 5000) {
    cruiseActive = true;
    cruiseSpeed = (abs(Motor_FL) + abs(Motor_FR)) / 2;
  }
  
  if (cruiseActive) {
    if (brakeValue > 100 || accelValue > 200 || currentMode != MODE_D) {
      cruiseActive = false;
    } else {
      int currentSpeed = (abs(Motor_FL) + abs(Motor_FR)) / 2;
      int speedError = cruiseSpeed - currentSpeed;
      
      if (abs(speedError) > 5) {
        Motor_FL = constrain(Motor_FL + speedError, 0, 255);
        Motor_FR = constrain(Motor_FR + speedError, 0, 255);
        Motor_RL = constrain(Motor_RL + speedError, 0, 255);
        Motor_RR = constrain(Motor_RR + speedError, 0, 255);
      }
    }
  }
}

void handlePowerLimiting() {
  int totalPower = abs(Motor_FL) + abs(Motor_FR) + abs(Motor_RL) + abs(Motor_RR);
  int maxPower = 800;
  
  if (batteryVoltage < 400) maxPower = 600;
  if (motorTemp > 800) maxPower = 500;
  if (coolantTemp > 750) maxPower = 400;
  
  if (totalPower > maxPower) {
    float scale = (float)maxPower / totalPower;
    Motor_FL = Motor_FL * scale;
    Motor_FR = Motor_FR * scale;
    Motor_RL = Motor_RL * scale;
    Motor_RR = Motor_RR * scale;
  }
}

void modeEncoderISR() {
  static unsigned long lastInterrupt = 0;
  unsigned long currentTime = micros();
  
  if (currentTime - lastInterrupt > 1000) {
    bool a = digitalRead(MODE_ENCODER_A);
    bool b = digitalRead(MODE_ENCODER_B);
    
    if (a != b) {
      modeEncoderPos++;
    } else {
      modeEncoderPos--;
    }
    
    lastInterrupt = currentTime;
  }
}

void volumeEncoderISR() {
  static unsigned long lastInterrupt = 0;
  unsigned long currentTime = micros();
  
  if (currentTime - lastInterrupt > 1000) {
    bool a = digitalRead(VOLUME_ENCODER_A);
    bool b = digitalRead(VOLUME_ENCODER_B);
    
    if (a != b) {
      volumeEncoderPos++;
    } else {
      volumeEncoderPos--;
    }
    
    lastInterrupt = currentTime;
  }
}