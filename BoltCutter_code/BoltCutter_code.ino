#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>

// ============================================================
// PINS
// ============================================================

#define ESC_PIN 22

#define MOTOR1_IN1 27
#define MOTOR1_IN2 26

#define MOTOR2_IN1 16
#define MOTOR2_IN2 17

// ============================================================
// DRIVE CONFIG
// ============================================================

// Trigger threshold
constexpr int TRIGGER_THRESHOLD = 50;

// Left joystick steering deadzone.
// Bluepad32 joystick axes are roughly centered around 0.
constexpr int STEERING_DEADZONE = 100;

// Set true if left/right steering comes out backwards.
constexpr bool INVERT_STEERING = false;

// Diagnostic print interval
constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 200;

// ============================================================
// ESC CONFIGURATION
// ============================================================

Servo esc;

constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 2000;

constexpr int ESC_START_KICK_US = 1450;
constexpr int ESC_RUN_US = 1550;

constexpr uint32_t ESC_START_KICK_MS = 300;

constexpr int ESC_PWM_HZ = 50;

constexpr int ESC_RAMP_STEP_US = 15;
constexpr uint32_t ESC_RAMP_INTERVAL_MS = 10;

constexpr uint32_t ESC_ARM_TIME_MS = 5000;

constexpr uint32_t CONTROLLER_TIMEOUT_MS = 2000;

// ============================================================
// CONTROLLERS
// ============================================================

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ============================================================
// GLOBAL STATE
// ============================================================

int escCurrentUs = ESC_MIN_US;
int escTargetUs = ESC_MIN_US;

bool brushlessRunning = false;
bool brushlessRequested = false;

bool previousCircleState = false;

bool escStarting = false;
bool controllerTimedOut = false;

uint32_t escStartupTime = 0;
uint32_t lastEscRampUpdate = 0;
uint32_t lastControllerPacket = 0;
uint32_t lastDiagnosticPrint = 0;

// ============================================================
// DRIVE STATE
// ============================================================

enum DriveState {

  DRIVE_STOPPED,

  DRIVE_FORWARD,
  DRIVE_REVERSE,

  DRIVE_FORWARD_LEFT,
  DRIVE_FORWARD_RIGHT,

  DRIVE_REVERSE_LEFT,
  DRIVE_REVERSE_RIGHT,

  DRIVE_PIVOT_LEFT,
  DRIVE_PIVOT_RIGHT
};

DriveState previousDriveState = DRIVE_STOPPED;

// ============================================================
// PRINT MOTOR PINS
// ============================================================

void printMotorPins() {

  Serial.printf(
    "M1: GPIO%d=%d GPIO%d=%d | ",
    MOTOR1_IN1,
    digitalRead(MOTOR1_IN1),
    MOTOR1_IN2,
    digitalRead(MOTOR1_IN2)
  );

  Serial.printf(
    "M2: GPIO%d=%d GPIO%d=%d\n",
    MOTOR2_IN1,
    digitalRead(MOTOR2_IN1),
    MOTOR2_IN2,
    digitalRead(MOTOR2_IN2)
  );
}

// ============================================================
// INDIVIDUAL MOTOR FUNCTIONS
// ============================================================

void motor1Stop() {

  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
}

void motor1Forward() {

  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
}

void motor1Reverse() {

  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
}

void motor2Stop() {

  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
}

void motor2Forward() {

  digitalWrite(MOTOR2_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);
}

void motor2Reverse() {

  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, HIGH);
}

// ============================================================
// PRINT DRIVE STATE
// ============================================================

void printDriveChange(
  DriveState newState,
  const char* text
) {

  if (previousDriveState != newState) {

    previousDriveState = newState;

    Serial.println();
    Serial.print(">>> DRIVE: ");
    Serial.println(text);

    printMotorPins();
  }
}

// ============================================================
// STOP
// ============================================================

void stopRobot() {

  motor1Stop();
  motor2Stop();

  printDriveChange(
    DRIVE_STOPPED,
    "STOP"
  );
}

// ============================================================
// STRAIGHT FORWARD
// ============================================================

void robotForward() {

  motor1Forward();
  motor2Forward();

  printDriveChange(
    DRIVE_FORWARD,
    "FORWARD"
  );
}

// ============================================================
// STRAIGHT REVERSE
// ============================================================

void robotReverse() {

  motor1Reverse();
  motor2Reverse();

  printDriveChange(
    DRIVE_REVERSE,
    "REVERSE"
  );
}

// ============================================================
// FORWARD + LEFT
//
// Stop left/inside wheel.
// Right wheel keeps moving.
// ============================================================

void robotForwardLeft() {

  motor1Stop();
  motor2Forward();

  printDriveChange(
    DRIVE_FORWARD_LEFT,
    "FORWARD LEFT"
  );
}

// ============================================================
// FORWARD + RIGHT
//
// Left wheel keeps moving.
// Stop right/inside wheel.
// ============================================================

void robotForwardRight() {

  motor1Forward();
  motor2Stop();

  printDriveChange(
    DRIVE_FORWARD_RIGHT,
    "FORWARD RIGHT"
  );
}

// ============================================================
// REVERSE + LEFT
// ============================================================

void robotReverseLeft() {

  motor1Stop();
  motor2Reverse();

  printDriveChange(
    DRIVE_REVERSE_LEFT,
    "REVERSE LEFT"
  );
}

// ============================================================
// REVERSE + RIGHT
// ============================================================

void robotReverseRight() {

  motor1Reverse();
  motor2Stop();

  printDriveChange(
    DRIVE_REVERSE_RIGHT,
    "REVERSE RIGHT"
  );
}

// ============================================================
// PIVOT LEFT
//
// Left wheel backwards.
// Right wheel forwards.
// ============================================================

void robotPivotLeft() {

  motor1Reverse();
  motor2Forward();

  printDriveChange(
    DRIVE_PIVOT_LEFT,
    "PIVOT LEFT"
  );
}

// ============================================================
// PIVOT RIGHT
//
// Left wheel forwards.
// Right wheel backwards.
// ============================================================

void robotPivotRight() {

  motor1Forward();
  motor2Reverse();

  printDriveChange(
    DRIVE_PIVOT_RIGHT,
    "PIVOT RIGHT"
  );
}

// ============================================================
// FORCE ESC STOP
// ============================================================

void forceStopESC() {

  brushlessRunning = false;
  brushlessRequested = false;

  escStarting = false;

  escCurrentUs = ESC_MIN_US;
  escTargetUs = ESC_MIN_US;

  esc.writeMicroseconds(ESC_MIN_US);
}

// ============================================================
// ESC UPDATE
// ============================================================

void updateESC() {

  uint32_t now = millis();

  // ----------------------------------------------------------
  // MOTOR OFF
  // ----------------------------------------------------------

  if (!brushlessRequested) {

    escStarting = false;

    if (escCurrentUs != ESC_MIN_US) {

      escCurrentUs = ESC_MIN_US;
      escTargetUs = ESC_MIN_US;

      esc.writeMicroseconds(
        ESC_MIN_US
      );

      Serial.println(
        "ESC -> STOP"
      );
    }

    return;
  }

  // ----------------------------------------------------------
  // STARTUP
  // ----------------------------------------------------------

  if (
    !escStarting &&
    escCurrentUs == ESC_MIN_US
  ) {

    escStarting = true;

    escStartupTime = now;

    escCurrentUs =
      ESC_START_KICK_US;

    esc.writeMicroseconds(
      escCurrentUs
    );

    Serial.println();
    Serial.println(
      "=============================="
    );

    Serial.printf(
      "ESC START KICK: %d us\n",
      ESC_START_KICK_US
    );

    Serial.println(
      "=============================="
    );

    return;
  }

  // ----------------------------------------------------------
  // HOLD STARTUP KICK
  // ----------------------------------------------------------

  if (escStarting) {

    esc.writeMicroseconds(
      ESC_START_KICK_US
    );

    if (
      now - escStartupTime >=
      ESC_START_KICK_MS
    ) {

      escStarting = false;

      escCurrentUs =
        ESC_START_KICK_US;

      lastEscRampUpdate = now;

      Serial.println(
        "Startup kick finished"
      );
    }

    return;
  }

  // ----------------------------------------------------------
  // RAMP
  // ----------------------------------------------------------

  escTargetUs =
    ESC_RUN_US;

  if (
    now - lastEscRampUpdate <
    ESC_RAMP_INTERVAL_MS
  ) {

    return;
  }

  lastEscRampUpdate = now;

  if (
    escCurrentUs <
    escTargetUs
  ) {

    escCurrentUs +=
      ESC_RAMP_STEP_US;

    if (
      escCurrentUs >
      escTargetUs
    ) {

      escCurrentUs =
        escTargetUs;
    }
  }

  else if (
    escCurrentUs >
    escTargetUs
  ) {

    escCurrentUs -=
      ESC_RAMP_STEP_US;

    if (
      escCurrentUs <
      escTargetUs
    ) {

      escCurrentUs =
        escTargetUs;
    }
  }

  esc.writeMicroseconds(
    escCurrentUs
  );
}

// ============================================================
// CONTROLLER CONNECTED
// ============================================================

void onConnectedController(
  ControllerPtr ctl
) {

  for (
    int i = 0;
    i < BP32_MAX_GAMEPADS;
    i++
  ) {

    if (
      myControllers[i] ==
      nullptr
    ) {

      myControllers[i] = ctl;

      lastControllerPacket =
        millis();

      previousCircleState =
        false;

      controllerTimedOut =
        false;

      Serial.println();

      Serial.printf(
        "Controller connected at index %d\n",
        i
      );

      return;
    }
  }

  Serial.println(
    "No controller slot available"
  );
}

// ============================================================
// CONTROLLER DISCONNECTED
// ============================================================

void onDisconnectedController(
  ControllerPtr ctl
) {

  for (
    int i = 0;
    i < BP32_MAX_GAMEPADS;
    i++
  ) {

    if (
      myControllers[i] ==
      ctl
    ) {

      myControllers[i] =
        nullptr;

      Serial.printf(
        "Controller disconnected from index %d\n",
        i
      );

      stopRobot();
      forceStopESC();

      return;
    }
  }
}

// ============================================================
// PROCESS GAMEPAD
// ============================================================

void processGamepad(
  ControllerPtr ctl
) {

  // ----------------------------------------------------------
  // READ CONTROLLER
  // ----------------------------------------------------------

  int r2 =
    ctl->throttle();

  int l2 =
    ctl->brake();

  // LEFT JOYSTICK HORIZONTAL AXIS
  int steering =
    ctl->axisX();

  // Easy way to reverse steering if required
  if (INVERT_STEERING) {

    steering =
      -steering;
  }

  bool steeringLeft =
    steering <
    -STEERING_DEADZONE;

  bool steeringRight =
    steering >
    STEERING_DEADZONE;

  bool forwardRequested =
    r2 > TRIGGER_THRESHOLD &&
    r2 > l2;

  bool reverseRequested =
    l2 > TRIGGER_THRESHOLD &&
    l2 > r2;

  // ----------------------------------------------------------
  // SERIAL DIAGNOSTICS
  // ----------------------------------------------------------

  uint32_t now =
    millis();

  if (
    now - lastDiagnosticPrint >=
    DIAGNOSTIC_INTERVAL_MS
  ) {

    lastDiagnosticPrint = now;

    Serial.println();
    Serial.println(
      "---------- CONTROLLER DIAGNOSTIC ----------"
    );

    Serial.printf(
      "R2 throttle  = %d\n",
      r2
    );

    Serial.printf(
      "L2 brake     = %d\n",
      l2
    );

    Serial.printf(
      "Left stick X = %d\n",
      steering
    );

    Serial.printf(
      "Circle/B     = %d\n",
      ctl->b()
    );

    Serial.printf(
      "ESC us       = %d\n",
      escCurrentUs
    );

    Serial.printf(
      "Weapon ON    = %d\n",
      brushlessRunning
    );

    printMotorPins();

    Serial.println(
      "-------------------------------------------"
    );
  }

  // ==========================================================
  // FORWARD
  // ==========================================================

  if (forwardRequested) {

    if (steeringRight) {

      robotForwardRight();
    }

    else if (steeringLeft) {

      robotForwardLeft();
    }

    else {

      robotForward();
    }
  }

  // ==========================================================
  // REVERSE
  // ==========================================================

  else if (reverseRequested) {

    if (steeringRight) {

      robotReverseRight();
    }

    else if (steeringLeft) {

      robotReverseLeft();
    }

    else {

      robotReverse();
    }
  }

  // ==========================================================
  // NO TRIGGER = STOP
  //
  // The left stick only selects steering direction.
  // It must never move the drive motors by itself.
  // ==========================================================

  else {

    stopRobot();
  }

  // ==========================================================
  // CIRCLE TOGGLES BRUSHLESS
  // ==========================================================

  bool circleNow =
    ctl->b();

  if (
    circleNow &&
    !previousCircleState
  ) {

    brushlessRunning =
      !brushlessRunning;

    if (brushlessRunning) {

      Serial.println();
      Serial.println(
        "=============================="
      );

      Serial.println(
        "BRUSHLESS MOTOR ON"
      );

      Serial.println(
        "=============================="
      );
    }

    else {

      Serial.println();
      Serial.println(
        "=============================="
      );

      Serial.println(
        "BRUSHLESS MOTOR OFF"
      );

      Serial.println(
        "=============================="
      );
    }
  }

  previousCircleState =
    circleNow;

  brushlessRequested =
    brushlessRunning;
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "ESP32 STARTING"
  );

  Serial.println(
    "Bluepad32 + LEFT STICK STEERING"
  );

  Serial.println(
    "========================================"
  );

  // ----------------------------------------------------------
  // DRV8833
  // ----------------------------------------------------------

  pinMode(
    MOTOR1_IN1,
    OUTPUT
  );

  pinMode(
    MOTOR1_IN2,
    OUTPUT
  );

  pinMode(
    MOTOR2_IN1,
    OUTPUT
  );

  pinMode(
    MOTOR2_IN2,
    OUTPUT
  );

  stopRobot();

  Serial.println();
  Serial.println(
    "DRV8833 GPIO STARTUP TEST"
  );

  printMotorPins();

  // ----------------------------------------------------------
  // ESC
  // ----------------------------------------------------------

  Serial.println();
  Serial.println(
    "Initializing ESC..."
  );

  esc.setPeriodHertz(
    ESC_PWM_HZ
  );

  esc.attach(
    ESC_PIN,
    ESC_MIN_US,
    ESC_MAX_US
  );

  escCurrentUs =
    ESC_MIN_US;

  escTargetUs =
    ESC_MIN_US;

  esc.writeMicroseconds(
    ESC_MIN_US
  );

  Serial.printf(
    "ESC minimum: %d us\n",
    ESC_MIN_US
  );

  Serial.printf(
    "ESC start kick: %d us\n",
    ESC_START_KICK_US
  );

  Serial.printf(
    "ESC running: %d us\n",
    ESC_RUN_US
  );

  Serial.println(
    "Waiting for ESC to arm..."
  );

  delay(
    ESC_ARM_TIME_MS
  );

  Serial.println(
    "ESC armed."
  );

  // ----------------------------------------------------------
  // BLUEPAD32
  // ----------------------------------------------------------

  BP32.setup(
    &onConnectedController,
    &onDisconnectedController
  );

  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "Bluetooth ready"
  );

  Serial.println(
    "R2 = Forward"
  );

  Serial.println(
    "L2 = Reverse"
  );

  Serial.println(
    "Left stick = Steering"
  );

  Serial.println(
    "Left stick alone = NO MOVEMENT"
  );

  Serial.println(
    "Circle = Weapon ON/OFF"
  );

  Serial.println(
    "========================================"
  );
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  BP32.update();

  ControllerPtr activeController =
    nullptr;

  // ----------------------------------------------------------
  // FIND CONNECTED CONTROLLER
  // ----------------------------------------------------------

  for (
    int i = 0;
    i < BP32_MAX_GAMEPADS;
    i++
  ) {

    ControllerPtr ctl =
      myControllers[i];

    if (
      ctl &&
      ctl->isConnected()
    ) {

      activeController =
        ctl;

      if (
        ctl->hasData()
      ) {

        lastControllerPacket =
          millis();

        controllerTimedOut =
          false;

        processGamepad(
          ctl
        );
      }

      break;
    }
  }

  // ----------------------------------------------------------
  // NO CONTROLLER
  // ----------------------------------------------------------

  if (
    activeController ==
    nullptr
  ) {

    stopRobot();

    if (
      brushlessRunning
    ) {

      Serial.println(
        "Controller disconnected -> ESC STOP"
      );
    }

    forceStopESC();
  }

  // ----------------------------------------------------------
  // CONTROLLER TIMEOUT
  // ----------------------------------------------------------

  else {

    uint32_t timeSincePacket =
      millis() -
      lastControllerPacket;

    if (
      timeSincePacket >
      CONTROLLER_TIMEOUT_MS
    ) {

      if (
        !controllerTimedOut
      ) {

        controllerTimedOut =
          true;

        Serial.println();
        Serial.println(
          "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        );

        Serial.println(
          "CONTROLLER TIMEOUT"
        );

        Serial.printf(
          "No controller data for %lu ms\n",
          timeSincePacket
        );

        Serial.println(
          "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        );
      }

      stopRobot();
      forceStopESC();
    }
  }

  // ----------------------------------------------------------
  // UPDATE ESC
  // ----------------------------------------------------------

  updateESC();

  delay(2);
}