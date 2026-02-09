#include <IRremote.h>
// Maze Robot – Left‑Wall Follower
// Arduino UNO R3 + 3×HC‑SR04 + L298 + 2 DC motors
// Speed ≈5 cm/s (PWM ~150 ▸ calibrate sau)
// Loop period 100 ms ▸ quét 10 Hz

// === PINOUT ===

// IR receiver
const int IR_PIN     = A1;   // VS1838B data output

// HC‑SR04 front
const int TRIG_F = 2;
const int ECHO_F = 3;
// HC‑SR04 left
const int TRIG_L = 4;
const int ECHO_L = 5;
// HC‑SR04 right
const int TRIG_R = 6;
const int ECHO_R = 7;

// L298 Motor A (left)
const int IN1 = 8;
const int IN2 = 12;
const int ENA = 9;  // PWM
// L298 Motor B (right)
const int IN3 = 11;
const int IN4 = 13;
const int ENB = 10; // PWM

// CONFIG
const unsigned long LOOP_PERIOD = 100; // ms
const float NguongPhai   = 12;   // cm, ngưỡng bám tường
const float NguongTruoc  = 10;

// TURN DURATION để rẽ 90°
// ▸ Bạn có thể hiệu chỉnh experimentally
const unsigned long TURN_TIME = 400; // ms  

// IR remote start code (NEC protocol).
// Thay giá trị dưới đây bằng mã nút START trên remote của bạn.
const unsigned long START_CODE = 3125149440U;
bool started = false;
const unsigned long STOP_CODE = 3108437760U; // Thay bằng mã nút STOP trên remote của bạn

// STATE MACHINE
enum State {
  FOLLOW_WALL,
  TURN_LEFT,
  TURN_RIGHT,
  STOP_ALL
};
State state = FOLLOW_WALL;

// Timing
unsigned long lastLoop = 0;


// -------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  // Serial debug
  Serial.begin(115200);
  // Initialize IR receiver
  IrReceiver.begin(IR_PIN, false);
  Serial.println("IR Scanner active: press a button to see codes...");
  // HC‑SR04 pins
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  // L298 pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);  pinMode(ENB, OUTPUT);
  Serial.println("Waiting for IR START...");

}

// -------------------------------------------------------------------
void loop() {
  if (!started) {
  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    if (code != 0) {
      Serial.print("IR code: "); Serial.println(code);
    }
    if (code == START_CODE) {
      started = true;
      Serial.println(">> Robot Started <<");
    }
    IrReceiver.resume();
  }
  return;
} else {
  // Đang chạy => kiểm tra nếu bấm nút STOP
  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    if (code == STOP_CODE) {
      started = false;
      stopMotors();
      Serial.println(">> Robot Stopped <<");
    }
    IrReceiver.resume();
  }
}


  // Chạy state machine khi đã start
  unsigned long now = millis();
  if (now - lastLoop < LOOP_PERIOD) return;
  lastLoop = now;

  // 1) Đọc khoảng cách (cm)
  float dF = getDistance(TRIG_F, ECHO_F);
  float dL = getDistance(TRIG_L, ECHO_L);
  float dR = getDistance(TRIG_R, ECHO_R);

  // 2) Quyết định state mới
  State nextState = state;
  
  if ( dR <=NguongPhai &&  dF <= NguongTruoc) {
    nextState = TURN_LEFT;
  }
  else if (dR <= NguongPhai && dF > NguongTruoc) {
    nextState = FOLLOW_WALL;
  }
  else if (dR > NguongPhai && dF <= NguongTruoc ) {
    nextState = TURN_RIGHT;
  }
  else {
    nextState = TURN_RIGHT;
  }

  
  // 3) Thực thi hành động theo state
  switch (nextState) {
    case FOLLOW_WALL:
      driveForward();
      break;
    case TURN_LEFT:
      turnLeft();
      break;
    case TURN_RIGHT:
      turnRight();
      break;
    case STOP_ALL:
    default:
      stopMotors();
      break;
  }
  state = nextState;

  // 4) Debug qua Serial
  Serial.print("F:"); Serial.print(dF,1);
  Serial.print(" L:"); Serial.print(dL,1);
  Serial.print(" R:"); Serial.print(dR,1);
  Serial.print(" | State:"); Serial.print(state);
  // Serial.print(" | PWM:"); Serial.print(MOTOR_SPEED);
  // Serial.print(" | Speed:"); Serial.print(BASE_SPEED);
  Serial.println();
}

// -------------------------------------------------------------------
// Đọc khoảng cách HC‑SR04 (trừ offset)
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); 
  // max timeout 30 ms ➔ ~5 m
  float dist = duration * 0.0343 / 2.0; // cm
  if (dist < 0) dist = 0;
  return dist;
}

// -------------------------------------------------------------------
// Di chuyển thẳng
void driveForward() {
  // Motor A forward
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(ENA, 120);
  // Motor B forward
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  analogWrite(ENB, 120); 
  delay(100);
}

// Rẽ trái in‑place 90°
void turnLeft() {
  // Motor A backward
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(ENA, 110);
  // Motor B forward
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  analogWrite(ENB, 110);
  delay(TURN_TIME);
  stopMotors();
  delay(150);  // delay nhẹ để robot ổn định trước khi đọc lại cảm biến
}

// Rẽ phải in‑place 90°
void turnRight() {
  // Motor A forward
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(ENA, 110);
  // Motor B backward
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(ENB, 110);
  delay(TURN_TIME);
  stopMotors();
  delay(150);
}

// Dừng tất cả động cơ
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
