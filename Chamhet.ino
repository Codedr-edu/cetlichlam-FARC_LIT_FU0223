#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PS2 pin
#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14

// PS2 Controller
PS2X ps2x;

// Động cơ di chuyển
#define LEFT_FORWARD 8
#define LEFT_BACKWARD 9
#define RIGHT_FORWARD 10
#define RIGHT_BACKWARD 11

// Động cơ nâng/thả (motor 180 RPM)
#define LIFT_UP_1 12
#define LIFT_DOWN_1 13
#define LIFT_UP_2 14
#define LIFT_DOWN_2 15

// Servo cánh chắn vật
#define SERVO_GATE 2  // kênh PWM số 2
#define SERVO_GATE1 3  
#define SERVO_GATE2 6
#define SERVO_GATE3 7  

// Trạng thái cửa
bool isGateOpen = false;
bool isGateOpen1 = false;
bool isGateOpen2 = false;

// Chuyển góc servo thành giá trị PWM (0–4095)
int angleToPulse(int angle) {
  return map(angle, 0, 180, 120, 500); // chỉnh tùy theo loại servo
}

void setup() {
  Serial.begin(115200);

  // Kết nối PS2
  int error = -1;
  for (int i = 0; i < 10; i++) {
    delay(100);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
    if (!error) break;
    Serial.print(".");
  }

  if (error) {
    Serial.println("❌ Không kết nối được tay cầm PS2!");
    while (true);
  }

  Serial.println("\n✅ Tay cầm PS2 đã kết nối!");

  // Khởi tạo PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);

  pwm.writeMicroseconds(SERVO_GATE3, 2000);
  pwm.writeMicroseconds(SERVO_GATE4, 1000);

  isGateOpen = false;
  isGateOpen1 = false;
  //isGateOpen2 = false;

  // Dừng tất cả động cơ
  stopAll();
}

void stopAll() {
  pwm.setPWM(LEFT_FORWARD, 0, 0);
  pwm.setPWM(LEFT_BACKWARD, 0, 0);
  pwm.setPWM(RIGHT_FORWARD, 0, 0);
  pwm.setPWM(RIGHT_BACKWARD, 0, 0);
  pwm.setPWM(LIFT_UP_1, 0, 0);
  pwm.setPWM(LIFT_DOWN_1, 0, 0);
  pwm.setPWM(LIFT_UP_2, 0, 0);
  pwm.setPWM(LIFT_DOWN_2, 0, 0);
}

void loop() {
  ps2x.read_gamepad();

  int deadzone = 10;

  // Đọc joystick trái (PSS_LX: ngang, PSS_LY: dọc)
  int rawX = ps2x.Analog(PSS_RX);
  int rawY = ps2x.Analog(PSS_LY);

  int x = 128 - rawX;
  int y = rawY - 128;

  if (abs(x) < deadzone) x = 0;
  if (abs(y) < deadzone) y = 0;

  // Tính tốc độ từng bánh
  int leftSpeed = y + x;
  int rightSpeed = y - x;

  leftSpeed = constrain(leftSpeed, -128, 128);
  rightSpeed = constrain(rightSpeed, -128, 128);

  int pwmLeft = map(abs(leftSpeed), 0, 128, 0, 2730);
  int pwmRight = map(abs(rightSpeed), 0, 128, 0, 2730);

  // Điều khiển bánh trái
  if (leftSpeed > 10) {
    pwm.setPWM(LEFT_FORWARD, 0, pwmLeft);
    pwm.setPWM(LEFT_BACKWARD, 0, 0);
  } else if (leftSpeed < -10) {
    pwm.setPWM(LEFT_FORWARD, 0, 0);
    pwm.setPWM(LEFT_BACKWARD, 0, pwmLeft);
  } else {
    pwm.setPWM(LEFT_FORWARD, 0, 0);
    pwm.setPWM(LEFT_BACKWARD, 0, 0);
  }

  // Điều khiển bánh phải
  if (rightSpeed > 10) {
    pwm.setPWM(RIGHT_FORWARD, 0, pwmRight);
    pwm.setPWM(RIGHT_BACKWARD, 0, 0);
  } else if (rightSpeed < -10) {
    pwm.setPWM(RIGHT_FORWARD, 0, 0);
    pwm.setPWM(RIGHT_BACKWARD, 0, pwmRight);
  } else {
    pwm.setPWM(RIGHT_FORWARD, 0, 0);
    pwm.setPWM(RIGHT_BACKWARD, 0, 0);
  }

  // Điều khiển cơ cấu nâng/thả
  if (ps2x.Button(PSB_R2)) {
    pwm.setPWM(LIFT_UP_1, 0, 2730);
    pwm.setPWM(LIFT_DOWN_1, 0, 0);
    pwm.setPWM(LIFT_UP_2, 0, 0);
    pwm.setPWM(LIFT_DOWN_2, 0, 2730);
  } else if (ps2x.Button(PSB_R1)) {
    pwm.setPWM(LIFT_UP_1, 0, 0);
    pwm.setPWM(LIFT_DOWN_1, 0, 2730);
    pwm.setPWM(LIFT_UP_2, 0, 2730);
    pwm.setPWM(LIFT_DOWN_2, 0, 0);
  } else {
    pwm.setPWM(LIFT_UP_1, 0, 0);
    pwm.setPWM(LIFT_DOWN_1, 0, 0);
    pwm.setPWM(LIFT_UP_2, 0, 0);
    pwm.setPWM(LIFT_DOWN_2, 0, 0);
  }

  // === Servo cánh chắn vật ===
  """
  if (ps2x.ButtonPressed(PSB_GREEN) && !isGateOpen1) {
    pwm.writeMicroseconds(SERVO_GATE1, 2000);  // Mở cửa
    isGateOpen1 = true;
  } else if (ps2x.ButtonPressed(PSB_CROSS) && isGateOpen1) {
    pwm.writeMicroseconds(SERVO_GATE1, 1000);  // Đóng cửa
    isGateOpen1 = false;
  }

  if (ps2x.ButtonPressed(PSB_PINK) && !isGateOpen2) {
    pwm.writeMicroseconds(SERVO_GATE2, 2000);  // Mở cửa
    isGateOpen2 = true;
  } else if (ps2x.ButtonPressed(PSB_RED) && isGateOpen2) {
    pwm.writeMicroseconds(SERVO_GATE2, 1000);  // Đóng cửa
    isGateOpen2 = false;
  }
  """

  if (ps2x.ButtonPressed(PSB_L2)) {
    if (!isGateOpen2){
      pwm.writeMicroseconds(SERVO_GATE, 1000);
      pwm.writeMicroseconds(SERVO_GATE2, 2000);
      delay(500);
      pwm.writeMicroseconds(SERVO_GATE, 1500);
      pwm.writeMicroseconds(SERVO_GATE2, 1500);
      isGateOpen = true;
    } else {
      pwm.writeMicroseconds(SERVO_GATE, 2000);
      pwm.writeMicroseconds(SERVO_GATE2, 1000);
      delay(500);
      pwm.writeMicroseconds(SERVO_GATE, 1500);
      pwm.writeMicroseconds(SERVO_GATE2, 1500);
      isGateOpen = false;
    }  // Mở cửa
  } else {
    pwm.writeMicroseconds(SERVO_GATE, 1500);
    pwm.writeMicroseconds(SERVO_GATE2, 1500);
  }
  
  if (ps2x.ButtonPressed(PSB_L1)) {
    if (!isGateOpen1){
      pwm.writeMicroseconds(SERVO_GATE, 1000); 
      pwm.writeMicroseconds(SERVO_GATE1, 2000); 
      isGateOpen1 = true;
    } else {
      pwm.writeMicroseconds(SERVO_GATE, 2000); 
      pwm.writeMicroseconds(SERVO_GATE1, 1000); 
      isGateOpen1 = false;
    }
  }

  // Debug
  Serial.print("LY: "); Serial.print(y);
  Serial.print(" | LX: "); Serial.print(x);
  Serial.print(" | L_PWM: "); Serial.print(pwmLeft);
  Serial.print(" | R_PWM: "); Serial.print(pwmRight);
  Serial.print(" | GATE: "); Serial.println(isGateOpen ? "OPEN" : "CLOSED");

  delay(30);
}
