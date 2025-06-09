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

// Động cơ trái: 8 (tiến), 9 (lùi)
// Động cơ phải: 10 (tiến), 11 (lùi)
#define LEFT_FORWARD 8
#define LEFT_BACKWARD 9
#define RIGHT_FORWARD 10
#define RIGHT_BACKWARD 11

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
    while (true); // Dừng tại đây
  }

  Serial.println("\n✅ Tay cầm PS2 đã kết nối!");

  // Khởi tạo PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);

  // Dừng tất cả động cơ
  stopAll();
}

void stopAll() {
  pwm.setPWM(LEFT_FORWARD, 0, 0);
  pwm.setPWM(LEFT_BACKWARD, 0, 0);
  pwm.setPWM(RIGHT_FORWARD, 0, 0);
  pwm.setPWM(RIGHT_BACKWARD, 0, 0);
}

void loop() {
  ps2x.read_gamepad();

  int deadzone = 10;

  // Đọc joystick trái (PSS_LX: ngang, PSS_LY: dọc)
  int rawX = ps2x.Analog(PSS_LX);
  int rawY = ps2x.Analog(PSS_LY);

  int x = rawX - 128;
  int y = 128 - rawY;

  if (abs(x) < deadzone) x = 0;
  if (abs(y) < deadzone) y = 0;

  // Tính tốc độ từng bánh
  int leftSpeed = y + x;
  int rightSpeed = y - x;

  // Giới hạn từ -128 đến 128
  leftSpeed = constrain(leftSpeed, -128, 128);
  rightSpeed = constrain(rightSpeed, -128, 128);

  // Đổi sang PWM (0 - 2730)
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

  // Debug giá trị
  Serial.print("LY: "); Serial.print(y);
  Serial.print(" | LX: "); Serial.print(x);
  Serial.print(" | L_PWM: "); Serial.print(pwmLeft);
  Serial.print(" | R_PWM: "); Serial.println(pwmRight);

  delay(30);
}