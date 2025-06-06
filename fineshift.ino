#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// PCA9685 - Driver board cho PWM
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Chân kết nối PS2 Controller
#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14

// Đối tượng PS2 Controller
PS2X ps2x;

// Định nghĩa chân PWM cho 2 động cơ chính (Motor trái và Motor phải)
// Sử dụng các chân 8, 9, 10, 11 trên PCA9685
#define LEFT_FORWARD 8    // Motor trái tiến
#define LEFT_BACKWARD 9   // Motor trái lùi
#define RIGHT_FORWARD 10  // Motor phải tiến
#define RIGHT_BACKWARD 11 // Motor phải lùi

// Định nghĩa chân và thông số cho động cơ thứ ba (DC Motor)
// Sử dụng các chân 12, 13 trên PCA9685
#define MOTOR3_FORWARD 12    // Motor 3 tiến
#define MOTOR3_BACKWARD 13   // Motor 3 lùi
#define MOTOR3_SPEED 2000    // Tốc độ cố định cho motor 3 (giá trị PWM, max là 2730)
#define MOTOR3_DURATION 2000 // Thời gian motor 3 sẽ quay khi nhấn R2 (miligiay)

// Biến trạng thái và thời gian để điều khiển motor 3 không chặn (non-blocking)
bool motor3_active = false;
unsigned long motor3_startTime = 0;

// Định nghĩa chân và thông số cho Servo
// Sử dụng chân 14 trên PCA9685
#define SERVO_PIN 14       // Chân PWM của servo
#define SERVO_POS_ACTIVE 450 // Giá trị PWM cho vị trí kích hoạt của servo (góc mở, nâng...)
#define SERVO_POS_REST 150   // Giá trị PWM cho vị trí nghỉ của servo (góc đóng, hạ...)
#define SERVO_DURATION 1500  // Thời gian servo giữ ở vị trí kích hoạt (miligiay)

// Biến trạng thái và thời gian để điều khiển servo không chặn (non-blocking)
bool servo_active = false;
unsigned long servo_startTime = 0;

void setup() {
  Serial.begin(115200); // Khởi tạo Serial Monitor để debug

  // --- Cấu hình và kết nối Tay cầm PS2 ---
  int error = -1;
  Serial.print("Dang ket noi tay cam PS2");
  for (int i = 0; i < 10; i++) { // Thử kết nối nhiều lần
    delay(100);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
    if (!error) break; // Nếu kết nối thành công, thoát vòng lặp
    Serial.print(".");
  }

  if (error) { // Nếu không kết nối được sau nhiều lần thử
    Serial.println("\n❌ Khong ket noi duoc tay cam PS2! Kiem tra day noi.");
    while (true); // Dừng chương trình tại đây nếu không có tay cầm
  }
  Serial.println("\n✅ Tay cam PS2 da ket noi!");

  // --- Khởi tạo PCA9685 ---
  pwm.begin(); // Bắt đầu giao tiếp với PCA9685
  pwm.setOscillatorFrequency(27000000); // Đặt tần số dao động (thường là 25MHz hoặc 27MHz, 27MHz ổn định hơn)
  pwm.setPWMFreq(50); // Đặt tần số PWM cho tất cả các kênh là 50Hz (tốt cho cả động cơ DC và servo)
  Wire.setClock(400000); // Đặt tốc độ truyền I2C lên 400kHz (tăng hiệu suất)

  // --- Dừng tất cả động cơ và đặt servo về vị trí ban đầu ---
  stopAllMotors(); // Dừng 2 động cơ chính và động cơ thứ 3
  pwm.setPWM(SERVO_PIN, 0, SERVO_POS_REST); // Đặt servo về vị trí nghỉ ban đầu
}

// Hàm dừng tất cả các động cơ DC (không ảnh hưởng đến servo)
void stopAllMotors() {
  pwm.setPWM(LEFT_FORWARD, 0, 0);
  pwm.setPWM(LEFT_BACKWARD, 0, 0);
  pwm.setPWM(RIGHT_FORWARD, 0, 0);
  pwm.setPWM(RIGHT_BACKWARD, 0, 0);
  pwm.setPWM(MOTOR3_FORWARD, 0, 0);
  pwm.setPWM(MOTOR3_BACKWARD, 0, 0);
}

void loop() {
  ps2x.read_gamepad(); // Đọc trạng thái từ tay cầm PS2

  int deadzone = 10; // Vùng chết cho joystick để tránh trôi nhẹ

  // --- Điều khiển 2 động cơ chính bằng Joystick phải (PSS_RX) và Joystick trái (PSS_LY) ---
  int rawX = ps2x.Analog(PSS_RX); // Giá trị ngang của joystick phải (dùng để xoay)
  int rawY = ps2x.Analog(PSS_LY); // Giá trị dọc của joystick trái (dùng để tiến/lùi)

  // Chuyển đổi giá trị analog (0-255) về khoảng -128 đến 127
  int x = rawX - 128; // Giá trị ngang
  int y = 128 - rawY; // Giá trị dọc (đảo ngược để joystick lên là tiến)

  // Áp dụng deadzone
  if (abs(x) < deadzone) x = 0;
  if (abs(y) < deadzone) y = 0;

  // Tính toán tốc độ riêng cho từng bánh (phương pháp mix joystick)
  int leftSpeed = y + x;
  int rightSpeed = y - x;

  // Giới hạn tốc độ trong khoảng -128 đến 128
  leftSpeed = constrain(leftSpeed, -128, 128);
  rightSpeed = constrain(rightSpeed, -128, 128);

  // Chuyển đổi tốc độ từ -128/128 sang giá trị PWM (0-2730)
  // Giá trị 2730 tương ứng khoảng 66% duty cycle cho động cơ DC, bạn có thể điều chỉnh max là 4095
  int pwmLeft = map(abs(leftSpeed), 0, 128, 0, 2730);
  int pwmRight = map(abs(rightSpeed), 0, 128, 0, 2730);

  // Điều khiển động cơ trái
  if (leftSpeed > 10) { // Tiến
    pwm.setPWM(LEFT_FORWARD, 0, pwmLeft);
    pwm.setPWM(LEFT_BACKWARD, 0, 0);
  } else if (leftSpeed < -10) { // Lùi
    pwm.setPWM(LEFT_FORWARD, 0, 0);
    pwm.setPWM(LEFT_BACKWARD, 0, pwmLeft);
  } else { // Dừng
    pwm.setPWM(LEFT_FORWARD, 0, 0);
    pwm.setPWM(LEFT_BACKWARD, 0, 0);
  }

  // Điều khiển động cơ phải
  if (rightSpeed > 10) { // Tiến
    pwm.setPWM(RIGHT_FORWARD, 0, pwmRight);
    pwm.setPWM(RIGHT_BACKWARD, 0, 0);
  } else if (rightSpeed < -10) { // Lùi
    pwm.setPWM(RIGHT_FORWARD, 0, 0);
    pwm.setPWM(RIGHT_BACKWARD, 0, pwmRight);
  } else { // Dừng
    pwm.setPWM(RIGHT_FORWARD, 0, 0);
    pwm.setPWM(RIGHT_BACKWARD, 0, 0);
  }

  // --- Điều khiển Động cơ thứ ba (DC Motor) bằng nút R2 ---
  // Kích hoạt motor khi nhấn R2 và motor hiện không hoạt động
  if (ps2x.Button(PSB_R2) && !motor3_active) {
    motor3_active = true;         // Đặt cờ trạng thái là đang hoạt động
    motor3_startTime = millis();  // Ghi lại thời điểm bắt đầu
    pwm.setPWM(MOTOR3_FORWARD, 0, MOTOR3_SPEED); // Quay motor 3 chiều tiến
    pwm.setPWM(MOTOR3_BACKWARD, 0, 0);
    Serial.println("Motor 3: KICH HOAT (R2)");
  }

  // Kiểm tra thời gian hoạt động của motor 3
  if (motor3_active) { // Nếu motor 3 đang hoạt động
    if (millis() - motor3_startTime >= MOTOR3_DURATION) {
      // Nếu đã hết thời gian định trước
      pwm.setPWM(MOTOR3_FORWARD, 0, 0);   // Dừng motor 3
      pwm.setPWM(MOTOR3_BACKWARD, 0, 0);
      motor3_active = false;              // Đặt lại cờ trạng thái
      Serial.println("Motor 3: HET THOI GIAN, DUNG.");
    }
  }

  // --- Điều khiển Servo bằng nút L2 ---
  // Kích hoạt servo khi nhấn L2 và servo hiện không hoạt động
  if (ps2x.Button(PSB_L2) && !servo_active) {
    servo_active = true;          // Đặt cờ trạng thái là đang hoạt động
    servo_startTime = millis();   // Ghi lại thời điểm bắt đầu
    pwm.setPWM(SERVO_PIN, 0, SERVO_POS_ACTIVE); // Quay servo đến vị trí kích hoạt
    Serial.println("Servo: KICH HOAT (L2)");
  }

  // Kiểm tra thời gian hoạt động của servo
  if (servo_active) { // Nếu servo đang hoạt động theo lệnh thời gian
    if (millis() - servo_startTime >= SERVO_DURATION) {
      // Nếu đã hết thời gian định trước
      pwm.setPWM(SERVO_PIN, 0, SERVO_POS_REST); // Quay servo về vị trí nghỉ
      servo_active = false;                     // Đặt lại cờ trạng thái
      Serial.println("Servo: HET THOI GIAN, VE VI TRI NGHI.");
    }
  }

  // --- Debug giá trị ra Serial Monitor ---
  Serial.print("LY: "); Serial.print(y);
  Serial.print(" | RX: "); Serial.print(x);
  Serial.print(" | L_PWM: "); Serial.print(pwmLeft);
  Serial.print(" | R_PWM: "); Serial.println(pwmRight);

  delay(30); // Khoảng dừng nhỏ để đảm bảo độ ổn định và giảm tải CPU
}