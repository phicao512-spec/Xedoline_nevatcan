# Xe Dò Line & Tránh Vật Cản (Line Follower & Obstacle Avoidance Robot)

Dự án xe dò line kết hợp tránh vật cản sử dụng thuật toán PID để bám vạch và điều khiển cân bằng tốc độ hai bánh xe thông qua encoder, kết hợp với State Machine để xử lý tránh vật cản tự động.

## 1. Cấu hình phần cứng (Pinout)

- **Cảm biến dò line (5 mắt):** 
  - SS1: 36
  - SS2: 39
  - SS3: 34
  - SS4: 35
  - SS5: 13
  - *Ngưỡng nhận diện (SENSOR_THRESHOLD):* 2500

- **Điều khiển động cơ (Motor Driver):**
  - **Động cơ Trái (Motor Left):** PWM: 17, BIN1: 16, BIN2: 4
  - **Động cơ Phải (Motor Right):** PWM: 21, AIN1: 19, AIN2: 18
  - *Tần số PWM:* 20kHz, Độ phân giải: 8-bit

- **Encoder (Đo tốc độ động cơ):**
  - Trái: ENCA_L (23), ENCB_L (22)
  - Phải: ENCA_R (27), ENCB_R (26)

- **Cảm biến siêu âm (Ultrasonic):**
  - Trig: 33
  - Echo: 25

## 2. Lý thuyết điều khiển & Số liệu thiết lập

### 2.1 Thuật toán PID Dò Line (Line Follower PID)
Xe sử dụng bộ điều khiển PID cơ bản để tính toán sai số từ 5 mắt cảm biến, từ đó bù trừ tốc độ giữa 2 bánh xe để giữ xe luôn ở giữa vạch.
- **Tốc độ cơ sở (BASE_SPEED):** `115`
- **Hệ số Kp (Proportional):** `27.0`
- **Hệ số Ki (Integral):** `0.01`
- **Hệ số Kd (Derivative):** `22.0`
*Tốc độ sẽ tự động giảm đi (động lực học) khi sai số lệch vạch lớn.*

### 2.2 Thuật toán PID Cân Bằng Tốc Độ (Speed Balance PID)
Khi xe đi thẳng (sai số dò line rất nhỏ), bộ điều khiển tự động kích hoạt PID cân bằng tốc độ sử dụng dữ liệu từ encoder. Điều này giúp hai bánh chạy đồng tốc, tránh việc xe bị lệch hướng do sai số cơ khí của động cơ.
- **Khoảng thời gian lấy mẫu:** `50ms`
- **Hệ số Kp_spd:** `0.8`
- **Hệ số Ki_spd:** `0.05`
- **Hệ số Kd_spd:** `0.01`

### 2.3 Logic Tránh Vật Cản (Obstacle Avoidance)
Hệ thống giám sát cảm biến siêu âm liên tục. Khi phát hiện vật cản một cách chắc chắn (4/5 lần đo liên tiếp < 15cm), xe sẽ dừng và kích hoạt State Machine để đi vòng qua vật cản.
- **Khoảng cách phát hiện (OBSTACLE_DIST):** `15 cm`
- **Tốc độ di chuyển khi tránh (AVOID_SPEED):** `150`
- **Quy trình tránh vật cản (8 State):**
  1. Lùi lại một chút.
  2. Quay trái.
  3. Tiến thẳng.
  4. Quay phải (lần 1).
  5. Tiến thẳng vượt qua chiều dài vật cản.
  6. Quay phải (lần 2).
  7. Tiến thẳng hướng về phía vạch line.
  8. Phát hiện vạch line, quay trái để căn chỉnh lại tư thế, kết thúc quá trình và trở về chế độ dò line bình thường.
  
*(Lưu ý: Sau khi hoàn thành tránh vật cản, tất cả các tham số tích phân (I) và sai số (error) của thuật toán PID đều được reset để tránh xe bị giật cục.)*
