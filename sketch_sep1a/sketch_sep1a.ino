#include <Arduino.h>
#define XCLK_PIN 32   // Chân ESP32 xuất XCLK -> nối vào XCLK camera
// ================== PHẦN 1: CHÂN KẾT NỐI ==================
#define OV7670_PIN_PCLK   27
#define OV7670_PIN_VSYNC  25
#define OV7670_PIN_HREF   26
#define OV7670_PIN_XCLK   32
#define OV7670_PIN_D0     33
#define OV7670_PIN_D1     12
#define OV7670_PIN_D2     13
#define OV7670_PIN_D3     14
#define OV7670_PIN_D4     15
#define OV7670_PIN_D5     2
#define OV7670_PIN_D6     4
#define OV7670_PIN_D7     5
#define OV7670_PIN_SIOC   22
#define OV7670_PIN_SIOD   21

// ================== PHẦN 2: KHAI BÁO OV7670 DRIVER ==================
#include "OV7670.h"
#define FRAME_WIDTH  160
#define FRAME_HEIGHT 120

OV7670 camera(
  OV7670_PIN_SIOC, OV7670_PIN_SIOD,
  OV7670_PIN_XCLK, OV7670_PIN_PCLK,
  OV7670_PIN_VSYNC, OV7670_PIN_HREF,
  OV7670_PIN_D0, OV7670_PIN_D1, OV7670_PIN_D2, OV7670_PIN_D3,
  OV7670_PIN_D4, OV7670_PIN_D5, OV7670_PIN_D6, OV7670_PIN_D7,
  FRAME_WIDTH, FRAME_HEIGHT
);

// ================== PHẦN 3: QUIRC QR DECODER ==================
extern "C" {
  #include <quirc.h>
}

struct quirc *qr;
uint8_t *gray_buf;
uint8_t *frame;  // ✅ chuyển sang con trỏ để cấp phát động

// ================== PHẦN 4: HÀM CHUYỂN ẢNH ==================
void convertToGrayscale(uint8_t *rgb565, uint8_t *gray, int w, int h) {  ////uint8_t*rgb565: con trỏ về mảng byte ảnh chứa dữ liệu kiểu ảnh rgb565
  for (int i = 0; i < w * h; i++) {                                      ///uint8_t*gray: con trỏ về mảng chứa ảnh xám
    uint16_t pixel = ((uint16_t*)rgb565)[i];   //lấy giá trị 16bit của một pixel màu
    uint8_t r = ((pixel >> 11) & 0x1F) << 3;   //tách 5 bit màu đỏ,sau đó dịch sang trái 3 bit để mở rộng thành 8bit
    uint8_t g = ((pixel >> 5) & 0x3F) << 2;    //tách 6bit màu xanh lá,dịch trái 2bit để mở rộng thành 8bit
    uint8_t b = (pixel & 0x1F) << 3;           //tách 5bit màu xanh dương,dịch trái 3bit để mở rộng thành 8bit
    gray[i] = (r * 30 + g * 59 + b * 11) / 100;//công thức chuyển đổi từ màu RGB sang giá trị độ sáng cho ảnh xám
  }
}

// ================== PHẦN 5: SETUP ==================
//////////////
volatile bool vsyncFlag = false;

void IRAM_ATTR vsyncISR() {
  vsyncFlag = true;
}
///////////

void setup() {
////////
  



/////////
  Serial.begin(115200);
  delay(2000);
////////
Serial.println("🚀 Bắt đầu test XCLK cho OV7670...");

  // ===== Tạo PWM trên chân XCLK =====
  ledcAttachPin(XCLK_PIN, 0);       // Channel 0
  ledcSetup(0, 8000000, 1);       // 8 MHz, độ phân giải 1 bit (chỉ 0/1)
  ledcWrite(0, 1);                  // Bật PWM

  Serial.println("✅ XCLK đã được xuất ra pin 32 (8 MHz)");

  // ===== Gọi hàm setPCLK trong thư viện =====
  camera.setPCLK(1, 0); // VD: chia clock /2, không dùng PLL
  Serial.println("📡 Đã cấu hình PCLK (chia 2, PLL bypass).");

  // Báo hiệu test chạy
  pinMode(2, OUTPUT); // LED on-board
  //////
  
  Serial.println("🚀 Khởi động camera OV7670...");

  qr = quirc_new();
  if (!qr) {
    Serial.println("❌ Không tạo được quirc struct");
    while (1);
  }

  if (quirc_resize(qr, FRAME_WIDTH, FRAME_HEIGHT) < 0) {
    Serial.println("❌ Không resize được quirc");
    while (1);
  }

  gray_buf = (uint8_t*)malloc(FRAME_WIDTH * FRAME_HEIGHT);
  if (!gray_buf) {
    Serial.println("❌ Không cấp phát được bộ nhớ grayscale");
    while (1);
  }

  frame = (uint8_t*)malloc(FRAME_WIDTH * FRAME_HEIGHT);  // ✅ cấp phát động
  if (!frame) {
    Serial.println("❌ Không cấp phát được bộ nhớ frame");
    while (1);
  }

  Serial.println("✅ Camera và quirc đã sẵn sàng!");
}

// ================== PHẦN 6: LOOP ==================
void loop() {
  camera.getFrame(frame);
  //
  delay(100);
  //
  

  Serial.println("📸 Kiểm tra dữ liệu ảnh:");
for (int i = 0; i < 10; i++) {
  uint16_t pixel = ((uint16_t*)frame)[i];
  Serial.printf("Pixel %d: 0x%04X\n", i, pixel);
}

  convertToGrayscale(frame, gray_buf, FRAME_WIDTH, FRAME_HEIGHT);
  int brightness = 0;
  for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
    brightness += gray_buf[i];
}
  brightness /= (FRAME_WIDTH * FRAME_HEIGHT);
  Serial.printf("🌕 Độ sáng trung bình: %d\n", brightness);
  Serial.println("🧪 Mẫu ảnh xám:");
for (int i = 0; i < 20; i++) {
  Serial.printf("%3d ", gray_buf[i]);
}
Serial.println();


//////////
if (vsyncFlag) {
  Serial.println("📶 VSYNC báo frame mới!");
  vsyncFlag = false;
}
/////////////////


  uint8_t *qimage = quirc_begin(qr, NULL, NULL);
  memcpy(qimage, gray_buf, FRAME_WIDTH * FRAME_HEIGHT);
  quirc_end(qr);

  int count = quirc_count(qr);
  if (count > 0) {
    Serial.printf("📷 Phát hiện %d QR code!\n", count);
    for (int i = 0; i < count; i++) {
      struct quirc_code *code = (struct quirc_code*)malloc(sizeof(struct quirc_code));
      struct quirc_data *data = (struct quirc_data*)malloc(sizeof(struct quirc_data));

      if (code && data) {
        quirc_extract(qr, i, code);
        if (!quirc_decode(code, data)) {
          Serial.print("🔍 QR content: ");
          Serial.println((char*)data->payload);
        }
      }

      free(code);
      free(data);
    }
  } else {
    Serial.println("...Không thấy QR");
  }

  delay(1000);
}



