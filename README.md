# -l-meandenstr-mantasyon
Bu proje, enkoderle motor hızını ölçüp PID kontrol ile referans değere sabitleyen kapalı çevrim bir sistemdir. Kesme tabanlı veri toplama sayesinde hassas ölçüm yapılır. Bluetooth/seri haberleşme ile anlık hız ve kontrol verileri izlenir. Ölçme, kontrol ve iletişim entegre bir otomasyon yapısı oluşturur.
/*
 * ============================================================
 *  KAPALI DÖNGÜ PID MOTOR HIZ KONTROL SİSTEMİ
 * ============================================================
 *  Donanım:
 *    - Arduino Nano (ATmega328P)
 *    - JGA12-N20B 12V 600RPM Enkoderli DC Motor
 *    - DRV8833 Çift Kanallı H-Köprüsü Motor Sürücü
 *    - HC-05 Bluetooth Modülü
 *    - 16x2 I2C LCD (adres: 0x27)
 *    - 2x 18650 Li-ion Pil (7.4V)
 *
 *  Pin Bağlantıları:
 *  ┌─────────────────────────────────────────────────┐
 *  │  Arduino Nano Pin  │  Bağlandığı Yer            │
 *  ├─────────────────────────────────────────────────┤
 *  │  D2  (INT0)        │  Enkoder Kanal A           │
 *  │  D3                │  Enkoder Kanal B (opsiyonel)│
 *  │  D5  (PWM)         │  DRV8833 IN1               │
 *  │  D6  (PWM)         │  DRV8833 IN2               │
 *  │  D10               │  HC-05 TX → Arduino RX     │
 *  │  D11               │  Arduino TX → Voltaj Bölücü│
 *  │                    │  → HC-05 RX                │
 *  │  A4  (SDA)         │  LCD I2C SDA               │
 *  │  A5  (SCL)         │  LCD I2C SCL               │
 *  │  VIN               │  Pil (+) 7.4V              │
 *  │  GND               │  Ortak GND                 │
 *  └─────────────────────────────────────────────────┘
 *
 *  Voltaj Bölücü (D11 → HC-05 RX):
 *    D11 ──[1kΩ]──┬──[2.2kΩ]── GND
 *                 └── HC-05 RX
 *    (5V → ~3.4V'a düşürür, HC-05 3.3V lojik için güvenli)
 *
 *  DRV8833 Bağlantıları:
 *    VCC  → Pil (+) 7.4V
 *    GND  → Ortak GND
 *    IN1  → D5
 *    IN2  → D6
 *    OUT1 → Motor terminal (+)
 *    OUT2 → Motor terminal (-)
 *
 *  Bluetooth Komutları (Seri monitör veya telefon uygulaması ile):
 *    "300"     → SetPoint'i 300 RPM'e ayarla
 *    "+"       → SetPoint'i 50 RPM artır
 *    "-"       → SetPoint'i 50 RPM azalt
 *    "S" / "s" → Motoru durdur (SetPoint = 0)
 *
 *  Yazar : Efe Işık
 *  Tarih : Nisan 2026
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// ======================== PIN TANIMLARI ========================
#define ENCODER_A_PIN   2     // Donanımsal kesme pini (INT0)
#define ENCODER_B_PIN   3     // Yön algılama (opsiyonel)
#define MOTOR_IN1       5     // DRV8833 IN1 (PWM)
#define MOTOR_IN2       6     // DRV8833 IN2 (PWM)
#define BT_RX_PIN       10    // HC-05 TX → Arduino RX
#define BT_TX_PIN       11    // Arduino TX → HC-05 RX (voltaj bölücü ile)

// ===================== MOTOR PARAMETRELERİ =====================
/*
 * JGA12-N20B enkoderinin çıkışı motor miline bağlıdır.
 * Tipik olarak 7 pals/devir (motor şaft) × dişli oranı = çıkış şaft PPR.
 * Eğer enkoderin tam PPR değerini bilmiyorsan, aşağıdaki kalibrasyon
 * adımlarını uygula:
 *   1) PULSES_PER_REV = 1 yap
 *   2) Motoru sabit PWM ile döndür, 1 tur elle say
 *   3) Seri monitörden okunan pals sayısını buraya yaz
 *
 * Yaygın değerler: 210, 330, 360, 420 (dişli oranına bağlı)
 */
#define PULSES_PER_REV  210   // ← Kendi motoruna göre kalibre et!
#define MAX_RPM         650   // Maksimum izin verilen RPM
#define RPM_STEP        50    // +/- komutlarıyla artış miktarı

// ===================== PID PARAMETRELERİ =======================
/*
 * Kp: Oransal kazanç — büyükse hızlı tepki ama overshoot artar.
 * Ki: İntegral kazanç  — yük altında kalıcı hatayı sıfırlar.
 * Kd: Türevsel kazanç  — titreşimi sönümler, aşırıysa gürültüye duyarlı.
 *
 * Ayarlama sırası (Ziegler-Nichols benzeri manuel yöntem):
 *   1) Ki=0, Kd=0 yap. Kp'yi artırarak sistemi salınıma getir.
 *   2) Salınım başladığı Kp değerinin ~%60'ını kullan.
 *   3) Ki'yi yavaş yavaş artırarak kalıcı hatayı yok et.
 *   4) Titreşim varsa Kd'yi küçük değerlerle ekle.
 */
float Kp = 1.2;
float Ki = 0.8;
float Kd = 0.05;

// =================== ZAMANLAMA SABİTLERİ =======================
#define PID_INTERVAL_MS     100   // PID hesaplama periyodu (ms)
#define LCD_INTERVAL_MS     250   // LCD güncelleme periyodu (ms)
#define SERIAL_INTERVAL_MS  200   // Seri debug çıktısı periyodu (ms)

// ===================== ANTI-WINDUP ============================
#define INTEGRAL_LIMIT  200.0     // İntegral birikiminin üst sınırı

// ======================== NESNELER ============================
LiquidCrystal_I2C lcd(0x27, 16, 2);    // I2C adres: 0x27 (yaygın)
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);

// ===================== GLOBAL DEĞİŞKENLER =====================
// -- Enkoder --
volatile long    pulseCount    = 0;     // ISR'da artırılır

// -- RPM --
float            currentRPM    = 0.0;
float            setPoint      = 0.0;   // Hedef RPM

// -- PID --
float            error         = 0.0;
float            prevError     = 0.0;
float            integral      = 0.0;
float            derivative    = 0.0;
int              pwmOutput     = 0;

// -- Zamanlama (millis tabanlı, delay YOK!) --
unsigned long    lastPidTime   = 0;
unsigned long    lastLcdTime   = 0;
unsigned long    lastSerialTime = 0;

// -- Bluetooth gelen veri tamponu --
String           btBuffer      = "";

// ======================== ISR (KESME) =========================
/*
 * Donanımsal kesme ile her yükselen kenarda pals sayısı artırılır.
 * ISR içinde mümkün olan en kısa kod yazılmalıdır.
 * volatile anahtar kelimesi, derleyicinin bu değişkeni optimize
 * etmemesini (register'da tutmamasını) sağlar.
 */
void encoderISR() {
  pulseCount++;
}

// ========================== SETUP =============================
void setup() {
  // -- Pin modları --
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // -- Motor başlangıçta durdurulmuş --
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);

  // -- Donanımsal kesme bağla --
  // RISING: Her yükselen kenarda ISR tetiklenir
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

  // -- Seri haberleşme --
  Serial.begin(9600);       // USB debug
  bluetooth.begin(9600);    // HC-05 varsayılan baud rate

  // -- LCD başlat --
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PID Motor Ctrl");
  lcd.setCursor(0, 1);
  lcd.print("Baslatiliyor...");

  // -- Başlangıç zamanı --
  lastPidTime    = millis();
  lastLcdTime    = millis();
  lastSerialTime = millis();

  Serial.println(F("=== PID Motor Kontrol Sistemi ==="));
  Serial.println(F("Komutlar: [sayi]=RPM, [+]=artir, [-]=azalt, [S]=durdur"));
}

// ========================== LOOP ==============================
void loop() {
  unsigned long now = millis();

  // -------- 1) BLUETOOTH KOMUT OKUMA --------
  readBluetooth();

  // -------- 2) PID HESAPLAMA (her PID_INTERVAL_MS'de bir) --------
  if (now - lastPidTime >= PID_INTERVAL_MS) {
    float dt = (now - lastPidTime) / 1000.0;   // saniye cinsinden
    lastPidTime = now;

    // -- RPM hesapla --
    noInterrupts();                   // ISR'yı kısa süreliğine durdur
    long pulses = pulseCount;         // Anlık pals sayısını kopyala
    pulseCount = 0;                   // Sayacı sıfırla
    interrupts();                     // ISR'yı tekrar aç

    // RPM = (pals / PPR) * (60 / dt)
    currentRPM = ((float)pulses / PULSES_PER_REV) * (60.0 / dt);

    // -- PID hesapla --
    computePID(dt);

    // -- Motoru sür --
    driveMotor(pwmOutput);
  }

  // -------- 3) LCD GÜNCELLEME (her LCD_INTERVAL_MS'de bir) --------
  if (now - lastLcdTime >= LCD_INTERVAL_MS) {
    lastLcdTime = now;
    updateLCD();
  }

  // -------- 4) SERİ DEBUG ÇIKTISI --------
  if (now - lastSerialTime >= SERIAL_INTERVAL_MS) {
    lastSerialTime = now;
    printDebug();
  }
}

// ===================== PID HESAPLAMA ==========================
void computePID(float dt) {
  // Hata = Hedef - Ölçülen
  error = setPoint - currentRPM;

  // İntegral (birikimli hata) — Anti-windup ile sınırlandırılmış
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Türev (hatanın değişim hızı)
  derivative = (error - prevError) / dt;
  prevError = error;

  // PID çıktısı
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // PWM sınırlandırma [0, 255]
  // Not: Bu kod tek yön dönüş için tasarlanmıştır.
  // Çift yön istenirse driveMotor() fonksiyonu güncellenmeli.
  pwmOutput = constrain((int)output, 0, 255);

  // SetPoint 0 ise motoru tamamen durdur ve integral sıfırla
  if (setPoint < 1.0) {
    pwmOutput  = 0;
    integral   = 0.0;
    prevError  = 0.0;
  }
}

// =================== MOTOR SÜRME ==============================
/*
 * DRV8833 Kontrol Tablosu:
 *   IN1=PWM, IN2=LOW  → İleri yön
 *   IN1=LOW, IN2=PWM  → Geri yön
 *   IN1=LOW, IN2=LOW  → Boşta (coast)
 *   IN1=HIGH,IN2=HIGH → Fren
 */
void driveMotor(int pwm) {
  if (pwm > 0) {
    analogWrite(MOTOR_IN1, pwm);
    analogWrite(MOTOR_IN2, 0);
  } else {
    // Motorun yavaşça durması için coast modu
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
  }
}

// ================ BLUETOOTH OKUMA =============================
void readBluetooth() {
  while (bluetooth.available()) {
    char c = bluetooth.read();

    // Satır sonu veya carriage return geldiğinde komutu işle
    if (c == '\n' || c == '\r') {
      if (btBuffer.length() > 0) {
        processCommand(btBuffer);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }

  // USB Seri'den de komut kabul et (debug için)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (btBuffer.length() > 0) {
        processCommand(btBuffer);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }
}

// ================ KOMUT İŞLEME ===============================
void processCommand(String cmd) {
  cmd.trim();

  if (cmd == "+" || cmd == "UP" || cmd == "up") {
    // RPM artır
    setPoint += RPM_STEP;
    if (setPoint > MAX_RPM) setPoint = MAX_RPM;

    bluetooth.print(F("SetPoint: "));
    bluetooth.println((int)setPoint);
    Serial.print(F(">> SetPoint arttirildi: "));
    Serial.println((int)setPoint);

  } else if (cmd == "-" || cmd == "DOWN" || cmd == "down") {
    // RPM azalt
    setPoint -= RPM_STEP;
    if (setPoint < 0) setPoint = 0;

    bluetooth.print(F("SetPoint: "));
    bluetooth.println((int)setPoint);
    Serial.print(F(">> SetPoint azaltildi: "));
    Serial.println((int)setPoint);

  } else if (cmd == "S" || cmd == "s" || cmd == "STOP" || cmd == "stop") {
    // Motoru durdur
    setPoint   = 0;
    integral   = 0;
    prevError  = 0;

    bluetooth.println(F("Motor DURDURULDU"));
    Serial.println(F(">> Motor durduruldu"));

  } else if (cmd == "P" || cmd == "p") {
    // Anlık PID parametrelerini göster
    bluetooth.print(F("Kp="));  bluetooth.print(Kp, 2);
    bluetooth.print(F(" Ki=")); bluetooth.print(Ki, 2);
    bluetooth.print(F(" Kd=")); bluetooth.println(Kd, 2);

  } else {
    // Sayısal RPM değeri denenecek
    float val = cmd.toFloat();
    if (val > 0 && val <= MAX_RPM) {
      setPoint = val;
      // İntegral sıfırlama: büyük set değişimlerinde birikim sorun yaratmasın
      integral  = 0;
      prevError = 0;

      bluetooth.print(F("Yeni SetPoint: "));
      bluetooth.println((int)setPoint);
      Serial.print(F(">> Yeni SetPoint: "));
      Serial.println((int)setPoint);

    } else if (val > MAX_RPM) {
      bluetooth.print(F("HATA: Maks RPM = "));
      bluetooth.println(MAX_RPM);

    } else {
      bluetooth.println(F("Gecersiz komut!"));
      Serial.print(F(">> Gecersiz: "));
      Serial.println(cmd);
    }
  }
}

// ================== LCD GÜNCELLEME ============================
void updateLCD() {
  // Satır 1: Anlık RPM
  lcd.setCursor(0, 0);
  lcd.print("RPM:");
  printPadded((int)currentRPM, 4);

  // PWM değerini de göster
  lcd.print(" PWM:");
  printPadded(pwmOutput, 3);

  // Satır 2: Hedef RPM ve hata
  lcd.setCursor(0, 1);
  lcd.print("SET:");
  printPadded((int)setPoint, 4);

  lcd.print(" e:");
  int errDisplay = (int)(setPoint - currentRPM);
  if (errDisplay >= 0) lcd.print("+");
  printPadded(abs(errDisplay), 3);
  if (errDisplay < 0) {
    // Negatif hata için geriye git ve - koy
    // (zaten printPadded abs değer yazdırdı)
  }
}

// Sayıyı belirli genişlikte yazdır (LCD'de kayma önleme)
void printPadded(int value, int width) {
  int digits = 1;
  int temp = abs(value);
  while (temp >= 10) { digits++; temp /= 10; }

  for (int i = 0; i < width - digits; i++) {
    lcd.print(" ");
  }
  lcd.print(value);
}

// =============== SERİ DEBUG ÇIKTISI ===========================
void printDebug() {
  Serial.print(F("RPM:"));
  Serial.print(currentRPM, 1);
  Serial.print(F(" | SET:"));
  Serial.print(setPoint, 0);
  Serial.print(F(" | ERR:"));
  Serial.print(error, 1);
  Serial.print(F(" | PWM:"));
  Serial.print(pwmOutput);
  Serial.print(F(" | I:"));
  Serial.println(integral, 2);
}
