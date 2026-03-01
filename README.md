# -l-meandenstr-mantasyon
Bu proje, enkoderle motor hızını ölçüp PID kontrol ile referans değere sabitleyen kapalı çevrim bir sistemdir. Kesme tabanlı veri toplama sayesinde hassas ölçüm yapılır. Bluetooth/seri haberleşme ile anlık hız ve kontrol verileri izlenir. Ölçme, kontrol ve iletişim entegre bir otomasyon yapısı oluşturur.
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// --- Pin Tanımlamaları ---
const int encoderPin = 2;   // Phase A - Interrupt Pin
const int motorIN1 = 9;     // PWM Pin
const int motorIN2 = 8;     // Direction Pin

// --- PID Değişkenleri ---
volatile long pulseCount = 0;
double setPoint = 150.0;    // Hedef RPM
double inputRPM = 0;
double outputPWM = 0;
double error, lastError, integral, derivative;

// --- PID Katsayıları (Kalibrasyon Gerektirir) ---
double Kp = 1.2, Ki = 0.5, Kd = 0.1; 

unsigned long lastTime;
double dt = 0.1; // 100ms örnekleme zamanı

void countPulse() {
  pulseCount++; // Donanımsal Kesme Servisi
}

void setup() {
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);
  
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  
  Serial.begin(9600); // Bluetooth ve Debug için
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 100) { // Örnekleme periyodu
    
    // 1. Hız Ölçümü (RPM Hesaplama)
    noInterrupts();
    long currentPulses = pulseCount;
    pulseCount = 0;
    interrupts();
    
    // N20 motorun tur başına puls sayısına göre RPM hesaplanır
    inputRPM = (currentPulses * 60.0) / (7.0 * 300.0 * dt); // Örnek formül

    // 2. PID Algoritması
    error = setPoint - inputRPM;
    integral += error * dt;
    // Anti-Windup (Hocanın istediği yük testi için kritik)
    if(integral > 255) integral = 255; 
    
    derivative = (error - lastError) / dt;
    
    outputPWM = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // 3. Motor Sürücüye Yazma (DRV8833)
    if (outputPWM > 255) outputPWM = 255;
    if (outputPWM < 0) outputPWM = 0;
    
    analogWrite(motorIN1, (int)outputPWM);
    digitalWrite(motorIN2, LOW);

    lastError = error;
    lastTime = currentTime;
    
    // 4. Telemetri (Debug)
    Serial.print("Target: "); Serial.print(setPoint);
    Serial.print(" Current: "); Serial.println(inputRPM);
  }
}
