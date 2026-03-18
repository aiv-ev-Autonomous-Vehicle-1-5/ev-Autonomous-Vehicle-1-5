// =====================================================================================
// RC 수신기 테스트 스케치
// A0(Throttle), A1(Steer), A4(Mode), A5(Gear) 4채널 실시간 모니터링
// 시리얼 모니터 115200 baud, No line ending
// =====================================================================================

const int PIN_THROTTLE = A0;
const int PIN_STEER    = A1;
const int PIN_MODE     = A4;
const int PIN_GEAR     = A5;

int throttle, steer, mode, gear;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("========================================");
  Serial.println("  RC Receiver Test (4ch)");
  Serial.println("========================================");
  Serial.println("  A0: Throttle");
  Serial.println("  A1: Steer");
  Serial.println("  A4: Mode");
  Serial.println("  A5: Gear");
  Serial.println("========================================");
  Serial.println("  Normal range: 1000~2000 us");
  Serial.println("  Center: ~1500 us");
  Serial.println("  0 = No signal (timeout)");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Throttle\tSteer\t\tMode\t\tGear");
}

void loop() {
  throttle = pulseIn(PIN_THROTTLE, HIGH, 30000);
  steer    = pulseIn(PIN_STEER,    HIGH, 30000);
  mode     = pulseIn(PIN_MODE,     HIGH, 30000);
  gear     = pulseIn(PIN_GEAR,     HIGH, 30000);

  Serial.print(throttle);
  Serial.print("\t\t");
  Serial.print(steer);
  Serial.print("\t\t");
  Serial.print(mode);
  Serial.print("\t\t");
  Serial.println(gear);

  delay(100);
}
