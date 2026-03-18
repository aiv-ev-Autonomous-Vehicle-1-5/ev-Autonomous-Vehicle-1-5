// =====================================================================================
// Arduino Mega 2560 전체 핀 테스트 스케치
// 쉴드 분리 후 본체만 USB 연결하여 사용
// 시리얼 모니터 115200 baud로 열어서 결과 확인
// =====================================================================================

// =====================================================================================
// 테스트 모드 선택 (시리얼 모니터에서 번호 입력)
// 1: 아날로그 입력 테스트 (A0~A15)
// 2: 디지털 출력 테스트 (핀별 LED 점멸)
// 3: PWM 출력 테스트 (LED 밝기 변화)
// 4: 인터럽트 핀 테스트 (핀 2, 3)
// 5: 전체 아날로그 실시간 모니터링
// =====================================================================================

const int PWM_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46};
const int PWM_PIN_COUNT = 15;

// 디지털 핀 테스트 범위 (0,1은 Serial이 사용하므로 제외)
const int DIGITAL_START = 2;
const int DIGITAL_END = 53;

// 인터럽트 테스트용
volatile long intCount2 = 0;
volatile long intCount3 = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial); // USB 시리얼 연결 대기

  printMenu();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    // 버퍼 비우기 (엔터키 등 잔여 문자 제거)
    while (Serial.available()) Serial.read();

    switch (cmd) {
      case '1': testAnalog();       break;
      case '2': testDigitalOut();   break;
      case '3': testPWM();          break;
      case '4': testInterrupt();    break;
      case '5': monitorAnalog();    break;
      case '6': monitorDigital();  break;
      default:  printMenu();        break;
    }
  }
}

// =====================================================================================
// 메뉴 출력
// =====================================================================================
void printMenu() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("  Arduino Mega 2560 Pin Test");
  Serial.println("========================================");
  Serial.println("  1: Analog Input Test   (A0~A15)");
  Serial.println("  2: Digital Output Test (pin 2~53)");
  Serial.println("  3: PWM Output Test     (PWM pins)");
  Serial.println("  4: Interrupt Test      (pin 2, 3)");
  Serial.println("  5: Analog Realtime Monitor");
  Serial.println("  6: Digital Realtime Monitor");
  Serial.println("========================================");
  Serial.println(">> Enter number to start test:");
}

// =====================================================================================
// 테스트 1: 아날로그 입력 (A0~A15)
// - 각 핀을 읽어서 값 출력
// - GND에 터치하면 0 근처, 5V에 터치하면 1023 근처여야 정상
// - 아무것도 안 연결하면 부유(floating) 값 (불규칙하게 변동 = 정상)
// =====================================================================================
void testAnalog() {
  Serial.println("\n--- Analog Input Test (A0~A15) ---");
  Serial.println("Tip: Touch pin to GND -> ~0, to 5V -> ~1023");
  Serial.println("Floating(nothing connected) = random values = NORMAL");
  Serial.println();

  for (int i = 0; i < 16; i++) {
    int pin = A0 + i;
    int val = analogRead(pin);
    Serial.print("  A");
    Serial.print(i);
    if (i < 10) Serial.print(" ");
    Serial.print(" : ");
    Serial.print(val);
    Serial.print("\t");

    // 간단한 판정
    if (val == 0) {
      Serial.println("[LOW or GND]");
    } else if (val >= 1020) {
      Serial.println("[HIGH or 5V]");
    } else {
      Serial.println("[Floating/Signal]");
    }
  }

  Serial.println("\n>> Now touch each pin to GND and 5V to verify.");
  Serial.println(">> Send '1' again to re-read.");
  Serial.println(">> Send any other key for menu.");
}

// =====================================================================================
// 테스트 2: 디지털 출력 (핀 2~53)
// - 각 핀을 OUTPUT으로 설정 후 HIGH → LOW 토글
// - LED + 저항을 해당 핀에 연결하면 깜빡임 확인 가능
// - 또는 멀티미터로 전압 확인 (HIGH=5V, LOW=0V)
// =====================================================================================
void testDigitalOut() {
  Serial.println("\n--- Digital Output Test (pin 2~53) ---");
  Serial.println("Connect LED+resistor to test pin and GND.");
  Serial.println("Each pin will blink 3 times (200ms ON/OFF).");
  Serial.println();

  for (int pin = DIGITAL_START; pin <= DIGITAL_END; pin++) {
    // 핀 0,1은 Serial 사용 중이므로 스킵
    if (pin == 0 || pin == 1) continue;

    pinMode(pin, OUTPUT);

    Serial.print("  Pin ");
    if (pin < 10) Serial.print(" ");
    Serial.print(pin);
    Serial.print(" : Blinking... ");

    for (int j = 0; j < 3; j++) {
      digitalWrite(pin, HIGH);
      delay(200);
      digitalWrite(pin, LOW);
      delay(200);
    }

    Serial.println("DONE");

    // 핀을 INPUT으로 되돌려서 다른 테스트에 영향 안 주게
    pinMode(pin, INPUT);
  }

  Serial.println("\n>> Test complete. Check which pins did NOT blink.");
  Serial.println(">> Send any key for menu.");
}

// =====================================================================================
// 테스트 3: PWM 출력
// - PWM 가능 핀에 0 → 255 서서히 올림 (LED 밝기 증가)
// - LED + 저항을 해당 핀에 연결하여 확인
// =====================================================================================
void testPWM() {
  Serial.println("\n--- PWM Output Test ---");
  Serial.println("Connect LED+resistor to test pin and GND.");
  Serial.println("LED brightness will ramp up on each PWM pin.");
  Serial.println();

  for (int i = 0; i < PWM_PIN_COUNT; i++) {
    int pin = PWM_PINS[i];
    pinMode(pin, OUTPUT);

    Serial.print("  Pin ");
    if (pin < 10) Serial.print(" ");
    Serial.print(pin);
    Serial.print(" (PWM): Ramping... ");

    // 0 → 255 서서히 올리기
    for (int pwm = 0; pwm <= 255; pwm += 5) {
      analogWrite(pin, pwm);
      delay(10);
    }
    // 255 → 0 서서히 내리기
    for (int pwm = 255; pwm >= 0; pwm -= 5) {
      analogWrite(pin, pwm);
      delay(10);
    }

    analogWrite(pin, 0);
    pinMode(pin, INPUT);
    Serial.println("DONE");
  }

  Serial.println("\n>> Test complete. Check which pins did NOT ramp.");
  Serial.println(">> Send any key for menu.");
}

// =====================================================================================
// 테스트 4: 인터럽트 핀 (핀 2, 3)
// - 5초 동안 인터럽트 카운트 측정
// - 점퍼선으로 핀을 GND에 빠르게 터치하면 카운트 증가 = 정상
// =====================================================================================
void testInterrupt() {
  Serial.println("\n--- Interrupt Pin Test (pin 2, 3) ---");
  Serial.println("Use a jumper wire to tap pin 2 or 3 to GND quickly.");
  Serial.println("Each tap should increase the count.");
  Serial.println("Counting for 10 seconds...");
  Serial.println();

  intCount2 = 0;
  intCount3 = 0;

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), isr2, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), isr3, FALLING);

  unsigned long start = millis();
  while (millis() - start < 10000) {
    Serial.print("  Pin2: ");
    Serial.print(intCount2);
    Serial.print("  |  Pin3: ");
    Serial.println(intCount3);
    delay(500);
  }

  detachInterrupt(digitalPinToInterrupt(2));
  detachInterrupt(digitalPinToInterrupt(3));
  pinMode(2, INPUT);
  pinMode(3, INPUT);

  Serial.println();
  Serial.print(">> Final - Pin2: ");
  Serial.print(intCount2);
  Serial.print("  Pin3: ");
  Serial.println(intCount3);
  Serial.println(">> If counts increased when tapped -> pin OK");
  Serial.println(">> Send any key for menu.");
}

void isr2() { intCount2++; }
void isr3() { intCount3++; }

// =====================================================================================
// 테스트 5: 아날로그 실시간 모니터링
// - A0~A15 값을 500ms 간격으로 계속 출력
// - 핀을 GND/5V에 번갈아 터치하면서 반응 확인
// - 아무 키 입력하면 중지
// =====================================================================================
void monitorAnalog() {
  Serial.println("\n--- Analog Realtime Monitor (A0~A15) ---");
  Serial.println("Touch pins to GND or 5V to see value changes.");
  Serial.println("Send any key to stop.");
  Serial.println();

  // 헤더 출력
  Serial.print("     ");
  for (int i = 0; i < 16; i++) {
    Serial.print("A");
    Serial.print(i);
    if (i < 10) Serial.print("   ");
    else Serial.print("  ");
  }
  Serial.println();

  while (!Serial.available()) {
    Serial.print("  >> ");
    for (int i = 0; i < 16; i++) {
      int val = analogRead(A0 + i);
      if (val < 10) Serial.print("   ");
      else if (val < 100) Serial.print("  ");
      else if (val < 1000) Serial.print(" ");
      Serial.print(val);
      Serial.print(" ");
    }
    Serial.println();
    delay(500);
  }

  // 버퍼 비우기
  while (Serial.available()) Serial.read();
  Serial.println("\n>> Monitoring stopped.");
  Serial.println(">> Send any key for menu.");
}

// =====================================================================================
// 테스트 6: 디지털 핀 실시간 모니터링
// - 핀 2~53을 INPUT_PULLUP으로 설정 (기본 HIGH)
// - GND에 터치하면 0으로 바뀜 = 정상
// - 반응 없으면 해당 핀 고장
// - 아무 키 입력하면 중지
// =====================================================================================
void monitorDigital() {
  Serial.println("\n--- Digital Realtime Monitor (pin 2~53) ---");
  Serial.println("All pins set to INPUT_PULLUP (default = 1)");
  Serial.println("Touch pin to GND -> should change to 0");
  Serial.println("Send any key to stop.");
  Serial.println();

  // 핀 2~53을 INPUT_PULLUP으로 설정
  for (int pin = 2; pin <= 53; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }

  // 헤더 출력
  Serial.print("  ");
  for (int pin = 2; pin <= 53; pin++) {
    if (pin < 10) Serial.print(" ");
    Serial.print(pin);
    Serial.print(" ");
  }
  Serial.println();

  while (!Serial.available()) {
    Serial.print("  ");
    for (int pin = 2; pin <= 53; pin++) {
      int val = digitalRead(pin);
      if (pin < 10) Serial.print(" ");
      Serial.print(val);
      Serial.print(" ");
    }
    Serial.println();
    delay(500);
  }

  // 핀 되돌리기
  for (int pin = 2; pin <= 53; pin++) {
    pinMode(pin, INPUT);
  }

  // 버퍼 비우기
  while (Serial.available()) Serial.read();
  Serial.println("\n>> Monitoring stopped.");
  Serial.println(">> Send any key for menu.");
}
