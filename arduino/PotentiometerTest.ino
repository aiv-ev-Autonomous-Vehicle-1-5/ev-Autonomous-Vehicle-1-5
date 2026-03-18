// =====================================================================================
// 조향 포텐셔미터 테스트 스케치
// A8 핀에서 포텐셔미터 값을 실시간으로 읽어서 시리얼 모니터에 출력
// 시리얼 모니터 115200 baud, No line ending
// =====================================================================================

const int POT_PIN = A8;

// 기존 코드 기준 설정값 (참고용)
const int VAL_MAX_LEFT  = 990;   // 좌회전 한계
const int VAL_MAX_RIGHT = 20;    // 우회전 한계
const int VAL_NEUTRAL   = 526;   // 직진 중립값

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("========================================");
  Serial.println("  Steering Potentiometer Test (A8)");
  Serial.println("========================================");
  Serial.println("  Expected values:");
  Serial.println("    Left limit  : ~990");
  Serial.println("    Neutral     : ~526");
  Serial.println("    Right limit : ~20");
  Serial.println("========================================");
  Serial.println("  Turn the wheel and check values.");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  int val = analogRead(POT_PIN);

  Serial.print("A8 = ");
  Serial.print(val);
  Serial.print("\t");

  // 위치 판정
  if (abs(val - VAL_NEUTRAL) < 20) {
    Serial.print("[CENTER]");
  } else if (val > VAL_NEUTRAL) {
    Serial.print("[LEFT ]");
  } else {
    Serial.print("[RIGHT]");
  }

  // 바 그래프 표시 (0~1023을 50칸으로 매핑)
  Serial.print("  |");
  int bar = map(val, 0, 1023, 0, 50);
  for (int i = 0; i < 50; i++) {
    if (i == map(VAL_NEUTRAL, 0, 1023, 0, 50)) {
      if (i == bar) Serial.print("@");  // 현재 위치가 중립과 겹침
      else Serial.print("|");           // 중립 위치 표시
    } else if (i == bar) {
      Serial.print("@");                // 현재 위치
    } else {
      Serial.print("-");
    }
  }
  Serial.println("|");

  delay(100);
}
