// =====================================================================================
// Serial2 통신 테스트 스케치
// PC(USB-to-RS232) ↔ Arduino Serial2 (핀 16, 17) 연결 확인용
// 시리얼 모니터 115200 baud, No line ending
// =====================================================================================

void setup() {
  Serial.begin(115200);    // USB 디버깅
  Serial2.begin(115200);   // RS232 통신
  Serial2.setTimeout(1);

  Serial.println("========================================");
  Serial.println("  Serial2 Connection Test");
  Serial.println("========================================");
  Serial.println("  Sending 'PING' every 1 second via Serial2");
  Serial.println("  Waiting for any data from PC...");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  // --- Arduino → PC: 1초마다 PING 전송 ---
  Serial2.println("PING");
  Serial.println("[TX] Sent 'PING' via Serial2");

  // --- PC → Arduino: 수신 데이터 확인 ---
  delay(100);
  if (Serial2.available()) {
    Serial.print("[RX] Received: ");
    while (Serial2.available()) {
      char c = Serial2.read();
      Serial.print(c);
    }
    Serial.println();
  } else {
    Serial.println("[RX] No data from PC");
  }

  delay(900);
}
