// =====================================================================================
// 조향 모터 테스트 스케치 (RC + ROS2 통신)
// RC 수신기 Mode 스위치로 수동/자율 전환
// 자율 모드: Serial2로 13바이트 패킷 수신하여 조향 제어
// 수동 모드: RC 스틱으로 직접 조향 제어
// 시리얼 모니터 115200 baud, No line ending
// =====================================================================================

// --- 조향 모터 핀 ---
const int LMS = 5;        // 조향 모터 PWM
const int LMD = 24;       // 조향 모터 방향

// --- 포텐셔미터 핀 ---
const int POT_PIN = A8;

// --- RC 수신기 핀 ---
const int PIN_STEER    = A1;
const int PIN_MODE     = A4;

// --- 조향 한계값 (기존 코드 방식) ---
const int VAL_MAX_LEFT  = 1024;
const int VAL_MAX_RIGHT = 0;
const int VAL_NEUTRAL   = 526;
const float SAFETY_RATE  = 0.97;

// --- 자동 계산 변수 ---
int St_M;
int St_D;

// --- RC 조향 범위 ---
const int RC_CENTER = 1460;
const int RC_LEFT   = 1670;
const int RC_RIGHT  = 1250;
const int RC_DEADZONE = 50;

// --- 조향 PID (기존 코드와 동일) ---
float kp = 0.1;
float kd = 0.1;
float e, e_prev = 0;
float V;
float Vmax = 24;
unsigned long t, t_prev = 0;
int dt;

// --- 모드 ---
int res = 20;
byte AorM = 0;  // 0=수동, 1=자율
unsigned long modeCheckCount = 0;  // 자율 모드에서 모드 체크 주기용

// --- ROS 통신 변수 ---
byte rxBuf[13];
uint8_t rxIdx = 0;
byte Ham, Hes, Hge, Hal;
int Hsp, Hst;
int Con_status = 0;
unsigned long Last_Rx_Time = 0;
const unsigned long Rx_Timeout = 500;

// --- 피드백 송신 ---
char Senddata[13];
byte Lal = 0;
int Steer_Feed;
int encoder_val;

void setup() {
  Serial.begin(115200);     // USB 디버깅
  Serial2.begin(115200);    // ROS 통신
  Serial2.setTimeout(1);

  // 조향 범위 자동 계산 (기존 코드 방식)
  St_M = VAL_NEUTRAL;
  int range_left  = abs(VAL_MAX_LEFT - VAL_NEUTRAL);
  int range_right = abs(VAL_NEUTRAL - VAL_MAX_RIGHT);
  int min_range = (range_left < range_right) ? range_left : range_right;
  St_D = (int)(min_range * SAFETY_RATE);

  pinMode(LMS, OUTPUT);
  pinMode(LMD, OUTPUT);
  analogWrite(LMS, 0);
  digitalWrite(LMD, 0);

  Serial.println("========================================");
  Serial.println("  Steer Motor Test (RC + ROS2)");
  Serial.println("========================================");
  Serial.print("  Range: ");
  Serial.print(St_M - St_D);
  Serial.print(" ~ ");
  Serial.println(St_M + St_D);
  Serial.println("  RC Mode switch: Manual/Auto");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  // --- 포텐셔미터 읽기 ---
  encoder_val = analogRead(POT_PIN);

  int target;

  // --- 모드별 제어 ---
  if (AorM == 0) {
    // ===== 수동 모드: RC 수신기만 읽음 (pulseIn 블로킹 OK) =====
    int mode = pulseIn(PIN_MODE, HIGH, 30000);
    if (mode != 0) {
      mode = mode / res * res;
      if (mode < 1250) {
        AorM = 1;  // 자율 모드로 전환
        e_prev = 0;
        return;     // 다음 루프부터 자율 모드 진입
      }
    }

    int rc_steer = pulseIn(PIN_STEER, HIGH, 30000);

    if (rc_steer == 0) {
      analogWrite(LMS, 0);
      target = encoder_val;
    } else {
      if (rc_steer > RC_CENTER - RC_DEADZONE && rc_steer < RC_CENTER + RC_DEADZONE) {
        rc_steer = RC_CENTER;
      }
      target = map(rc_steer, RC_RIGHT, RC_LEFT, St_M - St_D, St_M + St_D);
      target = constrain(target, St_M - St_D, St_M + St_D);
      steerPID(target, encoder_val);
    }
  }
  else {
    // ===== 자율 모드: Serial2만 읽음 (pulseIn 없음, 블로킹 없음) =====
    // 100루프(~1초)마다 모드 스위치 짧게 확인 (수동 복귀용)
    modeCheckCount++;
    if (modeCheckCount >= 100) {
      modeCheckCount = 0;
      int mode = pulseIn(PIN_MODE, HIGH, 5000);  // 5ms 짧은 타임아웃
      if (mode != 0) {
        mode = mode / res * res;
        if (mode >= 1250) {
          AorM = 0;  // 수동 모드로 전환
          analogWrite(LMS, 0);
          e_prev = 0;
          return;
        }
      }
    }
    bool packet_ok = false;

    while (Serial2.available()) {
      byte b = Serial2.read();
      if (rxIdx == 0 && b != 'S') continue;
      rxBuf[rxIdx++] = b;

      if (rxIdx == 13) {
        if (rxBuf[0] == 'S' && rxBuf[1] == 'T' && rxBuf[2] == 'X' &&
            rxBuf[11] == 0x0D && rxBuf[12] == 0x0A) {
          packet_ok = true;
        } else {
          rxIdx = 0;
        }
        break;
      }
    }

    if (packet_ok) {
      rxIdx = 0;
      Last_Rx_Time = millis();
      Ham = rxBuf[3];
      Hes = rxBuf[4];
      Hge = rxBuf[5];
      Hsp = word(rxBuf[6], rxBuf[7]);
      Hst = word(rxBuf[8], rxBuf[9]);
      Hal = rxBuf[10];

      Hst = map(Hst, -2000, 2000, St_M + St_D, St_M - St_D);
    }

    if (millis() - Last_Rx_Time < Rx_Timeout) {
      Con_status = 1;
      if (Ham == 1) {
        target = Hst;
        target = constrain(target, St_M - St_D, St_M + St_D);
        steerPID(target, encoder_val);
      } else {
        target = St_M;
        steerPID(target, encoder_val);
      }
    } else {
      Con_status = 0;
      target = St_M;
      analogWrite(LMS, 0);  // 통신 끊김 → 정지
    }

    // 피드백 송신
    sendFeedback();
  }

  // --- 4. 디버깅 출력 ---
  Serial.print(AorM == 0 ? "MANUAL" : "AUTO  ");
  Serial.print("\tCon:");
  Serial.print(Con_status);
  Serial.print("\tPos:");
  Serial.print(encoder_val);
  Serial.print("\tTarget:");
  Serial.print(target);
  Serial.print("\t");

  int bar = map(encoder_val, 0, 1023, 0, 40);
  int neutral = map(St_M, 0, 1023, 0, 40);
  Serial.print("|");
  for (int i = 0; i < 40; i++) {
    if (i == neutral) {
      if (i == bar) Serial.print("@");
      else Serial.print("|");
    } else if (i == bar) {
      Serial.print("@");
    } else {
      Serial.print("-");
    }
  }
  Serial.println("|");

  delay(10);
}

// =====================================================================================
// 조향 PID 제어 (기존 SteerCon과 동일)
// =====================================================================================
void steerPID(int target, int pos) {
  // 오차가 충분히 작으면 모터 정지 (떨림 방지)
  if (abs(target - pos) <= 25) {
    analogWrite(LMS, 0);
    e_prev = 0;
    t_prev = millis();
    return;
  }

  t = millis();
  dt = t - t_prev;
  if (dt == 0) dt = 1;

  e = target - pos;
  V = kp * e + kd * (e - e_prev) / dt;

  if (V > Vmax) V = Vmax;
  if (V < -Vmax) V = -Vmax;

  int pwmOut = (int)(150.0 * abs(V) / Vmax);
  if (pwmOut > 150) pwmOut = 150;

  if (V > 0.5) {
    analogWrite(LMS, pwmOut);
    digitalWrite(LMD, 1);
  } else if (V < -0.5) {
    analogWrite(LMS, pwmOut);
    digitalWrite(LMD, 0);
  } else {
    analogWrite(LMS, 0);
  }

  t_prev = t;
  e_prev = e;
}

// =====================================================================================
// 피드백 송신 (기존 Send()와 동일 프로토콜)
// =====================================================================================
void sendFeedback() {
  Steer_Feed = map(encoder_val, St_M + St_D, St_M - St_D, -2000, 2000);
  Steer_Feed = constrain(Steer_Feed, -2000, 2000);

  Senddata[0] = 'S';
  Senddata[1] = 'T';
  Senddata[2] = 'X';
  Senddata[3] = AorM | 0x00;
  Senddata[4] = 0x00;             // ESTOP 없음 (조향만 테스트)
  Senddata[5] = 0x00;             // 기어 없음 (조향만 테스트)
  Senddata[6] = 0x00;             // RPM 상위 (구동 없음)
  Senddata[7] = 0x00;             // RPM 하위
  Senddata[8] = (Steer_Feed >> 8) | 0x00;
  Senddata[9] = Steer_Feed | 0x00;
  Senddata[10] = Lal | 0x00;
  Senddata[11] = 0x0D;
  Senddata[12] = 0x0A;

  Serial2.write(Senddata, 13);
  Lal = Lal + 1;
}
