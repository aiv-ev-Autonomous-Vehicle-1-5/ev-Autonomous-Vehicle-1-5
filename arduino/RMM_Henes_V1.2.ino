// =====================================================================================
// LiquidCrystal 라이브러리: 16x2 LCD 화면을 제어하기 위한 아두이노 표준 라이브러리
// =====================================================================================
#include <LiquidCrystal.h>

// ===================================================================================
// [USER CONFIGURATION] 플랫폼별 측정값을 여기서 수정하세요
// ===================================================================================

// 1. 조향 포텐셔미터(가변저항)의 물리적 한계값
//    → 아두이노 전원만 켜고 손으로 바퀴를 끝까지 좌/우로 돌린 뒤
//      시리얼 모니터에서 읽은 analogRead(A8) 값을 적으세요.
//    [참고] 각 차량별 측정값:
//      세종대 : 1010, 20, 520
//      충남대 : 1000, 35, 510
//      청주대 : 960, 20, 518
const int VAL_MAX_LEFT  = 990;   // 좌회전 물리적 한계값 (포텐셔미터 최댓값 쪽)
const int VAL_MAX_RIGHT = 20;    // 우회전 물리적 한계값 (포텐셔미터 최솟값 쪽)

// 2. 바퀴를 일자(직진)로 정렬했을 때의 센서값 (중립값)
const int VAL_NEUTRAL   = 526;   // 직접 보고 맞춘 직진(중립) 값

// 3. 기구 보호를 위한 안전율 (0.97 = 물리적 한계의 97%까지만 사용)
//    → 1.0이면 한계까지 꽉 쓰고, 작을수록 여유를 더 둠
const float SAFETY_RATE = 0.97;
// ===================================================================================

// 자동 계산 변수 (수정하지 마세요)
// → setup()에서 위의 USER CONFIGURATION 값을 바탕으로 자동 계산됨
int St_M; // 조향 중립 위치 (= VAL_NEUTRAL)
int St_D; // 중립에서 좌/우로 허용되는 최대 편차 (안전율 적용 후)
          // 실제 조향 범위: [St_M - St_D] ~ [St_M + St_D]


// =====================================================================================
// LCD 화면 관련 변수
// =====================================================================================
// LiquidCrystal(RS핀, E핀, DB4핀, DB5핀, DB6핀, DB7핀)
// → 4비트 모드로 16x2 LCD를 제어. 아두이노 메가의 디지털 30~35번 핀 사용.
LiquidCrystal lcd(30,31,32,33,34,35);

// =====================================================================================
// RC 수신기 관련 변수
// =====================================================================================
// RC 조종기에서 수신기로 전달되는 PWM 채널값 (단위: μs, 보통 1000~2000 범위)
int steer;     // 조향 채널 (좌우 스틱)
int throttle;  // 스로틀 채널 (속도 스틱)
int elev;      // 엘리베이터 채널 (현재 미사용)
int rudd;      // 러더 채널 (현재 미사용)
int mode;      // 모드 스위치 채널 (수동/자율/디버깅 전환)
int gear;      // 기어 스위치 채널 (전진/중립/후진)

// 수신기 노이즈 제거용 상수
// → 수신된 값을 res(20) 단위로 반올림하여 미세한 떨림을 제거
//   예: 1523 → 1523/20*20 = 1520
int res = 20;

// 루프 카운터: 매 루프마다 1씩 증가하며, i%4로 수신기 채널을 번갈아 읽음
// → 한 루프에 모든 채널을 읽으면 pulseIn() 대기 시간 때문에 루프가 느려지므로
//   4번에 나눠서 한 채널씩 읽는 방식
unsigned long i = 0;

// =====================================================================================
// 플랫폼 제어 관련 변수
// =====================================================================================
int Speed;            // 최종 구동 모터 속도 명령값 (0~250)
int Steer;            // 최종 조향 모터 위치 명령값 (포텐셔미터 단위)

int Max_Speed = 250;  // 플랫폼의 최대 속도값 (PWM 범위 0~255)

byte Gear;  // 최종 기어 명령값: 0=전진(D), 1=중립(N), 2=후진(R)
byte AorM;  // 동작 모드: 0=수동주행, 1=자율주행, 2=디버깅
byte ESTOP; // 긴급정지 상태: 0=정상, 1=정지

// 모터 속도 이동평균 필터용 변수 (최근 5개 값의 평균)
// → 급격한 속도 변화를 부드럽게 만들어 줌
int lsp1,lsp2,lsp3,lsp4,lsp5 = 0;

// 하위(L=Low level, 수신기) 및 상위(H=High level, PC/ROS) 제어 명령값
int Lsp;  // Low-level 속도 (수신기에서 온 속도)
int Lst;  // Low-level 조향 (수신기에서 온 조향)
int Hsp;  // High-level 속도 (PC에서 온 속도)
int Hst;  // High-level 조향 (PC에서 온 조향)
int Hsp_p, Hst_p; // 이전 값 저장용 (현재 미사용)

// 하위/상위 제어의 기어, 비상정지, 모드, alive 변수
byte Lge;  // Low-level 기어
byte Les;  // Low-level 비상정지 (수신기 스로틀 역방향으로 트리거)
byte Ham;  // High-level 자율주행 모드 활성화 (1=활성)
byte Hge;  // High-level 기어
byte Hes;  // High-level 비상정지
byte Hal;  // High-level alive 카운터 (통신 살아있는지 확인용)
byte Ham_p, Hge_p, Hes_p, Hal_p; // 이전 값 저장용
byte Lal = 0;  // Low-level alive 카운터 (Send()에서 매번 +1 증가)

int Steer_Feed; // 조향 피드백값: 현재 조향 위치를 -2000~2000으로 변환한 값
int RPM;        // 구동 모터의 현재 RPM (바퀴 기준)

// =====================================================================================
// 시리얼 통신 패킷 관련 변수
// =====================================================================================
// 13바이트 고정 길이 패킷 구조:
//   [0]S [1]T [2]X [3]데이터... [11]0x0D [12]0x0A
//   → 'S','T','X'는 헤더, 0x0D 0x0A는 푸터(CR LF)

// PC(ROS) → 아두이노 수신 패킷: S T X Ham Hes Hge Hsp(2byte) Hst(2byte) Hal 0D 0A
char Receivedata[13];

// 아두이노 → PC(ROS) 송신 패킷: S T X AorM Les Lge RPM(2byte) SteerFeed(2byte) Lal 0D 0A
char Senddata[13];

int Con_status = 0; // 상위 제어기 통신 상태: 0=끊김, 1=정상

// 수신 버퍼 및 인덱스 (바이트 단위로 패킷을 조립하는 데 사용)
byte rxBuf[13];   // 수신 중인 패킷을 임시 저장하는 버퍼
uint8_t rxIdx = 0; // 현재까지 받은 바이트 수 (0~12)


// =====================================================================================
// 모터 핀 정의
// =====================================================================================
// [주의] 변수명이 LMS/RMS(Left/Right)로 되어있지만, 실제로는:
//   LMS/LMD = 조향(Steering) 모터
//   RMS/RMD = 구동(Drive) 모터
// 변수명이 원래 좌/우 모터 기준으로 지어졌다가, 조향/구동으로 용도가 바뀐 것.

// 조향 모터 핀 (DC모터 + 포텐셔미터로 위치 제어)
const int LMS = 5;    // 조향 모터 Speed → analogWrite()로 PWM 출력 (회전 세기)
const int LMD = 24;   // 조향 모터 Direction → digitalWrite()로 회전 방향 결정

// 구동 모터 핀 (DC모터 + 쿼드러처 엔코더로 속도 제어)
const int RMS = 4;    // 구동 모터 Speed → analogWrite()로 PWM 출력
const int RMD = 22;   // 구동 모터 Direction → 0=전진, 1=후진

// =====================================================================================
// 구동 모터 엔코더 및 PID 속도 제어 변수
// =====================================================================================

// --- 엔코더 핀 ---
// 쿼드러처 엔코더: A, B 두 채널로 회전 방향과 속도를 측정
const int interruptPinA = 3;  // 엔코더 A채널 (인터럽트 핀)
const int interruptPinB = 2;  // 엔코더 B채널 (인터럽트 핀)

// volatile: ISR(인터럽트 서비스 루틴)에서 값이 변경되므로,
//           컴파일러가 최적화로 값을 캐시하지 못하게 하는 키워드
volatile long EncoderCount_R = 0;  // 현재 엔코더 누적 카운트 (ISR에서 증감)
long EncoderCount_prev_R = 0;      // 이전 루프의 엔코더 카운트 (속도 계산용)

// --- PID 변수 (구동 모터) ---
int E_R = 0;          // 한 루프 동안의 엔코더 변화량 = 현재 실제 속도
int Sp_MAX_R = 15;    // 목표 속도 상한 (엔코더 카운트/루프 단위)
int Sp_MIN_R = -15;   // 목표 속도 하한 (음수 = 후진)
int Sp_encoder_pot_R; // (현재 미사용)
int Sp_val_R;         // 현재 목표 속도 (기어 방향 반영 후)
int Sp_encoder_val_R; // PID 계산에 넣을 현재 속도 (= E_R)

// PID 게인값
// → P만 사용하고 I, D는 거의 0 (사실상 P제어)
float Sp_kp_R = 0.1;      // 비례(P) 게인: 오차에 비례하여 출력
float Sp_ki_R = 0.000001;  // 적분(I) 게인: 극히 작아서 거의 영향 없음
float Sp_kd_R = 0.0000;    // 미분(D) 게인: 0이므로 미사용

float Sp_Theta_R, Sp_Theta_d_R; // (현재 미사용)

// PID 시간 관련 변수
int Sp_dT_R;                     // 루프 간 시간 간격 (ms)
unsigned long Sp_T_R;            // 현재 시각 (millis)
unsigned long Sp_T_prev_R = 0;   // 이전 루프 시각
int Sp_val_prev_R = 0;           // (현재 미사용)

// PID 오차/적분/미분 변수
float Sp_e_R;           // 현재 오차 (목표 - 실제)
float Sp_e_prev_R = 0;  // 이전 오차 (미분 계산용)
float Sp_inte_R;         // 적분 누적값 (사다리꼴 적분)
float Sp_inte_prev_R = 0; // 이전 적분값
float Sp_deri_R;         // 미분값

// PID 출력 제한
float Sp_Vmax_R = 24;   // PID 출력 상한 (24V 모터 기준)
float Sp_Vmin_R = -24;  // PID 출력 하한
float Sp_V_R = 0.1;     // PID 출력값 (초기값 0.1)

// DC모터 PWM 변수
int Sp_PWMval_R;      // PID 출력을 PWM(0~250)으로 변환한 값 (한 루프의 증분)
int Sp_PWM_F_R = 0;   // 최종 PWM 누적값 (매 루프마다 Sp_PWMval_R을 더함)
                       // → 한번에 목표로 점프하지 않고 부드럽게 가감속

// =====================================================================================
// RPM 측정 변수
// =====================================================================================
volatile long EncoderCount = 0;      // 엔코더 카운트 (회전당 약 280 count)
volatile long EncoderCount_prev = 0; // 이전 엔코더 카운트
unsigned long rpmtime;               // 현재 시각 (RPM 계산용)
unsigned long rpmtime_prev = 0;      // 이전 RPM 계산 시각
long E, R;    // E: 엔코더 변화량, R: 경과 시간(ms)
float rev;    // 초당 회전수 중간 계산값
float rps;    // 초당 회전수(Revolutions Per Second)

// =====================================================================================
// 조향 모터 PID 위치 제어 변수
// =====================================================================================
// 조향 모터는 포텐셔미터(A8핀)로 현재 각도를 읽고,
// PID 제어로 목표 각도까지 DC모터를 돌리는 방식

int encoder_pot = A8;  // 조향 포텐셔미터가 연결된 아날로그 핀
int val;               // 조향 목표값 (포텐셔미터 단위, 0~1023)
int encoder_val;       // 현재 조향 위치 (analogRead(A8)로 읽은 값)

// PID 게인값
// → P + D 제어 (I는 0). 조향은 위치 제어이므로 D(미분)로 오버슈트 억제
float kp = 0.1;     // 비례(P) 게인
float ki = 0.00000;  // 적분(I) 게인: 0이므로 미사용
float kd = 2.00;     // 미분(D) 게인: 급격한 움직임을 억제 (오버슈트 방지)

float Theta, Theta_d;   // Theta: 현재 각도, Theta_d: 목표 각도
int dt;                  // 루프 간 시간 간격 (ms)
unsigned long t;         // 현재 시각
unsigned long t_prev = 0; // 이전 시각
int val_prev = 0;        // 이전 목표값
float e, e_prev = 0;    // 현재/이전 오차
float inte, inte_prev = 0; // 현재/이전 적분값
float Vmax = 24;         // PID 출력 상한
float Vmin = -24;        // PID 출력 하한
float V = 0.1;           // PID 출력값
int PWMval;              // PID 출력을 PWM(0~255)으로 변환한 값


// =====================================================================================
// setup() : 전원 ON 시 1번만 실행되는 초기화 함수
// =====================================================================================
void setup() {
  // --- 시리얼 통신 초기화 ---
  Serial.begin(115200);    // USB 시리얼 (PC 시리얼 모니터로 디버깅 출력용)
  Serial2.begin(115200);   // 하드웨어 시리얼2 (PC/ROS와 데이터 통신용)
  Serial2.setTimeout(1);   // 읽기 타임아웃을 1ms로 설정 (기본 1초 → 제어 루프 지연 방지)

  // =========================================================================
  // [자동 계산 로직] 사용자 설정값을 기반으로 St_M, St_D 계산
  // =========================================================================
  St_M = VAL_NEUTRAL; // 사용자가 지정한 중립값 적용

  // 중립에서 좌측 한계까지의 거리와 우측 한계까지의 거리를 계산
  int range_left  = abs(VAL_MAX_LEFT - VAL_NEUTRAL);   // 예: |990 - 526| = 464
  int range_right = abs(VAL_NEUTRAL - VAL_MAX_RIGHT);  // 예: |526 - 20|  = 506

  // 좌/우 중 '더 짧은 거리'를 기준으로 안전 범위를 설정 (좌우 대칭 유지를 위해)
  // → 긴 쪽을 기준으로 하면 짧은 쪽에서 기계적 한계를 넘을 위험이 있음
  int min_physical_radius = (range_left < range_right) ? range_left : range_right;

  // 안전율 적용 → 최종 조향 범위: [St_M - St_D] ~ [St_M + St_D]
  St_D = min_physical_radius * SAFETY_RATE;
  // =========================================================================

  // --- 모터 핀 모드 설정 ---
  pinMode(RMS, OUTPUT);  // 구동 모터 속도 (PWM 출력)
  pinMode(RMD, OUTPUT);  // 구동 모터 방향 (디지털 출력)
  pinMode(LMS, OUTPUT);  // 조향 모터 속도 (PWM 출력)
  pinMode(LMD, OUTPUT);  // 조향 모터 방향 (디지털 출력)

  // --- 구동 모터 엔코더 핀 설정 ---
  // INPUT_PULLUP: 내부 풀업 저항을 활성화하여 외부 풀업 저항 없이도 안정적으로 신호 읽기
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);

  // attachInterrupt: 핀 상태가 변할 때마다 자동으로 ISR 함수를 호출
  // CHANGE: HIGH→LOW 또는 LOW→HIGH 둘 다 감지 → 엔코더 분해능 2배
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);

  // --- 모터 초기 상태: 정지 ---
  // 부팅 시 핀이 불확정 상태일 수 있으므로, 명시적으로 정지 상태로 설정
  analogWrite(RMS, 0);    // 구동 모터 속도 = 0
  digitalWrite(RMD, 0);   // 구동 모터 방향 = 전진(정지 상태)
  analogWrite(LMS, 0);    // 조향 모터 속도 = 0
  digitalWrite(LMD, 0);   // 조향 모터 방향 초기화

  // --- LCD 부팅 화면 ---
  lcd.begin(16, 2);       // 16칸 x 2줄 LCD 초기화
  lcd.setCursor(0, 0);    // 커서를 1행 1열로
  lcd.write("ROMOMO LAB");
  lcd.setCursor(0, 1);    // 커서를 2행 1열로
  lcd.write("HENES_T870");
  delay(1000);

  // 3-2-1 카운트다운 표시
  lcd.setCursor(0, 0);
  lcd.write("3               ");
  lcd.setCursor(0, 1);
  lcd.write("                ");
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.write("2");
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.write("1");
  delay(1000);
}

// =====================================================================================
// loop() : setup() 이후 무한 반복 실행되는 메인 루프
// =====================================================================================
void loop() {
  unsigned long loopstart = millis(); // 루프 시작 시각 기록 (루프 소요 시간 측정용)

  // --- 메인 제어 흐름 (순서대로 실행) ---
  controller();            // 1. RC 수신기에서 조종 데이터 읽기
  platform_control();      // 2. 모드(수동/자율)에 따라 최종 Speed, Steer, Gear 결정
  SteerCon(Steer);         // 3. 조향 모터 PID 위치 제어 실행
  SpeedCon_R(Speed, Gear); // 4. 구동 모터 PID 속도 제어 실행
  LCD();                   // 5. LCD 화면 업데이트
  Send();                  // 6. 현재 상태를 PC(ROS)로 전송

  int potpot = analogRead(A8); // 조향 포텐셔미터 값 읽기 (디버깅용)

  unsigned long loopend = millis(); // 루프 종료 시각

  // --- 시리얼 모니터 디버깅 출력 ---
  // 형식: throttle, steer, gear, Steer, encoder_val, RPM, 루프시간(ms)
  Serial.print(throttle); Serial.print(", ");
  Serial.print(steer); Serial.print(", ");
  Serial.print(gear); Serial.print(", ");
  Serial.print(Steer); Serial.print(", ");
  Serial.print(encoder_val); Serial.print(", ");
  Serial.print(RPM); Serial.print(", ");
  Serial.println(loopend - loopstart);

  delay(1); // 1ms 대기 (루프 안정화)

  // --- 구동 모터 엔코더 속도 계산 ---
  // 한 루프 동안 엔코더가 얼마나 변했는지 = 현재 속도
  E_R = EncoderCount_R - EncoderCount_prev_R;
  EncoderCount_prev_R = EncoderCount_R;

  i++; // 루프 카운터 증가 (수신기 채널 순환에 사용)
}

// =====================================================================================
// controller() : RC 수신기에서 채널 데이터를 읽고 가공하는 함수
// =====================================================================================
void controller(){
  // noInterrupts(): pulseIn() 실행 중 엔코더 인터럽트가 간섭하지 않도록 일시 차단
  noInterrupts();

  // i%4로 한 루프에 한 채널씩 번갈아 읽음 (pulseIn이 블로킹이라 루프 속도 보호)
  // pulseIn(핀, HIGH, 30000): 핀에서 HIGH 펄스의 폭을 μs로 측정, 최대 30ms 대기
  if(i%4 == 0){
    throttle = pulseIn(A0, HIGH, 30000); // 스로틀 읽기
    throttle = throttle / res * res;     // 20μs 단위로 반올림 (노이즈 제거)

    // 이동평균 필터: 과거 값을 한 칸씩 밀어넣기
    lsp5 = lsp4;
    lsp4 = lsp3;
    lsp3 = lsp2;
    lsp2 = lsp1;
  }
  else if(i%4 == 1){
    steer = pulseIn(A1, HIGH, 30000); // 조향 읽기
    steer = steer / res * res;
  }

  // elev, rudd 채널은 현재 사용하지 않음 (주석 처리)
  // else if(i%6 == 2){
  //   elev = pulseIn(A2,HIGH,30000);
  //   elev = elev/res*res;
  // }
  // else if(i%6 == 3){
  //   rudd = pulseIn(A3,HIGH,30000);
  //   rudd = rudd/res*res;
  // }

  else if(i%4 == 2){
    mode = pulseIn(A4, HIGH, 30000); // 모드 스위치 읽기
    mode = mode / res * res;
  }
  else if(i%4 == 3){
    gear = pulseIn(A5, HIGH, 30000); // 기어 스위치 읽기
    gear = gear / res * res;
  }

  // interrupts(): 인터럽트 다시 활성화 (엔코더 카운트 재개)
  interrupts();

  // --- throttle(스로틀) 신호 후처리 ---
  // 데드존 처리: 중립(1500) 근처 ±50은 정지로 간주
  if(throttle < 1550 and throttle > 1450){
    throttle = 1500;
    Les = 0;       // 비상정지 해제
  }
  else if(throttle < 1200){
    // 스로틀이 매우 낮음 → 비상정지 트리거
    throttle = throttle;
    Les = 1;       // 비상정지 활성화
  }
  else{
    throttle = throttle;
    Les = 0;
  }

  // 스로틀 값을 엔코더 속도 단위(0~Sp_MAX_R)로 매핑
  // map(값, 입력최소, 입력최대, 출력최소, 출력최대)
  lsp1 = map(throttle, 1550, 1850, 0, Sp_MAX_R);
  lsp1 = constrain(lsp1, 0, Sp_MAX_R); // 범위 초과 방지

  // 최근 5개 값의 이동평균 → 급격한 속도 변화를 부드럽게
  Lsp = (lsp1 + lsp2 + lsp3 + lsp4 + lsp5) / 5;

  // --- steer(조향) 신호 후처리 ---
  // 데드존 처리: 중립(1460) 근처 ±50은 직진으로 간주
  if(steer < 1510 and steer > 1410){
    steer = 1460;
  }
  else{
    steer = steer;
  }

  // 수신기 조향값(μs)을 포텐셔미터 단위로 매핑
  // [수정됨] 자동 계산된 St_M, St_D를 사용하여 매핑
  Lst = map(steer, 1250, 1670, St_M - St_D, St_M + St_D);
  Lst = constrain(Lst, St_M - St_D, St_M + St_D); // 안전 범위 내로 제한

  // --- gear(기어) 신호 후처리 ---
  if(gear > 1600){
    Lge = 0; // 전진(D)
  }
  else if(gear <= 1600 and gear > 1400){
    Lge = 1; // 중립(N)
  }
  else{
    Lge = 2; // 후진(R)
  }

  // --- mode(모드) 신호 후처리 ---
  if(mode < 1250){
    AorM = 1; // 자율주행 모드
  }
  else if(mode > 1251 and mode < 1650){
    AorM = 0; // 수동주행 모드
  }
  else{
    AorM = 2; // 디버깅 모드
  }
}

// =====================================================================================
// 통신 타임아웃(Watchdog) 관련 변수
// =====================================================================================
// [추가] 통신 타임아웃 관련 변수 (전역 변수로 선언 필요)
unsigned long Last_Rx_Time = 0;  // 마지막으로 유효한 패킷을 받은 시간
const unsigned long Rx_Timeout = 500; // 0.5초(500ms) 동안 신호 없으면 비상 정지

// =====================================================================================
// platform_control() : 모드에 따라 최종 Speed, Steer, Gear을 결정하는 함수
// =====================================================================================
void platform_control() {
  if (AorM == 0) { // ========== 수동주행 모드 ==========
    // RC 수신기(Low-level) 명령을 그대로 사용
    Speed = Lsp;
    Steer = Lst;
    Gear = Lge;
    Hes = 0;       // 상위 비상정지 해제
  }
  else if (AorM == 1) { // ========== 자율주행 모드 ==========
    bool packet_ok = false;

    // ====== HLC(PC/ROS) → LLC(아두이노) 13바이트 패킷 파싱 ======
    // Serial2 버퍼에 수신된 바이트를 하나씩 읽어서 패킷을 조립
    while (Serial2.available()) {
      byte b = Serial2.read();

      // 첫 바이트가 'S'가 아니면 무시 (패킷 시작점 동기화)
      if (rxIdx == 0 && b != 'S') continue;

      rxBuf[rxIdx++] = b; // 바이트를 버퍼에 저장하고 인덱스 증가

      if (rxIdx == 13) { // 13바이트가 모이면 패킷 검증
        // 헤더(S,T,X)와 푸터(0x0D=CR, 0x0A=LF) 확인
        if (rxBuf[0] == 'S' && rxBuf[1] == 'T' && rxBuf[2] == 'X' &&
            rxBuf[11] == 0x0D && rxBuf[12] == 0x0A) {
          packet_ok = true; // 유효한 패킷!
        } else {
          rxIdx = 0; // 패킷이 깨졌으면 처음부터 다시 조립
        }
        break; // 한 루프에 패킷 하나만 처리 (제어 루프 속도 보호)
      }
    }

    // --- 유효한 패킷이 도착했을 때 ---
    if (packet_ok) {
      rxIdx = 0;                  // 다음 패킷을 위해 인덱스 초기화
      Last_Rx_Time = millis();    // ★ 수신 시간 갱신 (Watchdog용)

      // 패킷 데이터 파싱
      Ham = rxBuf[3];                     // 자율주행 활성화 여부 (1=활성)
      Hes = rxBuf[4];                     // 비상정지
      Hge = rxBuf[5];                     // 기어
      Hsp = word(rxBuf[6], rxBuf[7]);     // 속도 (2바이트 → 16비트 정수)
      Hst = word(rxBuf[8], rxBuf[9]);     // 조향 (2바이트 → 16비트 정수)
      Hal = rxBuf[10];                    // Alive 카운터

      // 명령값을 실제 제어 범위로 매핑
      Hsp = map(Hsp, 0, 1000, 0, 15);                        // 속도: 0~1000 → 0~15
      Hst = map(Hst, -2000, 2000, St_M + St_D, St_M - St_D); // 조향: -2000~2000 → 포텐셔미터 범위
    }

    // --- 안전 장치 (Watchdog): 통신이 살아있는지 시간으로 체크 ---
    if (millis() - Last_Rx_Time < Rx_Timeout) {
      // [통신 정상] 0.5초 이내에 패킷을 받은 적 있음
      Con_status = 1;

      if (Ham == 1) { // 자율주행 활성화 상태일 때만 명령 적용
        Speed = Hsp;
        Steer = Hst;
        Gear = Hge;
      } else {
        // 자율주행 비활성화 → 정지
        Speed = 0;
        Steer = St_M; // 직진 위치
        Gear = 0;
      }
    }
    else {
      // [통신 끊김] 0.5초 이상 유효 데이터 없음 → 비상 정지
      Con_status = 0;
      Speed = 0;
      Steer = St_M; // 직진 위치로 복귀
      Gear = 1;     // 중립
    }
  }
  else { // ========== 디버깅 모드 ==========
    // 모든 출력 정지 (센서값 확인 전용)
    Speed = 0;
    Steer = St_M;
    Gear = 1;
  }

  // --- 공통 안전 로직 ---
  // 하위제어기(Les: 수신기)나 상위제어기(Hes: PC) 중 하나라도 비상정지면 즉시 멈춤
  if (Les == 1 || Hes == 1) {
    ESTOP = 1;
  } else {
    ESTOP = 0;
  }

  // 최종 출력 제한 (어떤 경우에도 안전 범위를 넘지 않도록)
  Steer = constrain(Steer, St_M - St_D, St_M + St_D);
  Speed = constrain(Speed, 0, 250);

  // 비상정지 상태이면 무조건 정지
  if (ESTOP == 1) {
    Steer = St_M; // 직진
    Speed = 0;    // 정지
  }
}

// =====================================================================================
// SteerCon() : 조향 모터 PID 위치 제어 함수
//   Q = 조향 목표값 (포텐셔미터 단위)
//   DC모터 + 포텐셔미터 조합으로 서보처럼 동작하게 만든 PID 제어
// =====================================================================================
void SteerCon(int Q) {
  val = Q; // 목표 조향 위치

  // [안전] 목표값이 허용 범위를 벗어나면 강제로 제한
  if(val > St_M + St_D){
    val = St_M + St_D;
  }
  else if(val < St_M - St_D){
    val = St_M - St_D;
  }

  // --- 현재 상태 읽기 ---
  encoder_val = analogRead(A8); // 포텐셔미터에서 현재 조향 각도 읽기 (0~1023)
  t = millis();                 // 현재 시각
  dt = (t - t_prev);           // 이전 루프로부터 경과 시간 (ms)

  // --- PID 계산 ---
  Theta = encoder_val; // 현재 위치 (Actual)
  Theta_d = val;       // 목표 위치 (Desired)
  e = Theta_d - Theta; // 오차 = 목표 - 현재 (양수: 더 꺾어야 함, 음수: 돌아와야 함)

  // 적분(I): 사다리꼴 적분법으로 오차를 시간에 대해 누적
  inte = inte_prev + (dt * (e + e_prev) / 2);

  // PID 제어 출력 = P항 + I항 + D항
  V = kp * e + ki * inte + (kd * (e - e_prev) / dt);

  // --- 출력 제한 (Anti-Windup) ---
  // PID 출력이 상한/하한을 넘으면 값을 클램핑하고,
  // 적분값도 이전 값으로 되돌려서 적분이 계속 쌓이는 것(Windup)을 방지
  if (V > Vmax) {
    V = Vmax;
    inte = inte_prev;
  }
  if (V < Vmin) {
    V = Vmin;
    inte = inte_prev;
    val_prev = val;
  }

  // --- PID 출력 → PWM 변환 ---
  // 출력(V)의 절대값을 0~255 PWM 범위로 변환
  PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 150) {
    PWMval = 150; // 조향 모터 보호: PWM 최대 150으로 제한 (과부하 방지)
  }

  // --- 모터 구동 ---
  if (V > 0.5) {
    // 양의 출력 → 한 방향으로 회전 (예: 좌회전 방향)
    analogWrite(LMS, PWMval);  // 속도 설정
    digitalWrite(LMD, 1);      // 방향 = 1
  }
  else if (V < -0.5) {
    // 음의 출력 → 반대 방향으로 회전 (예: 우회전 방향)
    analogWrite(LMS, PWMval);  // 속도 설정
    digitalWrite(LMD, 0);      // 방향 = 0
  }
  else {
    // 오차가 거의 없음 (±0.5 이내) → 모터 정지 (데드존)
    digitalWrite(LMD, 0);
    analogWrite(LMS, 0);
  }

  // --- 다음 루프를 위해 현재 값을 이전 값으로 저장 ---
  t_prev = t;
  inte_prev = inte;
  e_prev = e;
}


// =====================================================================================
// SpeedCon_R() : 구동 모터 PID 속도 제어 함수
//   Sp_R = 목표 속도, g = 기어 (0=전진, 1=중립, 2=후진)
// =====================================================================================
void SpeedCon_R(int Sp_R, int g) {
  // --- 기어에 따라 목표 속도 방향 설정 ---
  if(g == 0){
    Sp_val_R = Sp_R;       // 전진: 양의 속도
  }
  else if(g == 2){
    Sp_val_R = Sp_R * -1;  // 후진: 음의 속도
  }
  else{
    Sp_val_R = 0;           // 중립: 정지
  }

  // --- 목표 속도 제한 ---
  if(Sp_val_R > Sp_MAX_R){
    Sp_val_R = Sp_MAX_R;    // 상한 클램핑 (15)
  }
  else if(Sp_val_R < Sp_MIN_R){
    Sp_val_R = Sp_MIN_R;    // 하한 클램핑 (-15)
  }
  else{
    Sp_val_R = Sp_val_R;    // 범위 내면 그대로
  }

  // --- 현재 속도 읽기 및 PID 계산 ---
  Sp_encoder_val_R = E_R;   // 현재 속도 = 한 루프 동안의 엔코더 변화량
  Sp_T_R = millis();
  Sp_dT_R = (Sp_T_R - Sp_T_prev_R); // 경과 시간 (ms)

  Sp_e_R = Sp_val_R - Sp_encoder_val_R;   // 오차 = 목표 속도 - 현재 속도

  // 적분(I): 사다리꼴 적분법
  Sp_inte_R = Sp_inte_prev_R + (Sp_dT_R * (Sp_e_R + Sp_e_prev_R) / 2);

  // 미분(D): 오차 변화율
  Sp_deri_R = (Sp_e_R - Sp_e_prev_R) / Sp_dT_R;

  // PID 제어 출력 = P + I + D
  Sp_V_R = Sp_kp_R * Sp_e_R + Sp_ki_R * Sp_inte_R + Sp_kd_R * Sp_deri_R;

  // --- 출력 제한 (Anti-Windup) ---
  if (Sp_V_R > Sp_Vmax_R) {
    Sp_V_R = Sp_Vmax_R;
    Sp_inte_R = Sp_inte_prev_R; // 적분 누적 방지
  }
  else if (Sp_V_R < Sp_Vmin_R) {
    Sp_V_R = Sp_Vmin_R;
    Sp_inte_R = Sp_inte_prev_R;
  }
  else{
    Sp_V_R = Sp_V_R;
    Sp_inte_R = Sp_inte_prev_R;
  }

  // --- PID 출력 → PWM 변환 (누적 방식) ---
  // PID 출력 비율을 0~250 PWM 범위로 변환
  Sp_PWMval_R = int(250 * Sp_V_R / Sp_Vmax_R);

  // 누적: 매 루프마다 조금씩 더해서 부드러운 가감속 실현
  Sp_PWM_F_R = Sp_PWM_F_R + Sp_PWMval_R;

  // PWM 누적값 제한 (±250)
  if(Sp_PWM_F_R > 250){
    Sp_PWM_F_R = 250;
  }
  else if(Sp_PWM_F_R < -250){
    Sp_PWM_F_R = -250;
  }
  else{
    Sp_PWM_F_R = Sp_PWM_F_R;
  }

  // --- 모터 구동 ---
  if(Sp_PWM_F_R > 1){
    // 양의 PWM → 전진
    analogWrite(RMS, Sp_PWM_F_R);  // 속도 설정
    digitalWrite(RMD, 0);           // 방향 = 전진
  }
  else if(Sp_PWM_F_R < 0){
    // 음의 PWM → 후진 (PWM은 절대값으로 변환)
    analogWrite(RMS, -1 * Sp_PWM_F_R);
    digitalWrite(RMD, 1);           // 방향 = 후진
  }
  else{
    // PWM ≈ 0 → 정지
    analogWrite(RMS, 0);
    digitalWrite(RMD, 0);
  }

  // --- 다음 루프를 위해 현재 값을 이전 값으로 저장 ---
  Sp_T_prev_R = Sp_T_R;
  Sp_inte_prev_R = Sp_inte_R;
  Sp_e_prev_R = Sp_e_R;
}

// =====================================================================================
// LCD() : LCD 화면에 현재 상태를 표시하는 함수
// =====================================================================================
void LCD(){
  if(AorM == 0){ // ===== 수동 모드 화면 =====
    // 1행: 모드 표시
    lcd.setCursor(0, 0);
    lcd.print("Manual Mode     ");

    // 2행: 기어 + 비상정지 상태
    lcd.setCursor(0, 1);
    lcd.print("Gear:");
    lcd.setCursor(5, 1);
    if(Lge == 0){
      lcd.print("D ");  // Drive (전진)
    }
    else if(Lge == 1){
      lcd.print("N ");  // Neutral (중립)
    }
    else{
      lcd.print("R ");  // Reverse (후진)
    }

    // ESTOP 상태 표시
    lcd.setCursor(7, 1);
    lcd.print("ESTOP:");
    lcd.setCursor(13, 1);
    if(Les == 1 and Hes == 0){
      lcd.print("R  ");  // R = RC(수신기)에서 비상정지
    }
    else if(Les == 0 and Hes == 1){
      lcd.print("H  ");  // H = HLC(상위제어기)에서 비상정지
    }
    else if(Les == 1 and Hes == 1){
      lcd.print("A  ");  // A = All (둘 다 비상정지)
    }
    else{
      lcd.print("X  ");  // X = 비상정지 없음 (정상)
    }
  }
  else if(AorM == 2){ // ===== 디버깅 모드 화면 =====
    // 수신기 원시값을 직접 표시 (캘리브레이션/문제 진단용)
    lcd.setCursor(0, 0);
    lcd.print("Debugging Mode  ");
    lcd.setCursor(0, 1);
    lcd.print("thr:");
    lcd.setCursor(4, 1);
    lcd.print(throttle);       // 스로틀 원시값
    lcd.setCursor(8, 1);
    lcd.print("str:");
    lcd.setCursor(12, 1);
    lcd.print(steer);          // 조향 원시값
  }
  else{ // ===== 자율주행 모드 화면 =====
    lcd.setCursor(0, 0);
    lcd.print("Auto Mode       ");

    // 통신 상태 표시
    lcd.setCursor(0, 1);
    lcd.print("Con:");
    if(Con_status == 1){
      lcd.setCursor(4, 1);
      lcd.print("O  ");  // O = 통신 정상
    }
    else{
      lcd.setCursor(4, 1);
      lcd.print("X  ");  // X = 통신 끊김
    }

    // ESTOP 상태 표시 (수동 모드와 동일)
    lcd.setCursor(7, 1);
    lcd.print("ESTOP:");
    lcd.setCursor(13, 1);
    if(Les == 1 and Hes == 0){
      lcd.print("R  ");
    }
    else if(Les == 0 and Hes == 1){
      lcd.print("H  ");
    }
    else if(Les == 1 and Hes == 1){
      lcd.print("A  ");
    }
    else{
      lcd.print("X  ");
    }
  }
}

// =====================================================================================
// Send() : 아두이노 → PC(ROS)로 현재 상태를 13바이트 패킷으로 전송하는 함수
// 패킷 구조: [S][T][X][AorM][Les][Lge][RPM_H][RPM_L][StFeed_H][StFeed_L][Lal][0x0D][0x0A]
// =====================================================================================
void Send(){
  // --- 조향 피드백값 계산 ---
  // 현재 포텐셔미터 값(encoder_val)을 -2000~2000 범위로 변환하여 PC에 전달
  // → PC 쪽에서 조향 명령(-2000~2000)과 동일한 단위로 비교 가능
  Steer_Feed = map(encoder_val, St_M + St_D, St_M - St_D, -2000, 2000);
  Steer_Feed = constrain(Steer_Feed, -2000, 2000);

  // --- RPM 계산 ---
  rpmtime = millis();
  E = E_R;                          // 한 루프 동안의 엔코더 변화량
  R = (rpmtime - rpmtime_prev);     // 경과 시간 (ms)
  rev = (1000000.00 * E) / 280.00;  // 엔코더 카운트 → 마이크로 단위 보정 (280 count/회전)
  rps = rev / R;                    // 초당 회전수 (Revolutions Per Second)
  RPM = 6 * rps;                    // RPS → RPM 변환 (× 60 / 10 스케일링)

  rpmtime_prev = rpmtime;

  // --- 송신 패킷 조립 ---
  Senddata[0] = 'S';                         // 헤더 1
  Senddata[1] = 'T';                         // 헤더 2
  Senddata[2] = 'X';                         // 헤더 3
  Senddata[3] = AorM | 0x00;                 // 현재 모드 (0=수동, 1=자율, 2=디버그)
  Senddata[4] = Les | 0x00;                  // 비상정지 상태
  Senddata[5] = Lge | 0x00;                  // 기어 상태
  Senddata[6] = (RPM >> 8) | 0x00;           // RPM 상위 바이트 (16비트를 2바이트로 분리)
  Senddata[7] = RPM | 0x00;                  // RPM 하위 바이트
  Senddata[8] = (Steer_Feed >> 8) | 0x00;    // 조향 피드백 상위 바이트
  Senddata[9] = Steer_Feed | 0x00;           // 조향 피드백 하위 바이트
  Senddata[10] = Lal | 0x00;                 // Alive 카운터 (매 전송마다 +1)
  Senddata[11] = 0x0D;                       // 푸터: CR (Carriage Return)
  Senddata[12] = 0x0A;                       // 푸터: LF (Line Feed)

  Serial2.write(Senddata, 13); // 13바이트 패킷을 Serial2로 전송
  Lal = Lal + 1;               // Alive 카운터 증가 (byte이므로 0~255 순환)
}

// =====================================================================================
// ISR_EncoderA() : 구동 모터 엔코더 A채널 인터럽트 서비스 루틴
// =====================================================================================
// 엔코더 A핀의 상태가 변할 때(CHANGE) 자동 호출됨
// A, B 두 채널의 조합으로 회전 방향을 판별하여 카운트 증감
// (쿼드러처 엔코더 디코딩)
void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount_R++;  // 정방향 회전
    }
    else {
      EncoderCount_R--;  // 역방향 회전
    }
  }
  else {
    if (PinA == HIGH) {
      EncoderCount_R--;
    }
    else {
      EncoderCount_R++;
    }
  }
}

// =====================================================================================
// ISR_EncoderB() : 구동 모터 엔코더 B채널 인터럽트 서비스 루틴
// =====================================================================================
// 엔코더 B핀의 상태가 변할 때(CHANGE) 자동 호출됨
// A채널과 함께 사용하여 4체배(x4) 분해능 달성
void ISR_EncoderB() {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount_R--;
    }
    else {
      EncoderCount_R++;
    }
  }
  else {
    if (PinB == HIGH) {
      EncoderCount_R++;
    }
    else {
      EncoderCount_R--;
    }
  }
}