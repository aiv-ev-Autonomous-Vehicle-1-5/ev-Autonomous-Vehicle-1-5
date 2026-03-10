#include <LiquidCrystal.h>

// ===================================================================================
// [USER CONFIGURATION] 플랫폼별 측정값을 여기서 수정하세요
// ===================================================================================
// 1. 아두이노 전원만 켜고 손으로 바퀴를 끝까지 돌려 측정한 센서값(encoder_val)
// 세종대 : 1010, 20, 520
// 충남대 : 1000, 35, 510
// 청주대 : 960, 20. 518
const int VAL_MAX_LEFT  = 990;   // 좌회전 물리적 한계값
const int VAL_MAX_RIGHT = 20;    // 우회전 물리적 한계값

// 2. 바퀴를 일자로 정렬했을 때의 센서값 (중립값)
const int VAL_NEUTRAL   = 526;   // 직접 보고 맞춘 직진(중립) 값

// 3. 기구 보호를 위한 안전율 (0.97 = 97% 사용)
const float SAFETY_RATE = 0.97;  
// ===================================================================================

// 자동 계산 변수 (수정하지 마세요)
int St_M; // 설정된 중립값
int St_D; // 계산된 안전 작동 범위 (좌우 대칭)


// lcd 화면 관련 변수
LiquidCrystal lcd(30,31,32,33,34,35);//RS,E,DB4,DB5,DB6,DB7

// 수신기 관련 변수
int steer , throttle , elev , rudd ,mode, gear; // 수신기 데이터
int res = 20; // 수신기 데이터의 안정화를 위한 상수
unsigned long i = 0; // 루프 타임을 줄이기 위해 수신기 데이터를 차례로 받도록 해주는 변수

// 플랫폼 제어 관련 변수
int Speed; // 플랫폼 속도 명령값
int Steer; // 플랫폼 조향 명령값

int Max_Speed = 250; // 플랫폼의 최대 속도값 (0~255)
byte Gear; // 플랫폼 기어 명령값
byte AorM; // 플랫폼 자율주행 모드, 수동주행 모드, 디버깅 모드 명령값
byte ESTOP; // 플랫폼 긴급정지 명령값
int lsp1,lsp2,lsp3,lsp4,lsp5 = 0;// 모터 속도 이동 평균 제어
int Lsp,Lst,Hsp,Hst,Hsp_p,Hst_p; // 상위 및 하위 제어의 속도, 조향 명령값
byte Lge,Les,Ham,Hge,Hes,Hal,Ham_p,Hge_p,Hes_p,Hal_p; // 상위 및 하위 제어의 기어, 모드, 긴급정지 명령값
byte Lal = 0;
int Steer_Feed,RPM;
char Receivedata[13]; // S T X Ham Hes Hge Hsp Hst Hal 0D0A
char Senddata[13]; // S T X AorM Les Lge RPM St_feed Lal 0D0A
int Con_status = 0;
byte rxBuf[13];
uint8_t rxIdx = 0;


// 조향 모터 (Steering) L/R 이 left,right가 아님. 변수명 이상하게 잘못지음
const int LMS = 5;    // 조향 모터 Speed (PWM)
const int LMD = 24;   // 조향 모터 Direction
// 구동 모터 (Drive)
const int RMS = 4;    // 구동 모터 Speed (PWM)
const int RMD = 22;   // 구동 모터 Direction

//구동모터의 엔코더센서 핀번호    
  const int interruptPinA = 3;  
  const int interruptPinB = 2;
  volatile long EncoderCount_R= 0; 
  long EncoderCount_prev_R = 0;
   //PID 변수_R
  int E_R = 0;
  int Sp_MAX_R = 15;
  int Sp_MIN_R = -15;
  int Sp_encoder_pot_R;    
  int Sp_val_R; 
  int Sp_encoder_val_R;
  float Sp_kp_R = 0.1;
  float Sp_ki_R = 0.000001 ;
  float Sp_kd_R = 0.0000;
  float Sp_Theta_R, Sp_Theta_d_R;
  int Sp_dT_R;
  unsigned long Sp_T_R;
  unsigned long Sp_T_prev_R = 0;
  int Sp_val_prev_R =0;
  float Sp_e_R, Sp_e_prev_R = 0, Sp_inte_R, Sp_inte_prev_R = 0, Sp_deri_R;
  float Sp_Vmax_R = 24;
  float Sp_Vmin_R = -24;
  float Sp_V_R = 0.1;
  //DC모터 PWM변수  
  int Sp_PWMval_R;
  int Sp_PWM_F_R = 0;
//RPM측정을 위한 변수들:NO
  volatile long EncoderCount = 0; // 엔코더 값 변수 회전당 280 count
  volatile long EncoderCount_prev = 0;
  unsigned long rpmtime;
  unsigned long rpmtime_prev = 0;
  long E,R;                 
  float rev, rps;

//조향모터 가변저항 및 조향모터의 PID제어를 위한 변수 
  int encoder_pot = A8;    
  int val; 
  int encoder_val;
  float kp = 0.1;
  float ki = 0.00000 ;
  float kd = 2.00;
  float Theta, Theta_d;
  int dt;
  unsigned long t;
  unsigned long t_prev = 0;
  int val_prev =0;
  float e, e_prev = 0, inte, inte_prev = 0;
  float Vmax = 24;
  float Vmin = -24;
  float V = 0.1;
  int PWMval;


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial2.setTimeout(1);

  // =========================================================================
  // [자동 계산 로직] 사용자 설정값을 기반으로 St_M, St_D 계산
  // =========================================================================
  St_M = VAL_NEUTRAL; // 사용자가 지정한 중립값 적용

  // 중립에서 좌측 한계까지의 거리와 우측 한계까지의 거리를 계산
  int range_left  = abs(VAL_MAX_LEFT - VAL_NEUTRAL);
  int range_right = abs(VAL_NEUTRAL - VAL_MAX_RIGHT);

  // 좌/우 중 '더 짧은 거리'를 기준으로 안전 범위를 설정 (좌우 대칭 유지를 위해)
  int min_physical_radius = (range_left < range_right) ? range_left : range_right;
  
  // 안전율 적용
  St_D = min_physical_radius * SAFETY_RATE;     
  // =========================================================================

  pinMode(RMS,OUTPUT);
  pinMode(RMD,OUTPUT);
  pinMode(LMS,OUTPUT);
  pinMode(LMD,OUTPUT);
  
  pinMode(interruptPinA, INPUT_PULLUP); // 엔코더 핀 
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE); // 엔코더 값을 읽기 위한 인터럽트 서비스 루틴 
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);

  analogWrite(RMS,0);
  digitalWrite(RMD,0);
  analogWrite(LMS,0);
  digitalWrite(LMD,0);

  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.write("ROMOMO LAB");
  lcd.setCursor(0,1);
  lcd.write("HENES_T870");
  delay(1000);
  lcd.setCursor(0,0);
  lcd.write("3               ");
  lcd.setCursor(0,1);  
  lcd.write("                ");
  delay(1000);
  lcd.setCursor(0,0);
  lcd.write("2");
  delay(1000);
  lcd.setCursor(0,0);
  lcd.write("1");
  delay(1000);
}

void loop() {
unsigned long loopstart = millis();
controller();
platform_control();
SteerCon(Steer);
SpeedCon_R(Speed,Gear);
LCD();
Send();
int potpot = analogRead(A8);
unsigned long loopend = millis();

// Serial.print(throttle); Serial.print(", ");
// Serial.print(steer); Serial.print(", ");
// Serial.print(elev); Serial.print(", ");
// Serial.print(rudd); Serial.print(", ");
// Serial.print(mode); Serial.print(", ");
// Serial.print(gear); Serial.print(", ");
// Serial.println(loopend-loopstart);

Serial.print(throttle); Serial.print(", ");
Serial.print(steer); Serial.print(", ");
Serial.print(gear); Serial.print(", ");
Serial.print(Steer); Serial.print(", ");
Serial.print(encoder_val); Serial.print(", ");
Serial.print(RPM); Serial.print(", "); 
Serial.println(loopend-loopstart);
delay(1);
E_R = EncoderCount_R-EncoderCount_prev_R;
EncoderCount_prev_R = EncoderCount_R;
i++;
}

void controller(){ // 수신기 데이터를 받는 함수
  noInterrupts();
  if(i%4 == 0){
    throttle = pulseIn(A0,HIGH,30000); // throttle
    throttle = throttle/res*res;
    lsp5 = lsp4;
    lsp4 = lsp3;
    lsp3 = lsp2;
    lsp2 = lsp1;
  }
  else if(i%4 == 1){
    steer = pulseIn(A1,HIGH,30000); // steer 
    steer = steer/res*res;
  }
  
  // else if(i%6 == 2){   //gear ->A5
  //   elev = pulseIn(A2,HIGH,30000); // elevation
  //   elev = elev/res*res;
  // }
  // else if(i%6 == 3){
  //   rudd = pulseIn(A3,HIGH,30000); // rudder
  //   rudd = rudd/res*res;
  // }
  
  else if(i%4 == 2){
    mode = pulseIn(A4,HIGH,30000); // Debugging Manual Auto
    mode = mode/res*res;
  }
  else if(i%4 == 3){ //2
    gear = pulseIn(A5,HIGH,30000); // Drive Neutral Reverse
    gear = gear/res*res;
  }
  interrupts();
  // throttle 신호 처리
  if(throttle < 1550 and throttle > 1450){ // throttle 신호의 DEADZONE
    throttle = 1500;
    Les = 0;
  }
  else if(throttle < 1200){
    throttle = throttle;
    Les = 1;
  }
  else{
    throttle = throttle;
    Les = 0;
  }
  lsp1 = map(throttle,1550,1850,0,Sp_MAX_R);
  lsp1 = constrain(lsp1,0,Sp_MAX_R);
  Lsp = (lsp1+lsp2+lsp3+lsp4+lsp5)/5;

  // steer 신호 처리
  if(steer < 1510 and steer > 1410){ // steer 신호의 DEADZONE
    steer = 1460;
  }
  else{
    steer = steer;
  }

  // [수정됨] 자동 계산된 St_M, St_D를 사용하여 매핑
  Lst = map(steer, 1250, 1670, St_M - St_D, St_M + St_D); 
  
  Lst = constrain(Lst, St_M - St_D, St_M + St_D);

  // gear 신호 처리
  if(gear > 1600){
    Lge = 0; //전진
  }
  else if(gear <= 1600 and gear > 1400){
    Lge = 1; //중립
  }
  else{
    Lge = 2; //후진
  }
  // mode 신호 처리
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

// [추가] 통신 타임아웃 관련 변수 (전역 변수로 선언 필요)
unsigned long Last_Rx_Time = 0;  // 마지막으로 패킷을 받은 시간
const unsigned long Rx_Timeout = 500; // 0.5초(500ms) 동안 신호 없으면 정지

void platform_control() { 
  if (AorM == 0) { // 수동주행
    Speed = Lsp;
    Steer = Lst;
    Gear = Lge;
    Hes = 0;
  }
  else if (AorM == 1) { // 자율주행
    bool packet_ok = false;

    // ====== HLC → LLC 13바이트 패킷 파싱 ======
    while (Serial2.available()) {
      byte b = Serial2.read();

      // STX 동기화
      if (rxIdx == 0 && b != 'S') continue;

      rxBuf[rxIdx++] = b;

      if (rxIdx == 13) { // 패킷 길이 충족
        // 헤더(S,T,X)와 푸터(0x0D, 0x0A) 검증
        if (rxBuf[0] == 'S' && rxBuf[1] == 'T' && rxBuf[2] == 'X' &&
            rxBuf[11] == 0x0D && rxBuf[12] == 0x0A) {
          packet_ok = true;
        } else {
          rxIdx = 0; // 패킷 깨짐, 초기화
        }
        break; // 한 루프에 패킷 하나만 처리
      }
    }

    // 1. 유효한 패킷이 들어왔을 때 -> 데이터를 갱신하고 시간을 기록
    if (packet_ok) {
      rxIdx = 0; // 다음 패킷을 위해 인덱스 초기화
      Last_Rx_Time = millis(); // ★ 중요: 수신 시간 갱신

      // 데이터 파싱
      Ham = rxBuf[3];
      Hes = rxBuf[4];
      Hge = rxBuf[5];
      Hsp = word(rxBuf[6], rxBuf[7]); // SPEED
      Hst = word(rxBuf[8], rxBuf[9]); // STEER
      Hal = rxBuf[10];                // ALIVE

      // 명령값 매핑
      Hsp = map(Hsp, 0, 1000, 0, 15);
      Hst = map(Hst, -2000, 2000, St_M + St_D, St_M - St_D);
    }

    // 2. 안전 장치 (Watchdog): 통신이 살아있는지 시간으로 체크
    if (millis() - Last_Rx_Time < Rx_Timeout) {
      // [통신 정상]
      Con_status = 1;
      
      if (Ham == 1) { // 자율주행 활성화 상태면
        Speed = Hsp;  // 마지막으로 받은 속도 유지
        Steer = Hst;  // 마지막으로 받은 조향 유지
        Gear = Hge;
      } else {
        Speed = 0;
        Steer = St_M;
        Gear = 0;
      }
    } 
    else {
      // [통신 끊김] 0.5초 이상 데이터가 안 옴 -> 비상 정지
      Con_status = 0;
      Speed = 0;
      Steer = St_M;
      Gear = 1; // 중립
    }
  }
  else { // 디버그 모드
    Speed = 0;
    Steer = St_M;
    Gear = 1;
  }

  // --- 공통 안전 로직 ---
  // 하위제어기(Les)나 상위제어기(Hes) 중 하나라도 비상정지면 멈춤
  if (Les == 1 || Hes == 1) {
    ESTOP = 1;
  } else {
    ESTOP = 0;
  }

  // 최종 출력 제한
  Steer = constrain(Steer, St_M - St_D, St_M + St_D);
  Speed = constrain(Speed, 0, 250);

  if (ESTOP == 1) {
    Steer = St_M;
    Speed = 0;
  }
}

void SteerCon(int Q) { // 조향모터의 제어를 위한 함수, Q = 조향 목표값  
  val = Q; //Q = steer 조향값 범위
  
  // [수정됨] 자동 계산된 안전 범위를 벗어날 경우 강제 제한
  if(val > St_M + St_D){ 
    val = St_M + St_D;
  }
  else if(val < St_M - St_D){ 
    val = St_M - St_D;
  }

  encoder_val =analogRead(A8);               // Read V_out from Feedback Pot
  t = millis();
  dt = (t - t_prev);                      // Time step
  Theta = encoder_val;                                // Theta= Actual Angular Position of the Motor
  Theta_d = val;                        // Theta_d= Desired Angular Position of the Motor 조종기값
  e = Theta_d - Theta;                        // Error
  inte = inte_prev + (dt * (e + e_prev) / 2);         // Integration of Error
  V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ; // Controlling Function
  if (V > Vmax) {
    V = Vmax;
    inte = inte_prev;
  }
  if (V < Vmin) {
    V = Vmin;
    inte = inte_prev;
    val_prev= val;
  }
  PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 150) {
    PWMval = 150;
  }
  if (V > 0.5) {
    analogWrite(LMS, PWMval);
    digitalWrite(LMD, 1);
  }
  else if (V < -0.5) {
    analogWrite(LMS, PWMval);
    digitalWrite(LMD, 0);
  }
  else {
    digitalWrite(LMD, 0);
    analogWrite(LMS, 0);
  }
   t_prev = t;
   inte_prev = inte;
   e_prev = e;

}


void SpeedCon_R(int Sp_R, int g) { 
  if(g==0){
  Sp_val_R = Sp_R;
  }
  else if(g ==2){
    Sp_val_R = Sp_R*-1;
  }
  else{
    Sp_val_R = 0;
  }
  if(Sp_val_R>Sp_MAX_R){  
    Sp_val_R = Sp_MAX_R;
  }
  else if(Sp_val_R<Sp_MIN_R){ 
    Sp_val_R = Sp_MIN_R;
  }
  else{
    Sp_val_R = Sp_val_R;
  }

  Sp_encoder_val_R = E_R;          
  Sp_T_R = millis();
  Sp_dT_R = (Sp_T_R - Sp_T_prev_R);                          
  Sp_e_R = Sp_val_R - Sp_encoder_val_R;                                // Error
  Sp_inte_R = Sp_inte_prev_R + (Sp_dT_R * (Sp_e_R + Sp_e_prev_R) / 2);         // Integration of Error
  Sp_deri_R = (Sp_e_R - Sp_e_prev_R)/Sp_dT_R; // Derivation of Error
  Sp_V_R = Sp_kp_R * Sp_e_R + Sp_ki_R * Sp_inte_R + Sp_kd_R * Sp_deri_R ; // Controlling Function

  if (Sp_V_R > Sp_Vmax_R) {
    Sp_V_R = Sp_Vmax_R;
    Sp_inte_R = Sp_inte_prev_R;
  }
  else if (Sp_V_R < Sp_Vmin_R) {
    Sp_V_R = Sp_Vmin_R;
    Sp_inte_R = Sp_inte_prev_R;
  } 
  else{
    Sp_V_R = Sp_V_R;
    Sp_inte_R = Sp_inte_prev_R;    
  }

  Sp_PWMval_R = int(250*Sp_V_R/Sp_Vmax_R);
  Sp_PWM_F_R = Sp_PWM_F_R + Sp_PWMval_R;

  if(Sp_PWM_F_R > 250){
    Sp_PWM_F_R = 250;
  }
  else if(Sp_PWM_F_R < -250){
    Sp_PWM_F_R = -250;
  }
  else{
    Sp_PWM_F_R = Sp_PWM_F_R;
  }

  if(Sp_PWM_F_R>1){
    analogWrite(RMS,Sp_PWM_F_R);
    digitalWrite(RMD,0);  
  }
  else if(Sp_PWM_F_R<0){
    analogWrite(RMS,-1*Sp_PWM_F_R);
    digitalWrite(RMD,1); 
  }
  else{
    analogWrite(RMS,0);
    digitalWrite(RMD,0);     
  }
  Sp_T_prev_R = Sp_T_R;
  Sp_inte_prev_R = Sp_inte_R;
  Sp_e_prev_R = Sp_e_R;

}
void LCD(){ // LCD 화면
  if(AorM == 0){
    lcd.setCursor(0,0);
    lcd.print("Manual Mode     ");
    lcd.setCursor(0,1);
    lcd.print("Gear:");
    lcd.setCursor(5,1);
    if(Lge == 0){
      lcd.print("D ");
    }
    else if(Lge == 1){
      lcd.print("N ");
    }
    else{
      lcd.print("R ");
    }
    lcd.setCursor(7,1);
    lcd.print("ESTOP:");
    lcd.setCursor(13,1);
    if(Les == 1 and Hes == 0){
      lcd.print("R  ");
    }
    else if(Les == 0 and Hes == 1){
      lcd.print("H  ");
    }
    else if( Les == 1 and Hes == 1){
      lcd.print("A  ");
    }
    else{
      lcd.print("X  ");
    }
  }
  else if(AorM == 2){
    lcd.setCursor(0,0);
    lcd.print("Debugging Mode  ");
    lcd.setCursor(0,1);
    lcd.print("thr:");
    lcd.setCursor(4,1);
    lcd.print(throttle);
    lcd.setCursor(8,1);
    lcd.print("str:");
    lcd.setCursor(12,1);
    lcd.print(steer);
  }
  else{
    lcd.setCursor(0,0);
    lcd.print("Auto Mode       ");
    lcd.setCursor(0,1);
    lcd.print("Con:");
    if(Con_status == 1){
      lcd.setCursor(4,1);
      lcd.print("O  ");
    }
    else{
      lcd.setCursor(4,1);
      lcd.print("X  ");      
    }
    lcd.setCursor(7,1);
    lcd.print("ESTOP:");
    lcd.setCursor(13,1);
    if(Les == 1 and Hes == 0){
      lcd.print("R  ");
    }
    else if(Les == 0 and Hes == 1){
      lcd.print("H  ");
    }
    else if( Les == 1 and Hes == 1){
      lcd.print("A  ");
    }
    else{
      lcd.print("X  ");
    }
  }
}

void Send(){
  Steer_Feed = map(encoder_val,St_M+St_D,St_M-St_D,-2000,2000); //조향모터의 피드백값 저장
  Steer_Feed = constrain(Steer_Feed,-2000,2000);
  rpmtime = millis(); 
  E = E_R;
  R = (rpmtime - rpmtime_prev);
  rev = (1000000.00*E)/280.00;
  rps = rev/R;
  RPM = 6*rps; //플랫폼의 바퀴 rpm
  
  rpmtime_prev = rpmtime;
  Senddata[0] = 'S';
  Senddata[1] = 'T';
  Senddata[2] = 'X';
  Senddata[3] = AorM | 0x00;
  Senddata[4] = Les | 0x00;
  Senddata[5] = Lge | 0x00;
  Senddata[6] = (RPM >> 8) | 0x00;
  Senddata[7] = RPM | 0x00;
  Senddata[8] = (Steer_Feed >> 8) | 0x00;
  Senddata[9] = Steer_Feed | 0x00;
  Senddata[10] = Lal | 0x00;
  Senddata[11] = 0x0D;
  Senddata[12] = 0x0A;
  Serial2.write(Senddata,13);
  Lal=Lal+1;
}

void ISR_EncoderA() { // 엔코더 값을 저장하기 위한 내부 인터럽트 서비스 함수
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount_R++;
    }
    else {
      EncoderCount_R--;
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

void ISR_EncoderB() { // 엔코더 값을 저장하기 위한 내부 인터럽트 서비스 함수
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