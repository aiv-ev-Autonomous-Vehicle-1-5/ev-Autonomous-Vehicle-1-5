# t870_util 패키지 파이프라인

## 패키지 정보
- **Package**: t870_util
- **Description**: t870_ros 패키지용 유틸리티 라이브러리
- ROS 토픽 없음 (순수 유틸리티 라이브러리)

---

## 컴포넌트

### 1. LogHandler (추상 기반 클래스)

로그 출력을 위한 추상 인터페이스.

- **LogHandlerTerminal**: ANSI 컬러 터미널 출력
- **LogHandlerFile**: 파일 로깅

### 2. 로그 매크로

| 매크로 | 설명 |
|--------|------|
| `T870_DEBUG` | 디버그 레벨 로그 |
| `T870_INFO` | 정보 레벨 로그 |
| `T870_WARN` | 경고 레벨 로그 |
| `T870_ERROR` | 에러 레벨 로그 |

### 3. Exception 클래스

`std::runtime_error`를 상속한 예외 클래스.
