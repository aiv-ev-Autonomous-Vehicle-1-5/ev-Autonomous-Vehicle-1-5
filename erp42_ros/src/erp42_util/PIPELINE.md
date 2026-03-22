# erp42_util

## Package 정보
- **패키지명**: erp42_util
- **설명**: ERP42 ROS2 유틸리티 라이브러리
- **ROS 토픽 없음** (순수 유틸리티)

## 컴포넌트

### 1. Exception 클래스
- `std::runtime_error` 상속
- ERP42 전용 예외 처리

### 2. 로깅 시스템
매크로 기반 로깅 인터페이스

#### 로깅 매크로
| 매크로 | 로그 레벨 | 색상 |
|---|---|---|
| `ERP42_DEBUG` | DEBUG | 시안 |
| `ERP42_INFO` | INFO | 초록 |
| `ERP42_WARN` | WARNING | 노랑 |
| `ERP42_ERROR` | ERROR | 빨강 |

#### 로그 핸들러
| 핸들러 | 설명 |
|---|---|
| `LogHandlerTerminal` | ANSI 컬러 터미널 출력 |
| `LogHandlerFile` | 파일 로깅 |

#### 특징
- 스레드 안전 (mutex)
- 싱글톤 `LogManager`
