# DE2 보드를 사용한 엘리베이터 설계

 - 실제 엘리베이터 동작 알고리즘과 동일하게 설계한다.
 - Quartus 툴을 사용하여 설계 후 DE2 보드에 올려 동작을 검증한다.
 - Putty 또는 python GUI 창을 명령 입력 및 현재 상태 출력이 가능하게 한다.<br>

### 1. 기능 개요

   1) 주기 상태 송신(UART) : 1초마다 현재 층, 방향(UP/DOWN/STOP)전송
   2) 정지(정차) 규칙 : 정지 시 기본 3초 정차(문 열림 유지).
      - 열림 버튼 누르고 있는 동안 계속 STOP 유지(3초 이후라도 연장)
      - 닫힘 버튼 누르면 3초 이전이라도 즉시 정차 종료 후 다음 동작(UP/DOWN)을 진행
   3) 호출 입력 (UART RX, Putty)
      - 사용자가 숫자 + UP 또는 숫자 +DOWN 입력 후 Enter -> 해당 층의 Hall Call 등록.
   4) 스케줄링(현대식 컬렉티브 컨트롤)
      - 현재 진행 방향 우선 : 진행 방향 상의 모든 콜을 층 순서대로 처리
      - 진행 방향에 콜이 없으면 반대 방향으로 전환
      - 같은 층에 상/하 동시 콜이 있으면 현재 진행 방향 콜 우선 <bar>
     
### 2. UART

   1) 115200 baud
   2) TX 상태 프레임(1초마다)
      - 포멧 : F:<floor>, DIR:<UP|DOWN|STOP>\r\n
      - 예시) F:05,  DIR:UP\r\n
   3) RX 호출 명령(PuTTY 또는 python GUI 창을 사용!)
      - 홀콜 : <floor><UP|DOWN>\r\n
      - 예시) 5UP enter, 12DOWN enter => 이렇게 콜할 층 및 방향 입력
   4) 에러 응답 : 파싱 실패 시 (잘못된 입력 - 잘못된 층수, 잘못된 방향, 기타 잘못된 입력값 들어올 시)
      - ERR:CMD\r\n 을 출력 <bar>

### 3. 상태머신(FSM)

   1) S_IDLE_STOP
       - 콜 발생 -> 방향 결정 -> S_DOOR_CLOSING -> S_MOVING_*
       - 현재층 콜 존재 -> S_DOOR_OPENING -> S_DWELL
   2) S_DOOR_OPENING/CLOSING
       -문이 열리거나 닫히는데 걸리는 시간은 1초로 한다.
       -BTN_OPEN, BTN_CLOSE=1 되어도 State에는 영향을 주지 않는다.
   3) S_DWELL
       -BTN_OPEN=1일 때 S_DWELL 상태를 유지한다. (엘리베이터 열림 버튼을 꾹 누르고 있는 상황)
       -BTN_CLOSE=1 또는 타이머 만료(3초후 닫힘) : S_DOOR_CLOSING 상태로 넘어간다.
   4) S_MOVING_UP/DOWN
       - 현재 엘리베이터가 움직이는 방향을 말한다.
       - 다음 목적층 도달하면 S_DOOR_OPENING -> S_DWELL 상태가 된다. (도착 후 문열림 상태!)
       - 진행 구간에 콜 없고 반대 방향에 콜이 있으면 반대 방향으로 전환한다.
   5) S_EMG
       - 비상 버튼과 같다.
       - S_EMG 버튼을 누르면 현재 어떤 상황이라도 멈춘다.
       - S_EMG가 해제되면 다시 동작을 이어간다. <bar>

### 4. 스케쥴러 및 기타 규칙

   1) 진행 방향 우선 규칙
       - 만약 현재 방향이 UP이면 (현재층+1 ~ 최상층) 구간에서 UP 호출이 발견되면 가장가까운 상층부터 방문한다.
       - 해당 구간에 아무 콜이 없으면 DOWN으로 전환하고 (현재층-1 ~ 0)을 스캔한다.
       - STOP 상태에서는 최단 거리 층을 우선으로 한다.  (5층에서 stop 상태인데 2층, 11층 호출이 동시에 들어오면 2층부터 방문)
       - 같은 층 상/하 동시 콜 : 현재 진행 방향 콜을 우선 처리한다.
         (예시 - 현재 진행 방향이 UP 인 상황에서 10층 UP, 10층 DOWN이 동시에 들어오면 10층 UP부터 처리한다.)
   2) 정차 조건
       - 진행방향과 같은 방향의 콜이 있거나 차내 콜이 있으면 정차한다.
       - 정차 -> 문열림(1초) -> DWELL상태(3초)
   3) 문열림 / 닫힘
       - BTN_OPEN=1 : DWELL 상태를 연장한다(문열림 유지)
       - BTL_CLOSE=1 : DWELL 상태를 즉시 종료한다(문닫힘 버튼 입력시 바로 문닫고 출발!)
   4) 비상/예외
       - EMG_STOP=1 : 즉시 엘리베이터를 멈춘다. <bar>
      
### 5.기타 항목
   - 현재 층을 시각적으로 나타내기 위해 7SEGMENT를 사용하여 보드에 현재 층이 보이게 한다.
   - UP, DOWN 모두 보드의 서로 다른 LED에 지정한다. <bar>

### <사용 보드>
  <img width="1000" height="674" alt="image" src="https://github.com/user-attachments/assets/8a8dd435-429e-432d-b506-d168cd192e26" />

###  <전체 시스템 개요>
  <img width="984" height="638" alt="스크린샷 2025-11-12 205702" src="https://github.com/user-attachments/assets/09dbbdc2-0bbe-4e50-be01-5b3d03f69488" />

### 설계 코드

rtl 파일 내부 구조

ELEVATOR_TOP
  - U_DB_OPEN : DEBOUNCER.v
  - U_DB_CLOSE : DEBOUNCER.v
  - U_FSM : ELEVATOR_FSM.v
  - U_INTERFACE : ELEVATOR_INTERFACE.v
     - U_RX : UART_RX.v
     - U_TX_ERR : UART_TX.v
     - U_CMD : CMD_PASER.v
     - U_HEX0 : SEVEN_SEGMENT.v
     - U_HEX1 : SEVEN_SEGMENT.v
 

