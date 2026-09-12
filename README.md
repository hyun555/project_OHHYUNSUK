# project_OHHYUNSUK
포트폴리오 모음

1. [SoC_pj](SoC_pj) - MNIST CNN 연산 가속기 설계 (Arty Z7-20)
   - README : 설계 개요 및 결과
   - rtl : 전체 Verilog 코드
   - CNN Acceleration System design pj.pdf : 원본 보고서

2. [RISCV_pj](RISCV_pj) - GEMM 8x8 연산용 RISC-V 프로세서 설계
   - README : 설계 조건, 어셈블리 수정 전후, 합성 결과
   - rtl : 5단 파이프라인 CPU Verilog 코드, 어셈블리 및 hex 파일

3. [elevator_pj](elevator_pj) - DE2 보드를 사용한 엘리베이터 설계
   - README : 기능 개요, UART 규약, FSM 설명
   - rtl : 전체 Verilog 코드

4. [V2V_car_pj](V2V_car_pj) - V2V 차량 협력 주행 임베디드 시스템 설계 (Apache6 SoM 4대)
   - README : 시스템 구성, 핵심 코드 설명, 시연 영상 링크
   - 차량 보드 : 차량용 C 코드
   - 국토교통부 보드 : 날씨 및 혼잡도 송신용 C 코드
