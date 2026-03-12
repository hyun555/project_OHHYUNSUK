# SoC 설계 프로젝트

### 첨부 파일
  - rtl : 전체 코드
  - report : 8page 분량의 보고서 (결과 캡쳐 포함) <br><br><br>

### 설계 개요
  목표 : end-to-end 연산 가속기를 설계한다.
  모델 : LeNet-1 on MNIST (28x28 grayscale)
  플렛폼 : arty z7-20 보드를 사용
  툴 : Vivado + Vitis <br><br>

  CNN 모델
<img width="1207" height="624" alt="스크린샷 2025-11-18 172104" src="https://github.com/user-attachments/assets/15cdd1fd-3c27-4a9c-a302-7f3a24ee1f27" />
  - conv 층은 2개를 사용한다.
  - 각 층마다 max pooling을 한다.
  - FC layer는 1개 층만 사용한다.
  - 각 층마다 ReLU를 활성화 함수로 사용한다.
  - 양자화된 가중치를 각 CONV1, CONV2, FC 층마다 .h 파일로 사용한다.

### 설계 방법 및 결과
   - 자세한 사항 report 참고

     
