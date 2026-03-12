# SoC 설계 프로젝트

### 첨부 파일
  - rtl : 전체 코드
  - report : 8page 분량의 보고서 (결과 캡쳐 포함) <br><br><br>

### 설계 개요
  목표 : end-to-end 연산 가속기를 설계한다.
  
  모델 : LeNet-1 on MNIST (28x28 grayscale)
  
  플렛폼 : arty z7-20 보드를 사용
  
  툴 : Vivado + Vitis <br><br>

### 설계 CNN 모델
<img width="1207" height="624" alt="스크린샷 2025-11-18 172104" src="https://github.com/user-attachments/assets/15cdd1fd-3c27-4a9c-a302-7f3a24ee1f27" />

  - conv 층은 2개를 사용한다. 이때 5x5x4 feature map을 conv2에도 재사용한다. 
    
  - 각 층마다 max pooling을 진행한다.
    
  - FC layer는 1개 층만 사용하며 (192x10) 비교기를 추가하여 어떤 숫자에 해당하는지 분류한다.
    
  - 각 층마다 ReLU를 활성화 함수로 사용한다. 이때 +64/ 128을 사용하여 각 conv가 종료된 시점에서의 합을 8bit 크기로 재양자화한다.
    
  - 양자화된 가중치를 학습을 통해 생성하고 각 CONV1, CONV2, FC 층마다 사용한다. <br><br>
    

### 설계 방법
   - [report](CNN%Acceleration%System%design%pj.pdf)
   - 구체적인 설계 방법, 아이디어 6page 분량으로 정리 <br>

### 주요 결과
   - MNIST 분류 정확도 : 99.6%
   - PS/PL speedup : 559% 상승
   - LUT 21%/ BRAM 3%/ DSP 48% 사용
