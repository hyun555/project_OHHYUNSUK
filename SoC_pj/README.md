# MNIST CNN 연산 가속기 설계

## 첨부 파일
  - rtl : 전체 코드
  - report : 8page 분량의 보고서 (결과 캡쳐 포함) <br><br><br>

## 설계 개요
end-to-end 연산 가속기를 설계한다. LeNet-1 모델을 사용하며 분류 데이터는 MNIST (0~9 28x28 grayscale)이다. 
arty z7-20보드를 사용하며, Vivado + Vitis 기반으로 설계한다.
  

## 설계 CNN 모델
<img width="1207" height="624" alt="스크린샷 2025-11-18 172104" src="https://github.com/user-attachments/assets/15cdd1fd-3c27-4a9c-a302-7f3a24ee1f27" />

  - conv 층은 2개를 사용한다. 이때 5x5x4 feature map을 conv2에도 재사용한다. 
    
  - 각 층마다 max pooling을 진행한다.
    
  - FC layer는 1개 층만 사용하며 (192x10) 비교기를 추가하여 어떤 숫자에 해당하는지 분류한다.
    
  - 각 층마다 ReLU를 활성화 함수로 사용한다. 이때 +64/ 128을 사용하여 각 conv가 종료된 시점에서의 합을 8bit 크기로 재양자화한다.
    
  - 양자화된 가중치를 학습을 통해 생성하고 각 CONV1, CONV2, FC 층마다 사용한다. <br><br>
    

## 설계 방법

- 전체 시스템 다이어 그램 
<img width="1079" height="461" alt="image" src="https://github.com/user-attachments/assets/6dac2e14-9f65-45ec-8ff3-81e65b8db58f" />

- 설계 요약
1. PS
   AXI DMA : 가중치 데이터를 CNN 가속기 구동 시 한번만 전송한다. AXI DMA를 사용하여 해당 버퍼를
   AXI Stream 형태로 전송한다. 이때 DMA는 tdata[7:0]을 통해 8bit 데이터를 총 3220번 전송한다. 가속기 내부
   TOP 모듈이 Stream 형태로 수신하면서 weight BRAM에 저장한다. <br>
   
2. PL
   1) weight bram reader
      conv1, conv2에 사용되는 weight 1300개를 저장한다. BRAM에는 8bit 데이터 5개씩을 1word로 하여 저장한다.
   2) CONVOLUTION
      1. linebuffer
         axi stream으로 입력 데이터(28x28 grayscale)가 1개씩 들어올 때마다 line buffer로 shift하여 저장한다. line buffer가 모두 채워진 순간부터 conv 연산이 시작된다.
         <img width="1062" height="227" alt="image" src="https://github.com/user-attachments/assets/c9be073d-270f-46b5-b3cf-6d2131ace0b4" />

         
      2. PE_array & adder_tree
         weight bram으로부터 conv 연산에 필요한 가중치를 5개씩 불러와 weight flat buffer을 채운다. 100개의 가중치가 모두 채워지면 입력 값과 함께 연산이 시작된다. adder_tree에서는 이전 PE_array를 통해 25개의 곱셈 결과를 전부 더한 뒤 1개의 결과로 만든다. 이때 critical path를 고려하여 stage를 나눠 연산한다. 
<img width="738" height="323" alt="image" src="https://github.com/user-attachments/assets/b9ef2e87-be78-4fb8-b177-b2f148d006a5" />
<br>

  3) requantization
     conv 연산 결과를 다음 단계인 max pooling에서 사용 가능한 형태인 8bit unsigned 값으로 변환한다. 이때 ReLU와 양자화 과정을 수행한다. +64 / 128을 한다. 재양자화 이후 값이 255를 넘을 시 255로 하고 이보다 작을 시 그대로 내보낸다.
     <br>
     
  4) pooling_linebuffer & maxpooling
     pooling line buffer는 conv 출력 값을 1개씩 받고 이후 maxpooling에 필요한 2x2 윈도우를 만들어 넘긴다. 이때 conv1, conv2 각각 입력 크기가 24x24, 8x8로 다르기에 이 사이즈에 맞춰서 line buffer를 생성한다. maxpooling에서는 pooling linebuffer를 통해 넘어온 2x2 값들로부터 가장 큰 값을 출력한다.
     <img width="1047" height="233" alt="image" src="https://github.com/user-attachments/assets/b4a62771-a180-43ca-84e6-07eb983b198c" />
<br>
     
  5) FC layer
     max pooling 계층에서 추출된 2차원 특징 벡터를 1차원 벡터 형태로 평탄화 한 뒤 각 클래스에 대응되는 가중치와의 곱셈 및 누산 연산을 통해 최종 분류를 한다. 5개의 PE를 파이프라이닝을 하여 사용함으로써 자원 효율을 높인다. 이후 comparator 모듈을 통해 최종 최종 분류 값을 출력한다.

구체적인 설계 방법은 CNN Acceleration System design pj.pdf 참고


## 주요 결과
   - MNIST 분류 정확도 : 99.6%
   - PS/PL speedup : 559배 상승
   - LUT 21%/ BRAM 3%/ DSP 48% 사용
<img width="1057" height="164" alt="image" src="https://github.com/user-attachments/assets/84880525-ea54-4732-ba97-79154344cc08" />
<img width="556" height="162" alt="image" src="https://github.com/user-attachments/assets/27217cdf-75b9-4202-8636-f36c5dec49ff" />
