# 3sphere-wheel-transmit

3-Sphere Omniwheel Mobility — Transmit 펌웨어

조이스틱 ADC 수집 및 nRF24L01 RF 송신 STM32F103 펌웨어입니다.

## Branch 구조

| Branch | 설명 |
|--------|------|
| `master` | 폴링 기반 ADC + HAL_Delay(20ms) RF 송신 |
| `feature/interrupt-conversion` | TIM2(50Hz) + ADC DMA 3채널 동시 수집 + WFI 저전력 구조로 전환 |
| `State_Machine` | UART3로 RPI5 음성인식 명령 수신, (ST_IDLE, ST_JOYSTICK, ST_VOICE) 3상태 머신 구현 |

## 사용 기술
STM32F103, ADC DMA, TIM2, SPI, nRF24L01, UART

## 전체 프로젝트
→ [3sphere-omniwheel-mobility](https://github.com/xodrb/3sphere-omniwheel-mobility)
