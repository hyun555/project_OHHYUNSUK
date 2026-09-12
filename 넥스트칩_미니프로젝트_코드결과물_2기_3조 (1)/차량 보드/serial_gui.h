/**
********************************************************************************
* @file    : serial_gui.h
* @brief   : Qt(VM) -> 보드 시리얼(/dev/ttyS2) 수신 모듈
*            - VM의 Qt 앱이 시리얼로 보낸 이벤트 문자열을 받아 EventType 으로 변환.
*            - 별도 스레드로 계속 읽어 전역에 저장. wayland 가 읽어서 사용.
*            - 랜선은 V2V(보드간) 전용이므로 Qt->보드는 시리얼로 분리.
********************************************************************************
*/
#ifndef SERIAL_GUI_H
#define SERIAL_GUI_H

#include "v2v_comm.h"   /* EventType */

#ifdef __cplusplus
extern "C" {
#endif

/* 시리얼 수신 시작 (스레드 생성). 성공 0, 실패 -1.
 *   dev 예: "/dev/ttyS2" (NULL이면 기본값 사용) */
int  serial_gui_init(const char *dev);
void serial_gui_stop(void);

/* 가장 최근 Qt 이벤트(EventType) 반환.
 *   아직 아무 입력 없으면 EVT_LANE_CHANGE_LEFT_OFF(평소 대표값) 반환. */
uint32_t serial_gui_get_event(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_GUI_H */
