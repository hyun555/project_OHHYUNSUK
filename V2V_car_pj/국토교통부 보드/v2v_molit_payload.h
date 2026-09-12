// SDK_files/wayland_npu_app/v2v_molit_payload.h
/**
********************************************************************************
* @file    : v2v_molit_payload.h
* @brief   : 국토교통부 보드(vehicle_id=100) 전용 - 기상/혼잡도 분석 결과를
*            V2VEventMsg로 패킹해 v2v_send_event()로 전송.
*            [필드 재해석 규칙, Temp.md 참고] vehicle_id=100일 때만:
*              brake -> WeatherType, lane -> TrafficCongestionType, 나머지 0.
*            수신측(차량 보드) 재해석 로직은 v8(`wayland_npu_app_v8.c`)에서 구현.
*            [v8] WeatherType/TrafficCongestionType은 송수신 양쪽이 공유해야 해서
*            `v2v_comm.h`로 옮겨감 - 이 헤더에서 중복 정의하지 않고 그쪽 것을 그대로 씀.
********************************************************************************
*/
#ifndef V2V_MOLIT_PAYLOAD_H
#define V2V_MOLIT_PAYLOAD_H

#include <stdint.h>
#include "v2v_comm.h"             /* [v8] WeatherType 정의처 */
#include "traffic_congestion.h"   /* TrafficCongestion_e - 값이 TrafficCongestionType과 동일해 그대로 재사용 */

#ifdef __cplusplus
extern "C" {
#endif

/* Analyze_Weather_Condition() 반환 문자열 -> WeatherType.
 *   "맑음"/"야간"/"야간 & 우천"만 매핑. 그 외(과도상태 "분석 중..."/"전환 중..."/
 *   "분석 오류" 등 매핑 안 되는 문자열)는 직전에 매핑됐던 값을 그대로 반환(내부 static 상태 유지),
 *   최초 호출 전 기본값은 WEATHER_SUNNY. */
WeatherType v2v_molit_weather_str_to_enum(const char *weather_str);

/* 기상 문자열 + 혼잡도를 V2VEventMsg(vehicle_id=my_vehicle_id, brake=날씨, lane=혼잡도,
 * 나머지 필드 0)로 패킹해 v2v_send_event() 호출. */
void v2v_molit_send_payload(uint32_t my_vehicle_id, const char *weather_str, TrafficCongestion_e congestion);

#ifdef __cplusplus
}
#endif

#endif /* V2V_MOLIT_PAYLOAD_H */
