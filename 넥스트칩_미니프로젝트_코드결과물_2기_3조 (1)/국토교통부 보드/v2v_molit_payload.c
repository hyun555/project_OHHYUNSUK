// SDK_files/wayland_npu_app/v2v_molit_payload.c
/**
********************************************************************************
* @file    : v2v_molit_payload.c
* @brief   : 국토교통부 보드(vehicle_id=100) 전용 payload 송신 구현.
********************************************************************************
*/
#include <string.h>

#include "v2v_molit_payload.h"
#include "v2v_comm.h"

/* SDK 단조시간(ms) - nc_utils.c (wayland_npu_app_v6.c와 동일 패턴으로 extern 선언) */
extern uint64_t nc_get_mono_time(void);

WeatherType v2v_molit_weather_str_to_enum(const char *weather_str)
{
    static WeatherType last_known = WEATHER_SUNNY;   /* 과도상태 문자열 대비 직전값 유지 */

    if (weather_str) {
        if (strcmp(weather_str, "맑음") == 0) {
            last_known = WEATHER_SUNNY;
        } else if (strcmp(weather_str, "야간") == 0) {
            last_known = WEATHER_NIGHT;
        } else if (strcmp(weather_str, "야간 & 우천") == 0) {
            last_known = WEATHER_RAIN;
        }
        /* 그 외("분석 중...", "전환 중...", "분석 오류" 등)는 last_known 유지 */
    }

    return last_known;
}

void v2v_molit_send_payload(uint32_t my_vehicle_id, const char *weather_str, TrafficCongestion_e congestion)
{
    V2VEventMsg st;
    memset(&st, 0, sizeof(st));   /* event_type/front_distance/position/speed = 0 (미사용) */

    st.magic      = V2V_MAGIC;
    st.vehicle_id = my_vehicle_id;
    st.timestamp  = nc_get_mono_time();
    st.brake      = (int32_t)v2v_molit_weather_str_to_enum(weather_str);
    st.lane       = (int32_t)congestion;

    v2v_send_event(&st);
}
