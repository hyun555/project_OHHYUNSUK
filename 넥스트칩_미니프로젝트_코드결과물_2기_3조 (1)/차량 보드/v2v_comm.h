// SDK_files/wayland_npu_app/v2v_comm.h
/**
********************************************************************************
* @file    : v2v_comm.h
* @brief   : V2V(차량간) TCP 통신 - 순수 통신만 담당 (판단 없음)
*            - 각 보드는 서버(accept)+클라이언트(connect)를 겸하는 P2P 노드
*            - 메시지는 고정 크기 구조체 V2VEventMsg (TCP 프레이밍은 recv_all)
*            - ★ v2v_comm 은 절대 판단하지 않는다. 수신 메시지를 앱에 넘길 뿐이다.
*              감속/정지/차선변경/wait 등 모든 제어 판단은 wayland_npu_app.c 담당.
*            - 날씨/혼잡도는 보드끼리 주고받지 않음(구조체에서 제외).
*            [v8] WeatherType/TrafficCongestionType 추가 - vehicle_id=100(국토교통부)
*            송신측(v2v_molit_payload.c, v7)과 수신측(nc_v2v_decide(), v8) 둘 다
*            공유해야 해서 이 공용 헤더로 옮김(Temp.md 필드 재해석 규칙 참고).
********************************************************************************
*/
#ifndef V2V_COMM_H
#define V2V_COMM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 프레이밍 검증용 매직값 & 포트 */
#define V2V_MAGIC   0x56325645u   /* 'V2VE' - 수신측에서 메시지 검증 */
#define V2V_PORT    13002

/* 이벤트 종류 = Qt 신호 전용 (ON/OFF 쌍)
 *   - event_type 에는 이 중 하나만 들어감. Qt 버튼 신호를 담는다.
 *   - 영상 기반 급정지 위험은 event_type 이 아니라 아래 구조체의 brake 필드로 보낸다.
 *   - 아무것도 안 누른 평소 상태 = EVT_LANE_CHANGE_LEFT_OFF (NONE 대용).
 *     (ON 상황만 판단에 반응하고, 마지막이 OFF면 "이벤트 없음"과 같음) */
typedef enum {
    EVT_LANE_CHANGE_LEFT_ON = 0, /* 좌 차선변경 켬 */
    EVT_LANE_CHANGE_LEFT_OFF,    /* 좌 차선변경 끔 (= 평소/이벤트 없음 대표값) */
    EVT_LANE_CHANGE_RIGHT_ON,    /* 우 차선변경 켬 */
    EVT_LANE_CHANGE_RIGHT_OFF,   /* 우 차선변경 끔 */
    EVT_EMERGENCY_BUTTON_ON,     /* Qt 비상(SOS) 켬 */
    EVT_EMERGENCY_BUTTON_OFF,    /* Qt 비상(SOS) 끔 */
} EventType;

/* [v8] vehicle_id=100(국토교통부)일 때만 V2VEventMsg의 brake 필드를 이 값으로 해석.
 *   Analyze_Weather_Condition() 판정 로직엔 SNOW가 없어 현재는 미사용. */
typedef enum {
    WEATHER_SUNNY = 0,   /* 맑음 */
    WEATHER_NIGHT,       /* 야간 */
    WEATHER_RAIN,        /* 빗길 (야간 & 비) */
    WEATHER_SNOW,        /* 눈 */
} WeatherType;

/* [v8] vehicle_id=100(국토교통부)일 때만 V2VEventMsg의 lane 필드를 이 값으로 해석.
 *   값이 TrafficCongestion_e(traffic_congestion.h)와 동일 - 그대로 캐스팅 가능. */
typedef enum {
    TRAFFIC_SMOOTH = 0,  /* 원활 */
    TRAFFIC_NORMAL,      /* 보통 */
    TRAFFIC_HEAVY,       /* 혼잡 */
    TRAFFIC_JAM,         /* 정체 */
} TrafficCongestionType;

/* V2V 메시지 (보드끼리 주고받는 정보) - 고정 크기, 모든 필드 항상 전송
 *   [Qt 신호]    event_type (내 Qt 조작; 수신 시엔 상대 Qt 조작)
 *   [영상 기반]  front_distance, brake(영상 급정지 위험 0/1)
 *   [고정 설정]  position(1D 직선도로 위치), lane(차선번호)
 *   [내부 엔진]  speed (V2V/Qt 상호작용으로만 변함)
 *
 *   ★ event_type(Qt SOS)과 brake(영상 위험)는 별도 필드. 판단이 다르다:
 *     - Qt SOS(event_type)  : 거리 무관, 뒤+같은lane이면 -20 서행
 *     - 영상 brake          : 20m 이내+같은lane 완전정지, 그 밖 -20 서행
 *
 *   [v8] vehicle_id=100일 때는 위 해석이 전부 무효 - brake->WeatherType,
 *     lane->TrafficCongestionType, 나머지 필드는 0(미사용). */
typedef struct {
    uint32_t magic;           /* V2V_MAGIC 고정 - 검증 */
    uint32_t vehicle_id;      /* 송신 차량 ID (1~3: 차량, 100: 국토교통부) */
    uint32_t event_type;      /* EventType (Qt 신호. 평소 = LEFT_OFF) */
    uint64_t timestamp;       /* 송신 시각 */

    float    front_distance;  /* 영상 기반 앞차 거리(m), 없으면 999 */
    int32_t  brake;           /* 영상 기반 급정지 위험: 0=false, 1=true / [vehicle_id=100] WeatherType */

    float    position;        /* 직선도로 상의 내 위치(1D, 고정값) */
    int32_t  lane;            /* 내 차선 번호(고정값) / [vehicle_id=100] TrafficCongestionType */
    float    speed;           /* 내 현재 속도(내부 엔진값) */
} V2VEventMsg;

/* ── 공개 API (통신만) ────────────────────────────────── */
void v2v_init(uint32_t my_vehicle_id, const char **peer_ips, int peer_cnt);
void v2v_stop(void);
int  v2v_send_event(const V2VEventMsg *ev);
/* 수신 메시지가 있으면 out에 복사하고 1 반환(소비), 없으면 0 */
int  v2v_poll_event(V2VEventMsg *out);

#ifdef __cplusplus
}
#endif

#endif /* V2V_COMM_H */
