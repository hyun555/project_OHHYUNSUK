// SDK_files/wayland_npu_app/wayland_npu_app.c
/**
********************************************************************************
* Copyright (C) 2021 NEXTCHIP Inc. All rights reserved.
********************************************************************************
* @file    : wayland_npu_app.c
* @brief   : wayland_npu application (+ MTMC 객체-색상 매칭 추가)
********************************************************************************
*/

/*
********************************************************************************
*               INCLUDES
********************************************************************************
*/
#include <stdio.h>
/* ★ [BOARD1] UDP 전송용 소켓 헤더 */
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include <signal.h>
#include <linux/input.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include <time.h>
#include <SOIL.h>
#include <sys/eventfd.h>
#include <arm_neon.h>
#include <omp.h>

#include "wayland_egl.h"
#include "nc_opengl_init.h"
#include "v4l2_interface.h"
#include "nc_opengl_shader.h"
#include "nc_opengl_interface.h"
#include "nc_ts_fsync_flipflop_buffers.h"
#include "nc_app_config_parser.h"
#include "nc_cnn_aiware_runtime.h"
#include "nc_cnn_communicator.h"
#include "nc_cnn_worker_for_postprocess.h"
#include "nc_neon.h"
#include "v2v_comm.h"          /* ★ [V2V] 차량간 TCP 통신 */
#include "v2v_topology.h"      /* ★ [v6] peer 연결 topology (자가 IP 감지) */
#include "serial_gui.h"        /* ★ [Qt] VM Qt -> 보드 시리얼 수신 */
/* (날씨/혼잡도는 보드끼리 주고받지 않으므로 제외)
 * #include "weather_analysis.h"
 * #include "traffic_congestion.h" */
#ifdef AIWARE_DEVICE_SUPPORTED
#include "aiware/runtime/c/aiwaredevice.h"
#endif
#include "nc_opengl_ttf_font.h"
#ifdef USE_BYTETRACK
#include "nc_cnn_tracker.h"
#endif

#ifdef USE_8MP_VI
#include "nc_dsr_helper.h"
#include "nc_dsr_set.h"
#include "nc_dmabuf_ctrl_helper.h"
#endif

#ifdef USE_ADAS_LD
#include"ADAS_LD_Lib.h"
#endif

/*
********************************************************************************
*               DEFINES
********************************************************************************
*/
#define VIS0_MAX_CH         (1)
#define VIS1_MAX_CH         (0)
#define VIDEO_MAX_CH        (VIS0_MAX_CH + VIS1_MAX_CH)

#define VIDEO_BUFFER_NUM    (3)

#ifdef USE_8MP_VI
#define VIDEO_WIDTH         MAX_WIDTH_FOR_VDMA_CNN_DS
#define VIDEO_HEIGHT        MAX_HEIGHT_FOR_VDMA_CNN_DS
#else
#define VIDEO_WIDTH         NPU_INPUT_WIDTH
#define VIDEO_HEIGHT        NPU_INPUT_HEIGHT
#endif

#define CAP_FPS             (30)

#define MQ_NAME_CNN_BUF     "/cnn_data"
#define DEV_FILE_DSR        "/dev/dsr"

#ifdef SHOW_PELEE_SEG
    #define NETWORK_FILE_PELEE_SEG      "misc/networks/peleeseg/peleeseg_640x384_apache6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_PELEE_DETECT
    #define NETWORK_FILE_PELEE_DET      "misc/networks/peleeDet/peleedet_10class_640x384_apache6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_YOLOV5_DETECT
    #define NETWORK_FILE_YOLOV5_DET     "misc/networks/Yolov5s/yolov5s_coco_640x384_apache6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_YOLOV8_DETECT
    #define NETWORK_FILE_YOLOV8_DET     "misc/networks/Yolov8/yolov8s_coco_640x384_apache6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_UFLD_LANE
    #define NETWORK_FILE_UFLD_LANE      "misc/networks/ufld/ufld_a6sr250_aiw4939.aiwbin"
#endif
#ifdef SHOW_TRI_CHIMERA
    #define NETWORK_FILE_TRI_CHIMERA    "misc/networks/trichimera/trichimera_640x384_a6sr_aiw4939.aiwbin"
#endif
#define COLOR_PALETTE_CNT    (10)
typedef enum {
    RGBA_R = 0,
    RGBA_G,
    RGBA_B,
    RGBA_A,
    RGBA_CNT,
} E_RGBA_IDX;

/*    don't modify  */
#define MAX_WIDTH_FOR_VDMA_CNN_DS   (1920)
#define MAX_HEIGHT_FOR_VDMA_CNN_DS  (1080)
/********************/

/*
********************************************************************************
*               VARIABLE DECLARATIONS
********************************************************************************
*/
st_npu_input_info npu_input_info;
struct gl_npu_program g_npu_prog;
GLuint g_seg_texture;
struct gl_font_program g_font_prog;
Font font_38;
Font font_24;

unsigned char* image_data;
static int running = 1;

st_nc_v4l2_config v4l2_config[VIDEO_MAX_CH];

struct viewport g_viewport[VIDEO_MAX_CH];

/* ★★★ [BOARD1 추가 시작] 앞차 거리/접근 변화율 분석 설정 ★★★ */

/* --- 영상 입력 전환: 아래 줄을 주석 해제하면 실제 카메라 대신 영상(가상카메라)에서 읽음 --- */
#define USE_VIDEO_LOOPBACK               /* 정의 O = 영상(video11), 정의 X(주석) = 실제 카메라 */
#define LOOPBACK_DEVICE_BASE   (11)      /* run_v4l2loopback.sh 의 시작 디바이스 번호 */

/* --- 거리 추정 (단안 카메라, 부정확해도 됨) ---
 *   dist(m) = FOCAL_PX * REAL_CAR_HEIGHT_M / bbox_height_px
 *   ※ 붙었는데 거리가 크게 나오면 FOCAL_PX 를 낮추세요(반대면 높이기). */
#define CAM_FOCAL_PX           (400.0f)  /* 초점거리(px) - 실제 영상 보고 조정 */
#define REAL_CAR_HEIGHT_M      (1.5f)    /* 일반 승용차 높이(m) */

/* --- 내 차선 필터: 화면 가로 중앙부에 있는 차량만 "앞차" 후보 --- */
#define LANE_CENTER_MIN_RATIO  (0.30f)   /* 화면 가로의 30% ~ */
#define LANE_CENTER_MAX_RATIO  (0.70f)   /* ~ 70% 안에 bbox 중심이 있어야 앞차 */

/* --- 로그 출력 간격: N 프레임에 1번만 출력(도배 방지) --- */
#define LOG_EVERY_N_FRAMES     (8)       /* 30fps 기준 약 초당 3~4줄 */

/* --- 위험 판정 임계값 (실제 영상 보고 조정) --- */
#define WARN_DISTANCE_M        (7.0f)    /* 이 거리 안쪽이면 주의 */
#define DANGER_DISTANCE_M      (4.0f)    /* 이 거리 안쪽이면 위험 */

/* ★ [BOARD1] 경고 조건 개선용
 *   - 옆 차선 차가 잠깐 중앙에 걸쳐 "가깝지만 안 다가옴(level~0)"인데 경고 뜨는 문제 방지
 *   - (a) 거리 위험은 level 도 어느 정도 있어야 인정
 *   - (b) 여러 프레임 연속 위험일 때만 경고 (잠깐 스치는 것 무시) */
#define WARN_MIN_LEVEL         (1.0f)    /* 거리 위험 인정에 필요한 최소 접근레벨 */
#define WARN_HOLD_FRAMES       (3)       /* 연속 이 프레임 이상 위험이어야 경고 */

/* (UDP 모니터링 전송 제거 - V2V(TCP)로 일원화)
 *   관련 매크로(UDP_*)/구조체(st_udp_payload)/전역(g_udp_*) 모두 제거 */

/* ★★★ [V2V] 차량간 TCP 통신 설정 (급정지 이벤트 송수신) ★★★
 *   - 기능2: 다른 차량이 급정지하면 TCP로 EMERGENCY_BRAKE 이벤트를 받고,
 *     내 카메라 앞차 거리를 보고 스스로 감속 여부 판단(decision_engine).
 *   - 통신 실체는 v2v_comm.c 가 담당. 여기선 설정값과 판단만.
 *   - [v6] id/peer 목록은 더 이상 실행 인자/하드코딩이 아니라 v2v_topology_init()의
 *     자가 IP 감지로 자동 결정된다(차량 보드 100% 동일 코드 원칙). position/lane/speed는
 *     차량별 시뮬레이션 파라미터라 기존대로 실행 인자로 받는다.
 *     예) ./app_wayland_npu --position 0 --lane 2 --speed 60 */
static uint32_t g_my_vehicle_id = 1;      /* [v6] v2v_topology_init()이 채움(기본 1은 미사용 안전값) */
static float    g_my_position   = 0.0f;   /* --position(1D 위치, 기본 0) */
static int      g_my_lane       = 1;      /* --lane    (차선, 기본 1) */
static float    g_my_speed      = 60.0f;  /* --speed   (내부 엔진 속도, 기본 60) - V2V 판단만 건드림 */
static float    g_my_speed_init  = 60.0f;  /* 초기 속도 백업(리셋 시 복구용) */

/* [v9] 기상/혼잡도 기반 최대속도 캡을 반영한 "실제 출력 속도".
 *   speed_final = min(speed, 기상·혼잡도 기반 최대속도) - 매 틱 재계산(래칫 아님).
 *   speed(위 g_my_speed)는 이 값과 무관하게 기존 로직(v8.6의 자차 브레이크 반응 포함)
 *   으로만 갱신됨 - 그래서 최대속도가 다시 올라가면 speed_final도 자동으로 speed를
 *   따라 회복됨. position 갱신·V2V 방송(st.speed)은 전부 이 speed_final을 씀. */
static float    g_my_speed_final = 60.0f;

/* [v6] 접속할 다른 차량 IP 목록도 v2v_topology_init()이 자가 IP 감지로 자동 계산.
 *   아래 하드코딩 배열/카운트는 더 이상 쓰지 않음(주석처리, 삭제 아님). */
// static const char *g_v2v_peer_ips[] = {
//     NULL,   /* 실제 보드 연결 시 "192.168.13.34" 처럼 추가 + 아래 V2V_PEER_CNT 조정 */
// };
// #define V2V_PEER_CNT  (0)

/* ★★★ [출력 상태값] 영상 분석으로 매 프레임 갱신 -> V2V 송신 시 사용 ★★★
 *   - front_distance, approach_warn 만 영상 기반. (speed는 영상이 안 건드림) */
/* (APPROACH_WARN_TRUE_LEVEL 제거 - warn은 초기모델 WARN_DISTANCE_M/WARN_MIN_LEVEL 사용) */

static float g_out_front_distance = 999.0f;  /* 앞차 거리(영상) */
static int   g_out_approach_warn  = 0;       /* 접근 경고 T/F(영상) */

/* ★★★ [V2V 판단 파라미터] 판단은 전부 여기(wayland). v2v_comm은 통신만. ★★★
 *   급정지: 송신자 position vs 내 position 으로 앞/뒤·거리 판단
 *   차선변경: 옆차선 + 근접(10m) + speed 비교 -> wait / 무시 */
#define V2V_BRAKE_NEAR_M       (20.0f)   /* 영상 brake: 이 이내+같은lane 이면 완전정지 */
#define V2V_BRAKE_FAR_IGNORE_M (100.0f)  /* 이보다 멀리 떨어진 차의 brake 는 무시 */
#define SPEED_SLOW_MIN         (20.0f)   /* 서행 하한 - 서행으로는 이 아래로 안 내려감 */
#define LANE_CHANGE_NEAR_M     (10.0f)   /* 차선변경: position 차이 이 이내면 "근접" */
#define SPEED_SLOW_DELTA       (20.0f)   /* 서행 감속량: 현재 속도에서 20 줄임 */
#define V2V_WAIT_MS            (5000)    /* 차선변경 양보 시 wait 시간(ms) = 5초 */
#define V2V_STATE_HOLD_MS      (3000)    /* 판단 결과 화면 유지(ms) */

/* [v8] 국토교통부 보드(vehicle_id) - 매핑표상 100 고정. 하드코딩이지만 값 자체가
 *   원래 고정 상수(IP/vehicle_id 매핑, CONTEXT.md 참고)라 topology와 별개로 둬도 됨. */
#define V2V_MOLIT_VEHICLE_ID   (100u)

/* ── [시간축] position/speed 갱신 (1초 주기 = 30프레임 카운트로 처리) ──
 *   speed(km/h)를 1초에 이동하는 position 증가량으로 환산.
 *   실제 60km/h=16.7m/s 지만 로그가 너무 빨리 커지지 않게 스케일 축소. */
#define SPEED_TO_POS_SCALE     (0.1f)    /* 1초 이동량 = speed * 0.1 (60 -> 6/s) */
#define POS_RESET_FRAMES       (1200)    /* position 자동 리셋 주기(프레임) = 40초(30fps) */

/* ── [급정지 감속/목표정지] ──
 *   급정지(STOP): 빠르게 감속.  서서히(SLOW): 미리 조금씩 감속.
 *   목표: 앞차(송신자) position - STOP_GAP_M 지점에 서고, 추월 금지. */
#define DECEL_STOP_PER_S       (20.0f)   /* STOP 시 1초당 speed 감소량(빠름) */
#define DECEL_SLOW_PER_S       (4.0f)    /* SLOW 시 1초당 speed 감소량(천천히, 목표 근처까지) */
#define STOP_GAP_M             (5.0f)    /* 앞차 position 이 거리만큼 뒤에 정지 */

/* ── [warn latch] 거리 기준(초기 모델): WARN_DISTANCE_M/WARN_MIN_LEVEL 사용 ── */

/* 판단 결과(내 차량이 취할 행동) */
typedef enum {
    V2V_ACT_NONE = 0,     /* 무시(속도 유지) */
    V2V_ACT_CAUTION,      /* 주의 표시만 */
    V2V_ACT_SLOW,         /* 서서히 감속(영상 brake 목표정지) */
    V2V_ACT_STOP,         /* 즉시 정지 */
    V2V_ACT_WAIT,         /* 차선변경 양보: wait 후 진행 */
    V2V_ACT_SLOW_CRUISE,  /* 서행: 속도 -20 하고 계속 주행(Qt SOS / 영상 brake 원거리) */
} E_V2V_ACTION;

static E_V2V_ACTION g_v2v_action    = V2V_ACT_NONE;
static uint64_t     g_v2v_action_ms = 0;

/* 급정지 목표정지용 상태 */
static float    g_stop_target_pos = 0.0f;  /* 정지 목표 position (앞차 - STOP_GAP_M) */
static int      g_stop_active     = 0;     /* 목표정지 진행 중(1) */

/* warn latch 상태 */
static int g_warn_latched = 0;   /* 급정지 위험 진입 후 latch */

/* ── 자차 영상 brake 제어: 내 카메라가 앞차 급정지를 보면 나도 선다 ── */
#define VIDEO_BRAKE_CLEAR_FRAMES  (45)   /* 이 프레임 연속 안전해야 해제(깜빡임 무시) */
static int g_video_brake_active = 0;     /* 영상 감지로 내 차가 정지 중 */
static int g_brake_clear_cnt    = 0;     /* 해제 연속 카운트 */

/* ── Qt 차선변경: 신호 후 일정 시간 뒤 실제로 lane 변경 ── */
#define LANE_CHANGE_DELAY_MS   (3000)    /* 근처에 차 없으면 3초 후 변경 */
#define LANE_CHANGE_YIELD_MS   (5000)    /* 옆차선 뒤에 빠른 차 있으면 5초(양보) */
#define LANE_MIN               (1)       /* 차선 번호 최소 */
#define LANE_MAX               (3)       /* 차선 번호 최대 (3차선) */
#define PEER_STALE_MS          (3000)    /* 이 시간 넘게 소식 없는 차는 판단에서 제외 */
#define MAX_PEER_STATE         (8)

static int      g_lane_change_pending = 0;   /* 차선변경 예약 중 */
static int      g_lane_change_dir     = 0;   /* -1=왼쪽(번호 감소), +1=오른쪽(번호 증가) */
static uint64_t g_lane_change_ms      = 0;   /* 예약 시각 */
static uint32_t g_lane_change_delay   = LANE_CHANGE_DELAY_MS;

/* 다른 차량 최신 상태 보관 (판단용, 로그 안 찍음).
 *   메시지는 1초마다 오는데 버튼은 아무 때나 누르므로, 마지막 상태를 들고 있어야 한다. */
static V2VEventMsg g_peer_last[MAX_PEER_STATE];
static uint64_t    g_peer_last_ms[MAX_PEER_STATE];
static int         g_peer_used[MAX_PEER_STATE];

/* ── Qt SOS 자차 제어: 내 Qt 버튼(내 조작) 반영 ──
 *   내 EMERGENCY_BUTTON_ON  -> 내 speed -20 서행(g_sos_active=1, 한 번만 감속)
 *   내 EMERGENCY_BUTTON_OFF -> 복귀(초기 속도) */
static int g_sos_active = 0;      /* 내 SOS 서행 중 */

/* ── 수신 판단으로 인한 서행(-20) 상태: Qt SOS 받음 / 영상 brake 원거리 ── */
static int g_slow_active = 0;     /* -20 서행 유지 중(수신 판단 결과) */
static int g_slow_pending = 0;    /* 이번에 -20 감속을 적용해야 함(1회성) */
static uint32_t g_last_qt_event = EVT_LANE_CHANGE_LEFT_OFF;  /* 내가 보낼 Qt event(시리얼에서 갱신) */

/* [v8] MOLIT(vehicle_id=100) 수신 - brake/lane을 날씨/혼잡도로 재해석해 저장.
 *   [v9] 이제 죽은 변수 아님 - v2v_max_speed_by_condition()이 매 틱 소비함. */
static WeatherType           g_molit_weather    = WEATHER_SUNNY;
static TrafficCongestionType g_molit_congestion = TRAFFIC_SMOOTH;

/* --- 접근 변화율(레벨) 매핑: 0(변화없음/멀어짐) ~ 10(빠른 접근) ---
 *   raw = (이번넓이-이전넓이)/이전넓이  (음수면 0)
 *   level = raw * APPROACH_LEVEL_SCALE  (0~10 clamp)
 *   예) SCALE=50 이면 프레임당 20% 증가(raw=0.2) -> level 10 */
#define APPROACH_LEVEL_SCALE   (50.0f)   /* 클수록 작은 변화에도 레벨이 빨리 오름 */
#define APPROACH_LEVEL_MAX     (10.0f)   /* 레벨 최대값 */
#define APPROACH_WARN_LEVEL    (7.0f)    /* 이 레벨 이상이면 위험 경고 */

/* 보드2로 보낼 데이터 패킷 (지금은 로그만, 나중에 UDP로 이 구조체를 전송) */
typedef struct {
    uint32_t cam_ch;        /* 카메라 채널 */
    int      track_id;      /* 앞차 추적 ID */
    float    distance_m;    /* 앞차까지 추정 거리(m) */
    float    change_rate;   /* 접근 레벨 0(변화없음/멀어짐) ~ 10(빠른 접근) */
} st_front_car_packet;

/* 앞차 추적용 상태 (프레임 간 비교) */
static int   g_front_track_id  = -1;     /* 직전 프레임의 앞차 track_id */
static float g_front_prev_area = 0.0f;   /* 직전 프레임의 앞차 bbox 넓이 */
static float g_front_prev_dist = 0.0f;   /* 직전 프레임의 앞차 거리(m) */
/* ★★★ [BOARD1 추가 끝] ★★★ */

#ifdef USE_8MP_VI
size_t BUFF_SIZE;
int dsr_fd;
DSR_Data_t dsr_info;
dma_alloc_info dma_info;
st_nc_dsr_config dsr_config;
img_input_config dsr_input_config;
img_output_config dsr_output_config;
#endif

/*
********************************************************************************
*               FUNCTION DEFINITIONS
********************************************************************************
*/
#ifdef USE_8MP_VI
static int dsr_init(void)
{
    int input_fd = 0, output_fd = 0;
    long page_size = sysconf(_SC_PAGESIZE);

    dsr_input_config.format = IMG_FORMAT_RGB888;
    dsr_input_config.width = MAX_WIDTH_FOR_VDMA_CNN_DS;
    dsr_input_config.height = MAX_HEIGHT_FOR_VDMA_CNN_DS;
    dsr_output_config.format = IMG_FORMAT_RGB888;

    BUFF_SIZE = dsr_input_config.width * dsr_input_config.height * 3;
    BUFF_SIZE = ((BUFF_SIZE + page_size - 1) / page_size) * page_size;

    if(dsr_config_downscale(&dsr_config, 1, NPU_INPUT_WIDTH, NPU_INPUT_HEIGHT) < 0){
        perror("Error: DSR config fail");
        return -1;
    }
    if(open_device_and_dma_buffers(DEV_FILE_DSR, &dsr_fd, &input_fd, &output_fd, BUFF_SIZE) < 0){
        perror("Error: DSR open fail");
        return -1;
    }
    if(dsr_setup_buffer(&dsr_info, dsr_fd, &dma_info, input_fd, output_fd, BUFF_SIZE) < 0){
        perror("Error: DSR setup fail");
        return -1;
    }
    return 0;
}

static int dsr_deinit(void)
{
    nc_dmabuf_ctrl_end_cpu_access(dma_info.dmabuf_fd_in);
    nc_dmabuf_ctrl_end_cpu_access(dma_info.dmabuf_fd_out);
    nc_dmabuf_ctrl_free_dma_fd(dma_info.dmabuf_fd_in);
    nc_dmabuf_ctrl_free_dma_fd(dma_info.dmabuf_fd_out);
    nc_dmabuf_ctrl_close();
    for (int i = 0; i < BUFF_NUM; i++) {
        if (dsr_info.dsr_in_buf[i] != MAP_FAILED) munmap(dsr_info.dsr_in_buf[i], BUFF_SIZE);
        if (dsr_info.dsr_out_buf[i] != MAP_FAILED) munmap(dsr_info.dsr_out_buf[i], BUFF_SIZE);
    }
    dsr_device_deinit(&dsr_fd);
    return 0;
}
#endif

void set_viewport_config(void)
{
#if(VIDEO_MAX_CH == 1)
    g_viewport[0].x = 0;
    g_viewport[0].y = 0;
    g_viewport[0].width  = WINDOW_WIDTH;
    g_viewport[0].height = WINDOW_HEIGHT;
#elif(VIDEO_MAX_CH > 1)
    for(int i = 0; i < VIDEO_MAX_CH; i++)
    {
        g_viewport[i].width  = WINDOW_WIDTH/2;
        g_viewport[i].height = WINDOW_HEIGHT/2;
        switch(i)
        {
            case 0: g_viewport[i].x = 0;              g_viewport[i].y = WINDOW_HEIGHT/2; break;
            case 1: g_viewport[i].x = WINDOW_WIDTH/2; g_viewport[i].y = WINDOW_HEIGHT/2; break;
            case 2: g_viewport[i].x = 0;              g_viewport[i].y = 0;               break;
            case 3: g_viewport[i].x = WINDOW_WIDTH/2; g_viewport[i].y = 0;               break;
            default: break;
        }
    }
#endif
}

void set_v4l2_config(void)
{
    int i = 0, j = 0;

    for(i = 0; i < VIS0_MAX_CH; i++) {
#ifdef USE_VIDEO_LOOPBACK
        /* ★ [BOARD1] 실제 카메라 대신 영상(가상카메라 video11)에서 읽기 */
        v4l2_config[i].video_buf.video_device_num = LOOPBACK_DEVICE_BASE + i;
        v4l2_config[i].dma_mode                    = INTERLEAVE;  /* ffmpeg rgb24 = interleaved */
#else
        v4l2_config[i].video_buf.video_device_num = CNN_DEVICE_NUM(VISION0) + i;
  #ifdef USE_8MP_VI
        v4l2_config[i].dma_mode                    = INTERLEAVE;
  #else
        v4l2_config[i].dma_mode                    = PLANAR;
  #endif
#endif
        v4l2_config[i].video_buf.video_fd         = -1;
        v4l2_config[i].img_process                = MODE_DS;
        v4l2_config[i].pixformat                  = V4L2_PIX_FMT_RGB24;
        v4l2_config[i].crop_x_start               = 0;
        v4l2_config[i].crop_y_start               = 0;
        v4l2_config[i].crop_width                 = 0;
        v4l2_config[i].crop_height                = 0;
#ifdef USE_8MP_VI
        v4l2_config[i].ds_width                   = MAX_WIDTH_FOR_VDMA_CNN_DS;
        v4l2_config[i].ds_height                  = MAX_HEIGHT_FOR_VDMA_CNN_DS;
#else
        v4l2_config[i].ds_width                   = VIDEO_WIDTH;
        v4l2_config[i].ds_height                  = VIDEO_HEIGHT;
#endif
    }

    for(j = VIS0_MAX_CH; j < VIDEO_MAX_CH; j++) {
#ifdef USE_VIDEO_LOOPBACK
        v4l2_config[j].video_buf.video_device_num = LOOPBACK_DEVICE_BASE + j;
        v4l2_config[j].dma_mode                    = INTERLEAVE;
#else
        v4l2_config[j].video_buf.video_device_num = CNN_DEVICE_NUM(VISION1) + j;
  #ifdef USE_8MP_VI
        v4l2_config[j].dma_mode                    = INTERLEAVE;
  #else
        v4l2_config[j].dma_mode                    = PLANAR;
  #endif
#endif
        v4l2_config[j].video_buf.video_fd         = -1;
        v4l2_config[j].img_process                = MODE_DS;
        v4l2_config[j].pixformat                  = V4L2_PIX_FMT_RGB24;
        v4l2_config[j].crop_x_start               = 0;
        v4l2_config[j].crop_y_start               = 0;
        v4l2_config[j].crop_width                 = 0;
        v4l2_config[j].crop_height                = 0;
#ifdef USE_8MP_VI
        v4l2_config[j].ds_width                   = MAX_WIDTH_FOR_VDMA_CNN_DS;
        v4l2_config[j].ds_height                  = MAX_HEIGHT_FOR_VDMA_CNN_DS;
#else
        v4l2_config[j].ds_width                   = VIDEO_WIDTH;
        v4l2_config[j].ds_height                  = VIDEO_HEIGHT;
#endif
    }
}

int send_cnn_buf (uint8_t *ptr_cnn_buf, uint64_t time_stamp_us, uint32_t cam_ch, E_NETWORK_UID net_id)
{
    int ret = 0;
    stCnnData *cnn_data;
    struct mq_attr attr;
    attr.mq_maxmsg = MAX_MQ_MSG_CNT;
    attr.mq_msgsize = sizeof(stCnnData*);
    int oflag = O_WRONLY | O_CREAT;
    mqd_t mfd = mq_open(MQ_NAME_CNN_BUF, oflag, 0666, &attr);
    if (mfd == -1) {
        perror("mq open error");
        return -1;
    }

    cnn_data = (stCnnData*)malloc(sizeof(stCnnData));
    cnn_data->cam_ch = cam_ch;
    cnn_data->ptr_cnn_buf = ptr_cnn_buf;
    cnn_data->time_stamp_us = time_stamp_us;
    cnn_data->net_id = net_id;

    if ((ret = mq_send(mfd, (const char *)&cnn_data, attr.mq_msgsize, 1)) == -1) {
        printf("errno of mq_send = %d\n", errno);
    }
    mq_close(mfd);
    return ret;
}

int receive_cnn_buf (stCnnData **out_cnn_buf)
{
    int ret = 0;
    struct mq_attr attr;
    attr.mq_maxmsg = MAX_MQ_MSG_CNT;
    attr.mq_msgsize = sizeof(stCnnData *);
    int oflag = O_RDONLY | O_CREAT;
    mqd_t mfd = mq_open(MQ_NAME_CNN_BUF, oflag, 0666, &attr);
    if (mfd == -1) {
        perror("mq open error");
        return -1;
    }

    if ((ret = (int32_t)mq_receive(mfd, (char*)out_cnn_buf, attr.mq_msgsize, NULL)) == -1) {
        printf("errno of mq_receive = %d\n", errno);
    }
    mq_close(mfd);
    return ret;
}

int v4l2_initialize(void)
{
    for(int i = 0; i < VIDEO_MAX_CH; i++)
    {
        v4l2_config[i].video_buf.video_fd = nc_v4l2_open(v4l2_config[i].video_buf.video_device_num, true);
        if(v4l2_config[i].video_buf.video_fd == errno) {
            printf("[error] nc_v4l2_open() failure!\n");
        } else {
            if(nc_v4l2_init_device_and_stream_on(&v4l2_config[i], VIDEO_BUFFER_NUM) < 0) {
                printf("[error] nc_v4l2_init_device_and_stream_on() failure!\n");
                return -1;
            }
        }
    }
    nc_v4l2_show_user_config(&v4l2_config[0], VIDEO_MAX_CH);
    return 0;
}

/* ★★★ [BOARD1 추가 시작] 앞차 거리/접근 변화율 분석 함수 ★★★ */

/* SDK 단조시간(ms) - nc_utils.c (v2v 판단 유지시간 계산에 사용) */
extern uint64_t nc_get_mono_time(void);

/* ── 다른 차량 상태 저장 (조용히, 로그 없음) ── */
static void save_peer_state(const V2VEventMsg *ev)
{
    if (ev->vehicle_id == g_my_vehicle_id) return;
    if (ev->vehicle_id == V2V_MOLIT_VEHICLE_ID) return;   /* 국토교통부는 차량 아님 */

    int free_idx = -1;
    for (int i = 0; i < MAX_PEER_STATE; i++) {
        if (g_peer_used[i] && g_peer_last[i].vehicle_id == ev->vehicle_id) {
            g_peer_last[i]    = *ev;
            g_peer_last_ms[i] = nc_get_mono_time();
            return;
        }
        if (!g_peer_used[i] && free_idx < 0) free_idx = i;
    }
    if (free_idx >= 0) {
        g_peer_last[free_idx]    = *ev;
        g_peer_last_ms[free_idx] = nc_get_mono_time();
        g_peer_used[free_idx]    = 1;
    }
}

/* ── 차선변경 시 양보가 필요한가? ──
 *   옆차선 + 내 뒤 LANE_CHANGE_NEAR_M 이내 + 나보다 빠른 차가 있으면 1(5초 대기). */
static int lane_change_need_yield(void)
{
    uint64_t now = nc_get_mono_time();

    for (int i = 0; i < MAX_PEER_STATE; i++) {
        if (!g_peer_used[i]) continue;
        if (now - g_peer_last_ms[i] > PEER_STALE_MS) continue;   /* 오래된 정보 무시 */

        const V2VEventMsg *p = &g_peer_last[i];

        int lane_diff = p->lane - g_my_lane;
        if (lane_diff < 0) lane_diff = -lane_diff;
        if (lane_diff != 1) continue;                 /* 옆차선 아니면 무관 */

        float behind = g_my_position - p->position;   /* +면 상대가 내 뒤 */
        if (behind <= 0.0f || behind > LANE_CHANGE_NEAR_M) continue;

        if (p->speed > g_my_speed_final) return 1;    /* 나보다 빠름 -> 양보 [v9] 실제 속도끼리 비교 */
    }
    return 0;
}

/* ★★★ [V2V 판단] 수신 이벤트 대응 - 판단은 전부 여기(wayland) ★★★
 *   급정지: 송신자 position vs 내 position 으로 앞/뒤·거리 판단
 *   차선변경: 옆차선 + 근접 + speed 비교 -> wait / 무시
 *   결과: g_v2v_action 에 저장 (render가 반영)
 */
static void nc_v2v_decide(const V2VEventMsg *ev)
{
    /* 자기 자신이 보낸 것(브로드캐스트 반향)은 무시 */
    if (ev->vehicle_id == g_my_vehicle_id) return;

    /* [v8] 국토교통부(vehicle_id=100) - brake/lane 의미가 완전히 다르므로
     *   일반 차량 판단 로직으로 절대 진입시키지 않는다. 재해석해서 저장만(v9부터 소비됨). */
    if (ev->vehicle_id == V2V_MOLIT_VEHICLE_ID) {
        g_molit_weather    = (WeatherType)ev->brake;
        g_molit_congestion = (TrafficCongestionType)ev->lane;
        return;
    }

    uint32_t etype = ev->event_type;

    /* ── (1) 급정지 판단: 송신자 position vs 내 position ──
     *   내가 송신자보다 앞(내 position 큼) -> 무시(속도 유지)
     *   내가 뒤 + 멀다(100m 초과)         -> 서서히 감속
     *   내가 뒤 + 가깝다                  -> 즉시 정지 */
    /* ── (1) Qt SOS 판단: event_type = EMERGENCY_BUTTON_ON ──
     *   거리 무관. 송신자보다 내가 뒤 + 같은 lane 이면 -20 서행. 앞/다른lane 무시.
     *   (정지 아님. "앞에 비상상황이니 서행하며 조심"의 의미) */
    if (etype == EVT_EMERGENCY_BUTTON_ON) {
        float gap = g_my_position - ev->position;   /* +면 내가 앞, -면 내가 뒤 */
        if (gap < 0.0f && ev->lane == g_my_lane) {
            g_slow_active   = 1;                    /* -20 서행 진입 */
            g_slow_pending  = 1;                    /* 1회 감속 예약 */
            g_v2v_action    = V2V_ACT_SLOW_CRUISE;
            g_v2v_action_ms = nc_get_mono_time();
            printf("[V2V-DECIDE] from id=%u | Qt SOS | I'm behind & same lane -> SLOW CRUISE (-20)\n",
                   ev->vehicle_id);
        } else {
            printf("[V2V-DECIDE] from id=%u | Qt SOS | ahead or diff lane -> ignore\n",
                   ev->vehicle_id);
        }
        return;
    }
    /* Qt SOS 해제: 서행 풀고 복귀 */
    if (etype == EVT_EMERGENCY_BUTTON_OFF) {
        if (g_slow_active) {
            g_slow_active  = 0;
            g_slow_pending = 0;
            g_my_speed     = g_my_speed_init;      /* 복귀 */
            g_v2v_action   = V2V_ACT_NONE;
            printf("[V2V-DECIDE] from id=%u | Qt SOS OFF -> resume (speed back)\n",
                   ev->vehicle_id);
        }
        return;
    }

    /* ── (2) 영상 brake 판단: ev->brake == 1 ──
     *   같은 lane + 20m 이내(가까움) -> 완전정지(쏜차 -5m).
     *   같은 lane + 먼 거리          -> -20 서행(계속 주행, 정지 계획 없음).
     *   내가 앞 / 다른 lane          -> 무시. */
    if (ev->brake == 1) {
        float gap = g_my_position - ev->position;   /* +면 내가 앞 */
        if (gap >= 0.0f || ev->lane != g_my_lane) {
            /* 내가 앞 or 다른 lane -> 무시 (내 정지 상태는 건드리지 않는다:
             *   무관한 차 때문에 진행 중인 정지가 풀리면 안 됨) */
            printf("[V2V-DECIDE] from id=%u | VIDEO-BRAKE | ahead or diff lane -> ignore\n",
                   ev->vehicle_id);
        } else {
            float behind = -gap;   /* 내가 뒤로 떨어진 거리 */

            if (behind > V2V_BRAKE_FAR_IGNORE_M) {
                /* ★ 너무 멀면 반응하지 않음 (100m 초과) */
                printf("[V2V-DECIDE] from id=%u | VIDEO-BRAKE | too far(%.1fm) -> ignore\n",
                       ev->vehicle_id, behind);
            } else if (behind <= V2V_BRAKE_NEAR_M) {
                /* 가까움 -> 완전정지 (쏜차 -5m). 이미 정지 중이면 갱신만 하고 로그 생략 */
                if (!g_stop_active) {
                    printf("[V2V-DECIDE] from id=%u | VIDEO-BRAKE | near(%.1fm) same lane "
                           "-> STOP at %.1f\n", ev->vehicle_id, behind,
                           ev->position - STOP_GAP_M);
                }
                g_stop_target_pos = ev->position - STOP_GAP_M;
                g_stop_active     = 1;
                g_slow_active     = 0;                  /* 정지가 서행보다 우선 */
                g_slow_pending    = 0;
                g_v2v_action      = V2V_ACT_STOP;
                g_v2v_action_ms   = nc_get_mono_time();
            } else {
                /* 멀다 -> -20 서행. ★ 이미 서행 중이거나 정지 중이면 재적용하지 않음
                 *   (상대가 brake 를 계속 보내도 속도가 계속 깎이면 안 됨) */
                if (!g_slow_active && !g_stop_active) {
                    g_slow_active   = 1;
                    g_slow_pending  = 1;            /* 1회 감속 예약 */
                    g_v2v_action    = V2V_ACT_SLOW_CRUISE;
                    g_v2v_action_ms = nc_get_mono_time();
                    printf("[V2V-DECIDE] from id=%u | VIDEO-BRAKE | far(%.1fm) same lane "
                           "-> SLOW CRUISE (-20)\n", ev->vehicle_id, behind);
                }
            }
        }
        return;
    }

    /* ── (2) 차선변경 판단: 옆차선 + 근접(10m) + speed 비교 ──
     *   송신자가 내 옆차선(lane 차이 1)이고, position 차이 10m 이내(근접)일 때만 반응.
     *   상대가 나보다 빠르면 -> 내가 wait(양보) 후 진행.  (wait 로그는 "나"에게만 찍힘)
     *   내가 더 빠르거나 멀면 -> 무시(내가 먼저/무관). */
    if (etype == EVT_LANE_CHANGE_LEFT_ON || etype == EVT_LANE_CHANGE_RIGHT_ON) {
        int   lane_diff = ev->lane - g_my_lane; if (lane_diff < 0) lane_diff = -lane_diff;
        float pos_gap   = ev->position - g_my_position; if (pos_gap < 0.0f) pos_gap = -pos_gap;

        if (lane_diff == 1 && pos_gap <= LANE_CHANGE_NEAR_M) {
            if (ev->speed > g_my_speed_final) {   /* [v9] 실제 속도끼리 비교 */
                /* 상대가 빠름 -> 내가 양보(wait) */
                g_v2v_action    = V2V_ACT_WAIT;
                g_v2v_action_ms = nc_get_mono_time();
                printf("[V2V-DECIDE] from id=%u | LANE_CHANGE | side+near, peer faster "
                       "(peer_spd=%.1f > my_spd=%.1f) -> WAIT %dms then proceed\n",
                       ev->vehicle_id, ev->speed, g_my_speed_final, V2V_WAIT_MS);
            } else {
                printf("[V2V-DECIDE] from id=%u | LANE_CHANGE | side+near but I'm faster/equal "
                       "-> keep going (no yield)\n", ev->vehicle_id);
            }
        } else {
            printf("[V2V-DECIDE] from id=%u | LANE_CHANGE | not side/near "
                   "(lane_diff=%d pos_gap=%.1f) -> ignore\n",
                   ev->vehicle_id, lane_diff, pos_gap);
        }
        return;
    }

    /* ★ 상대 brake 가 풀렸을 때(brake==0) 복구:
     *   그 상대 때문에 서행/정지 중이었다면, 위험이 사라졌으므로 원래 속도로 돌아간다.
     *   (같은 lane + 앞차인 경우만 - 무관한 차 신호로 복구되면 안 됨) */
    if (ev->brake == 0 && ev->lane == g_my_lane) {
        float gap = g_my_position - ev->position;
        if (gap < 0.0f && (g_slow_active || g_stop_active)) {   /* 상대가 내 앞 */
            g_slow_active  = 0;
            g_slow_pending = 0;
            g_stop_active  = 0;
            g_my_speed     = g_my_speed_init;
            g_v2v_action   = V2V_ACT_NONE;
            printf("[V2V-DECIDE] from id=%u | brake cleared -> resume speed=%.1f\n",
                   ev->vehicle_id, g_my_speed);
        }
    }

    /* 그 외(NONE, LANE_*_OFF 등)는 무시 */
}

/* ★★★ [v9] 기상/혼잡도 -> 최대 허용 속도 ★★★
 *   cap = g_my_speed_init(내 기준 속도) x 날씨계수 x 혼잡도계수 - 비율 곱셈.
 *   나쁜 조건이 겹칠수록 자연히 더 강하게 제한됨(덧셈 방식과 달리 역설 없음).
 *   계수는 도로교통법 시행규칙의 악천후 감속 기준(노면 결빙·폭우 시 50% 감속 등)을 참고. */
static float v2v_max_speed_by_condition(WeatherType weather, TrafficCongestionType congestion)
{
    static const float weather_factor[4]    = { 1.0f, 0.9f, 0.6f, 0.3f };  /* SUNNY/NIGHT/RAIN/SNOW */
    static const float congestion_factor[4] = { 1.0f, 0.9f, 0.6f, 0.3f };  /* SMOOTH/NORMAL/HEAVY/JAM */

    int wi = (int)weather;
    int ci = (int)congestion;
    if (wi < 0 || wi > 3) wi = 0;
    if (ci < 0 || ci > 3) ci = 0;

    return g_my_speed_init * weather_factor[wi] * congestion_factor[ci];
}

/* [제거됨] UDP 모니터링 전송은 V2V(TCP)로 일원화하여 사용 안 함.
 *   필요 시 아래 두 함수와 관련 전역(g_udp_*)을 복원.
 * static void nc_udp_init(void) { ... }
 * static void nc_udp_send(float distance_m, float level) { ... }
 */

/* 클래스명이 차량류인지 (모델 클래스명에 맞게 필요시 수정) */
static int is_vehicle_class(const char* name)
{
    if (!name) return 0;
    return (strcmp(name, "car") == 0 ||
            strcmp(name, "bus") == 0 ||
            strcmp(name, "truck") == 0 ||
            strcmp(name, "motorcycle") == 0);
}

/* 한 채널의 검출 결과에서 "앞차"를 골라 거리/변화율을 계산하고 로그로 출력.
 * 앞차 = 차량류 중 화면에서 bbox 넓이가 가장 큰 것(가장 가까운 차). */
static void nc_board1_front_car_analyze(uint32_t cam_ch, pp_result_buf *det_buf)
{
    if (!det_buf) return;
    stCnnPostprocessingResults *r = &det_buf->cnn_result;
    stObjDrawInfo *d = &det_buf->draw_info;

    /* 1) 앞차 선택: "가장 중앙에 있는 것"(화면 가로 중심 0.5에 bbox 중심이 제일 가까운 차)
     *   - 크기/거리 무관, 오직 중앙성. 멀어서 작아도 정중앙이면 내 차선 앞차로 본다.
     *   - 옆에 가까운(큰) 차가 있어도 중앙이 아니면 무시. */
    float best_center_dist = 999.0f;  /* |cx_ratio - 0.5| 최소값 찾기 */
    int   found = 0;
    stObjInfo front;
    int   front_tid = -1;
    float best_area = 0.0f;           /* 선택된 차의 넓이(level 계산용) */

    for (int i = 0; i < d->max_class_cnt; i++) {
        if (!is_vehicle_class(d->class_names[i])) continue;
        for (int b = 0; b < r->class_objs[i].obj_cnt; b++) {
            stObjInfo o = r->class_objs[i].objs[b];

            /* bbox 가로 중심이 화면 중앙부(30~70%) 안에 있는 것만 후보 */
            float cx = (float)o.bbox.x + (float)o.bbox.w * 0.5f;
            float cx_ratio = cx / (float)WINDOW_WIDTH;
            if (cx_ratio < LANE_CENTER_MIN_RATIO || cx_ratio > LANE_CENTER_MAX_RATIO)
                continue;

            /* ★ 중앙성: |cx - 0.5| 가 가장 작은 것 선택 (크기 아님) */
            float center_dist = cx_ratio - 0.5f;
            if (center_dist < 0.0f) center_dist = -center_dist;

            if (center_dist < best_center_dist) {
                best_center_dist = center_dist;
                front = o;
                found = 1;
                best_area = (float)o.bbox.w * (float)o.bbox.h;
#ifdef USE_BYTETRACK
                front_tid = o.track_id;
#endif
            }
        }
    }

    /* 앞차 없음 -> 상태 리셋 */
    if (!found) {
        g_front_track_id = -1;
        g_front_prev_area = 0.0f;
        g_front_prev_dist = 0.0f;
        g_out_front_distance = 999.0f;   /* 앞 비었음 */
        g_out_approach_warn  = 0;
        g_warn_latched       = 0;        /* 앞차 없음 -> latch 해제 */
        if (g_video_brake_active) {
            if (++g_brake_clear_cnt >= VIDEO_BRAKE_CLEAR_FRAMES) {
                g_video_brake_active = 0;
                g_brake_clear_cnt    = 0;
                g_stop_active        = 0;
                g_my_speed           = g_my_speed_init;
                g_v2v_action         = V2V_ACT_NONE;
            }
        }
        return;
    }

    /* 2) 거리 추정: bbox 높이가 클수록 가까움
     *    dist = FOCAL_PX * REAL_CAR_HEIGHT_M / bbox_height_px  */
    float bbox_h = (float)front.bbox.h;
    float dist = (bbox_h > 1.0f) ? (CAM_FOCAL_PX * REAL_CAR_HEIGHT_M / bbox_h) : 999.0f;

    /* 3) 접근 레벨(0~10) 계산
     *    - track_id가 없을 때(-1) "가장 큰 차"가 프레임마다 바뀌면 거리가 튄다.
     *    - 그래서 이전 거리와 "비슷할 때만"(같은 앞차로 간주) 변화율을 계산하고,
     *      거리가 급변하면(다른 차로 바뀜) level=0 으로 처리한다. */
    float rate = 0.0f;   /* 접근 레벨 0~10 */
    if (g_front_prev_area > 1.0f && g_front_prev_dist > 0.1f) {
        /* 이전 거리 대비 이번 거리 비율: 0.5~2.0 범위 안이면 "같은 차"로 간주 */
        float dist_ratio = dist / g_front_prev_dist;
        if (dist_ratio > 0.5f && dist_ratio < 2.0f) {
            /* 넓이 증가율 -> 0~10 레벨 */
            float raw = (best_area - g_front_prev_area) / g_front_prev_area;
            if (raw < 0.0f) raw = 0.0f;                        /* 멀어지면 0 */
            rate = raw * APPROACH_LEVEL_SCALE;
            if (rate > APPROACH_LEVEL_MAX) rate = APPROACH_LEVEL_MAX;
        }
        /* dist_ratio가 벗어나면(다른 차로 점프) rate = 0 유지 */
    }

    /* 거리 평활화: 이전 거리와 섞어 급격한 튐 완화 (같은 차로 간주될 때만) */
    if (g_front_prev_dist > 0.1f && dist/g_front_prev_dist > 0.5f && dist/g_front_prev_dist < 2.0f) {
        dist = dist * 0.5f + g_front_prev_dist * 0.5f;   /* 이전값과 반반 평균 */
    }

    /* 상태 갱신 (다음 프레임 비교용) */
    g_front_track_id  = front_tid;
    g_front_prev_area = best_area;
    g_front_prev_dist = dist;

    /* ★ [출력] 앞차 거리 저장 */
    g_out_front_distance = dist;

    /* ★ [warn latch - 초기 모델] 거리 기준으로 켜고/유지/해제
     *   - 켜기: dist < WARN_DISTANCE_M(7m) AND level >= WARN_MIN_LEVEL(1)
     *   - 유지: warn=1 된 뒤 dist < 7m 인 동안 계속 1 (더 가까워지면 당연히 유지)
     *   - 해제: dist > 7m (앞차가 멀어지면) */
    if (dist < WARN_DISTANCE_M && rate >= WARN_MIN_LEVEL) {
        g_warn_latched = 1;                    /* 켜기(가깝고 + 다가옴) */
    } else if (g_warn_latched && dist > WARN_DISTANCE_M) {
        g_warn_latched = 0;                    /* 해제(7m 초과로 멀어짐) */
    }
    /* 유지: latch 상태이고 아직 7m 이내면 계속 1 (앞차 멈춰 level 0이어도) */
    g_out_approach_warn = g_warn_latched;

    /* ★ [자차 제어] 내 카메라가 앞차 급정지를 감지하면 나도 정지한다.
     *   - warn 진입 순간 목표정지 활성화(현재 위치에서 정지)
     *   - 해제는 VIDEO_BRAKE_CLEAR_FRAMES 연속 안전할 때만(깜빡임으로 속도가 도로
     *     올라가는 것 방지) */
    if (g_warn_latched && !g_video_brake_active) {
        g_video_brake_active = 1;
        g_brake_clear_cnt    = 0;
        g_stop_target_pos    = g_my_position;   /* 더 나아가지 않고 여기서 정지 */
        g_stop_active        = 1;
        g_slow_active        = 0;
        g_v2v_action          = V2V_ACT_STOP;
        g_v2v_action_ms      = nc_get_mono_time();
    } else if (g_video_brake_active) {
        if (g_warn_latched) {
            g_brake_clear_cnt = 0;                       /* 다시 위험 -> 카운트 리셋 */
        } else if (++g_brake_clear_cnt >= VIDEO_BRAKE_CLEAR_FRAMES) {
            g_video_brake_active = 0;
            g_brake_clear_cnt    = 0;
            g_stop_active        = 0;
            g_my_speed           = g_my_speed_init;      /* 위험 해제 -> 주행 복귀 */
            g_v2v_action         = V2V_ACT_NONE;
        }
    }

    /* 4) 보드2로 보낼 패킷 구성 */
    st_front_car_packet pkt;
    pkt.cam_ch      = cam_ch;
    pkt.track_id    = front_tid;
    pkt.distance_m  = dist;
    pkt.change_rate = rate;

    /* (영상 위험 판정은 warn latch(g_out_approach_warn)로 처리됨.
     *  danger_streak 기반 별도 급정지 송신은 제거 - brake 필드로 주기 송신에 실음) */

    /* 5) [전용 로그] 거리/level 상세 - 평소엔 화면 도배 방지로 주석 처리.
     *    디버깅 시 아래 주석을 풀면 매 N프레임 거리/level/경고를 볼 수 있음.
     *    (평소 화면에는 v2v_comm.c 의 SENT/RECV 로그만 보이게 함) */
    // static int log_cnt = 0;
    // if (++log_cnt % LOG_EVERY_N_FRAMES == 0) {
    //     printf("[board1 cam%u] front id%-3d | dist=%5.1fm | level=%4.1f\n",
    //            pkt.cam_ch, pkt.track_id, pkt.distance_m, pkt.change_rate);
    //     if (danger_streak >= WARN_HOLD_FRAMES) {
    //         printf("[board1 cam%u] !!! WARNING: front car too close / fast approach "
    //                "(dist=%.1fm level=%.1f) !!!\n", pkt.cam_ch, dist, rate);
    //     } else if (dist < WARN_DISTANCE_M && rate >= WARN_MIN_LEVEL) {
    //         printf("[board1 cam%u] (caution) front car approaching within %.1fm (level=%.1f)\n",
    //                pkt.cam_ch, dist, rate);
    //     }
    // }

    /* 6) 영상 warning 은 주기 송신(1초)에서 brake 필드로 실어 보낸다.
     *    (여기서 별도 즉시 송신하지 않음 - event_type 은 Qt 전용, brake 는 영상 전용) */

    (void)pkt;
}
/* ★★★ [BOARD1 추가 끝] ★★★ */

void nc_draw_gl_npu(struct viewport viewport, int network_task, pp_result_buf *net_result, struct gl_npu_program g_npu_prog)
{
    stCnnPostprocessingResults *det_result = &net_result->cnn_result;
    stObjDrawInfo *draw_cnn = &net_result->draw_info;
    stSegDrawInfo *draw_seg = &net_result->seg_info;
    stLaneDrawInfo *draw_lane = &net_result->lane_draw_info;

    int max_class_cnt = draw_cnn->max_class_cnt;
    int max_seg_class_cnt = draw_seg->max_class_cnt;
    float target_view_ratio = 1.f;
    char buftext[128];

    target_view_ratio = (float)WINDOW_HEIGHT / (float)viewport.height;
    glViewport(viewport.x, viewport.y, viewport.width, viewport.height);

    if(network_task == DETECTION)
    {
        float** color = (float**)malloc(max_class_cnt * sizeof(float*));
        for (int i = 0; i < max_class_cnt; ++i) {
            color[i] = (float*)malloc(RGBA_CNT * sizeof(float));
        }
        for (int j = 0; j < max_class_cnt; ++j) {
            color[j][RGBA_R] = (float)(draw_cnn->class_colors[j].r) / 255.0f;
            color[j][RGBA_G] = (float)(draw_cnn->class_colors[j].g) / 255.0f;
            color[j][RGBA_B] = (float)(draw_cnn->class_colors[j].b) / 255.0f;
            color[j][RGBA_A] = 1.0f;
        }

        for(int i=0; i<draw_cnn->max_class_cnt; i++){
            for(int bidx = 0; bidx < det_result->class_objs[i].obj_cnt; bidx++) {
                stObjInfo obj_info = det_result->class_objs[i].objs[bidx];
                nc_opengl_draw_rectangle(obj_info.bbox.x, obj_info.bbox.y, obj_info.bbox.w, obj_info.bbox.h, color[i], g_npu_prog);
#ifdef USE_BYTETRACK
                if (obj_info.track_id < 0) sprintf(buftext, "%s:%0.2f", draw_cnn->class_names[i], obj_info.prob);
                else sprintf(buftext, "[%d]%s:%0.2f", obj_info.track_id, draw_cnn->class_names[i], obj_info.prob);
#else
                sprintf(buftext, "%s:%0.2f", draw_cnn->class_names[i], obj_info.prob);
#endif
                float textcolor[3] = {color[i][RGBA_R], color[i][RGBA_G], color[i][RGBA_B]};
                nc_opengl_draw_text(&font_24, buftext, obj_info.bbox.x, (float)WINDOW_HEIGHT - (obj_info.bbox.y-12), target_view_ratio, textcolor, WINDOW_WIDTH, WINDOW_HEIGHT, g_font_prog);
            }
        }

        for (int i = 0; i < max_class_cnt; ++i) free(color[i]);
        free(color);
    }
    else if(network_task == SEGMENTATION)
    {
        glBindTexture(GL_TEXTURE_2D, g_seg_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, draw_seg->width, draw_seg->height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, det_result->seg);

        float color[COLOR_PALETTE_CNT][RGBA_CNT];
        for (int j = 0; j < max_seg_class_cnt; ++j) {
            color[j][RGBA_R] = (float)(draw_seg->class_colors[j].r) / 255.0f;
            color[j][RGBA_G] = (float)(draw_seg->class_colors[j].g) / 255.0f;
            color[j][RGBA_B] = (float)(draw_seg->class_colors[j].b) / 255.0f;
            color[j][RGBA_A] = (float)(draw_seg->class_colors[j].a) / 255.0f;
        }
        nc_opengl_draw_segmentation(g_seg_texture, (float**)color, draw_seg->max_class_cnt, g_npu_prog);
    }
    else if(network_task == LANE)
    {
        float color[COLOR_PALETTE_CNT][RGBA_CNT];
        for (int j = 0; j < draw_lane->max_lane_num; ++j) {
            color[j][RGBA_R] = (float)(draw_lane->index_colors[j].r) / 255.0f;
            color[j][RGBA_G] = (float)(draw_lane->index_colors[j].g) / 255.0f;
            color[j][RGBA_B] = (float)(draw_lane->index_colors[j].b) / 255.0f;
            color[j][RGBA_A] = 1.0f;
        }

        for(int i = 0; i < draw_lane->max_lane_num; i++)
        {
            int point_num = det_result->lane_det[i].point_cnt;
            int lane_class = det_result->lane_det[i].lane_class;
            if(point_num == 0) continue;
            for(int j=0; j<point_num-1; j++)
            {
                float st_x = det_result->lane_det[i].point[j].x;
                float st_y = det_result->lane_det[i].point[j].y;
                float end_x = det_result->lane_det[i].point[j+1].x;
                float end_y = det_result->lane_det[i].point[j+1].y;
                nc_opengl_draw_line(st_x, st_y, end_x, end_y, lane_class, color[i], g_npu_prog);
            }
        }
    }
    else
    {
        // invalid network
    }
}

void gl_initialize(struct window *window)
{
    int width, height, channels;
    char buf[128];

    sprintf(buf, "misc/image/nextchip_s.png");
    image_data = SOIL_load_image(buf, &width, &height, &channels, SOIL_LOAD_RGBA);

    nc_opengl_init_video_shader(window, 0);

    for(int i=0;i<VIDEO_MAX_CH;i++)
    {
        glGenTextures(1, &window->gl.texture[i]);
        glBindTexture(GL_TEXTURE_2D, window->gl.texture[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }

    nc_opengl_load_font("misc/font/NotoSans-Regular.ttf", 38, &font_38);
    nc_opengl_load_font("misc/font/NotoSans-Regular.ttf", 24, &font_24);
    nc_opengl_init_font_shader(&g_font_prog);
    nc_opengl_init_npu_shader(&g_npu_prog);

    glGenTextures(1, &g_seg_texture);
    glBindTexture(GL_TEXTURE_2D, g_seg_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
}

void render(void *data, struct wl_callback *callback, uint32_t time)
{
    static struct timespec begin, end, set_time;
    static uint64_t fpstime = 0;
    static uint64_t fpscount = 0;
    static uint64_t fpscount_00 = 0;
    static uint64_t frametime = 0;
    static uint64_t opengl_time = 0;
    static uint64_t framecnt = 0;
    int networkOrder[VIDEO_MAX_CH];

    (void)time;

    clock_gettime(CLOCK_MONOTONIC, &begin);
    struct window *window = (struct window *)data;

    assert(window->callback == callback);
    window->callback = NULL;
    if (callback) wl_callback_destroy(callback);
    if (!window->configured) return;

    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for(int i = 0;i<VIDEO_MAX_CH;i++)
    {
        if (v4l2_config[i].video_buf.video_fd == -1)
        {
            glBindTexture(GL_TEXTURE_2D, window->gl.texture[i]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, VIDEO_WIDTH, VIDEO_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
        }else
        {
            struct v4l2_buffer video_buf;
            CLEAR(video_buf);
            if (nc_v4l2_dequeue_buffer(v4l2_config[i].video_buf.video_fd, &video_buf) == -1) {
                //printf("Error VIDIOC_DQBUF buffer %d\n", video_buf.index);
            }else
            {
#if (VIDEO_MAX_CH == 4)
                if (((framecnt % 2 == 0) && (i == 0 || i == 2)) ||
                    ((framecnt % 2 == 1) && (i == 1 || i == 3)))
#endif
                {
                    uint64_t time_stamp_us = 0;
                    networkOrder[i] = nc_get_cnn_networks_id();
                    unsigned char* rgbdata_for_cnn = (unsigned char *)malloc(npu_input_info.rgb_size);
#ifdef USE_8MP_VI
                    memcpy(dsr_info.dsr_in_buf[0], (uint8_t *)v4l2_config[i].video_buf.buffers[video_buf.index].start, MAX_WIDTH_FOR_VDMA_CNN_DS*MAX_HEIGHT_FOR_VDMA_CNN_DS*RGB_CNT);
                    dsr_downscale(dsr_fd, &dsr_info, dsr_input_config, dsr_output_config, dsr_config, 0);
                    nc_rgb_interleaved_to_planar_neon((unsigned char *)dsr_info.dsr_out_buf[0],
                                                    (unsigned char *)rgbdata_for_cnn,
                                                    (unsigned char *)rgbdata_for_cnn + NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT,
                                                    (unsigned char *)rgbdata_for_cnn + NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT*2,
                                                    NPU_INPUT_WIDTH,  NPU_INPUT_HEIGHT);
#elif defined(USE_VIDEO_LOOPBACK)
                    /* ★ [BOARD1] 영상은 interleaved(rgb24) -> NPU용 planar 로 변환 */
                    nc_rgb_interleaved_to_planar_neon(
                        (unsigned char *)v4l2_config[i].video_buf.buffers[video_buf.index].start,
                        (unsigned char *)rgbdata_for_cnn,
                        (unsigned char *)rgbdata_for_cnn + NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT,
                        (unsigned char *)rgbdata_for_cnn + NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT*2,
                        NPU_INPUT_WIDTH, NPU_INPUT_HEIGHT);
#else
                    memcpy(rgbdata_for_cnn, (unsigned char *)v4l2_config[i].video_buf.buffers[video_buf.index].start, npu_input_info.rgb_size);
#endif
                    send_cnn_buf (rgbdata_for_cnn, time_stamp_us, (uint32_t)i, (E_NETWORK_UID)networkOrder[i]);
                }
#if !defined(USE_8MP_VI) && !defined(USE_VIDEO_LOOPBACK)
                unsigned char *interleaved_rgb = (unsigned char *)malloc(NPU_INPUT_WIDTH*NPU_INPUT_HEIGHT*3);
                nc_rgb_planar_to_interleaved_neon((uint8_t*)v4l2_config[i].video_buf.buffers[video_buf.index].start,
                                            (uint8_t*)v4l2_config[i].video_buf.buffers[video_buf.index].start + NPU_INPUT_DATA_SIZE,
                                            (uint8_t*)v4l2_config[i].video_buf.buffers[video_buf.index].start + (NPU_INPUT_DATA_SIZE*2),
                                            interleaved_rgb, NPU_INPUT_WIDTH, NPU_INPUT_HEIGHT);
#endif
                /* (날씨 판정 제거 - 보드끼리 주고받지 않음) */

                if(nc_v4l2_queue_buffer(v4l2_config[i].video_buf.video_fd, video_buf.index) == -1) {
                    printf("Error VIDIOC_QBUF buffer %d\n", video_buf.index);
                }

                glBindTexture(GL_TEXTURE_2D, window->gl.texture[i]);
#if defined(USE_8MP_VI) || defined(USE_VIDEO_LOOPBACK)
                /* 8MP/영상: 원본이 이미 interleaved -> 바로 텍스처 */
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, VIDEO_WIDTH, VIDEO_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, v4l2_config[i].video_buf.buffers[video_buf.index].start);
#else
                /* 실제 카메라(planar): 변환한 interleaved 버퍼로 텍스처 */
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, VIDEO_WIDTH, VIDEO_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, interleaved_rgb);
                if(interleaved_rgb){
                    free(interleaved_rgb);
                }
#endif
            }
        }

        glViewport(g_viewport[i].x, g_viewport[i].y, g_viewport[i].width, g_viewport[i].height);
        nc_opengl_draw_texture(window->gl.texture[i], window);
    }

#ifdef USE_ADAS_LD
    draw_ld_output_opengl();
#endif

    for(uint32_t ch = 0; ch < VIDEO_MAX_CH; ch++)
    {
        if (v4l2_config[ch].video_buf.video_fd == -1){
        }
        else{
            uint64_t time_stamp = 0;

        #ifdef DETECT_NETWORK
            pp_result_buf *det_buf = NULL;
            det_buf = (pp_result_buf *)nc_tsfs_ff_get_readable_buffer_and_timestamp(ch+DETECT_NETWORK, &time_stamp);
            if (det_buf) {
                nc_draw_gl_npu(g_viewport[ch], det_buf->net_task, det_buf, g_npu_prog);
                /* ★ [BOARD1] 앞차 거리/접근 변화율 분석 -> 로그(+추후 UDP 전송) */
                nc_board1_front_car_analyze(ch, det_buf);
                (void)time_stamp;
            }
            nc_tsfs_ff_finish_read_buf(ch+DETECT_NETWORK);
        #endif

        #ifdef SEGMENT_NETWORK
            pp_result_buf *seg_buf = NULL;
            seg_buf = (pp_result_buf *)nc_tsfs_ff_get_readable_buffer_and_timestamp(ch+SEGMENT_NETWORK, &time_stamp);
            if (seg_buf) {
                nc_draw_gl_npu(g_viewport[ch], seg_buf->net_task, seg_buf, g_npu_prog);
            }
            nc_tsfs_ff_finish_read_buf(ch+SEGMENT_NETWORK);
        #endif

        #ifdef LANE_NETWORK
            pp_result_buf *lane_buf = NULL;
            lane_buf = (pp_result_buf *)nc_tsfs_ff_get_readable_buffer_and_timestamp(ch+LANE_NETWORK, &time_stamp);
            if (lane_buf) {
                nc_draw_gl_npu(g_viewport[ch], lane_buf->net_task, lane_buf, g_npu_prog);
            }
            nc_tsfs_ff_finish_read_buf(ch+LANE_NETWORK);
        #endif
        }
    }

    /* ★ [Qt 시리얼] 내 Qt 이벤트 갱신(매 프레임) + 내 SOS 자차 제어
     *   - g_last_qt_event: 주기 송신 시 event_type 으로 실어 보냄(내 조작 전파)
     *   - 내 EMERGENCY_BUTTON_ON  -> 내 speed -20 (한 번, 서행)
     *   - 내 EMERGENCY_BUTTON_OFF -> 초기 속도 복귀 */
    {
        uint32_t qt = serial_gui_get_event();
        if (qt != g_last_qt_event) {
            /* Qt 상태가 바뀌는 순간에만 자차 제어 반영 */
            if (qt == EVT_EMERGENCY_BUTTON_ON && !g_sos_active) {
                g_sos_active = 1;
                g_my_speed  -= SPEED_SLOW_DELTA;      /* 내 SOS -> -20 서행 */
                if (g_my_speed < 0.0f) g_my_speed = 0.0f;
                printf("[SOS] my emergency ON -> speed -20 = %.1f (cruise)\n", g_my_speed);
            } else if (qt == EVT_EMERGENCY_BUTTON_OFF && g_sos_active) {
                g_sos_active = 0;
                g_my_speed   = g_my_speed_init;       /* 복귀 */
                printf("[SOS] my emergency OFF -> resume speed=%.1f\n", g_my_speed);
            }
            /* ★ 내 Qt 차선변경: 옆차선 뒤에 빠른 차가 있으면 5초(양보), 없으면 3초 후 변경.
             *   신호 자체는 event_type 으로 다른 차에 전파된다. */
            else if (qt == EVT_LANE_CHANGE_LEFT_ON) {
                g_lane_change_pending = 1;
                g_lane_change_dir     = -1;   /* 왼쪽: lane 번호 감소 */
                g_lane_change_ms      = nc_get_mono_time();
                g_lane_change_delay   = lane_change_need_yield()
                                        ? LANE_CHANGE_YIELD_MS : LANE_CHANGE_DELAY_MS;
                printf("[LANE] left signal -> change in %ums\n", g_lane_change_delay);
            } else if (qt == EVT_LANE_CHANGE_RIGHT_ON) {
                g_lane_change_pending = 1;
                g_lane_change_dir     = +1;   /* 오른쪽: lane 번호 증가 */
                g_lane_change_ms      = nc_get_mono_time();
                g_lane_change_delay   = lane_change_need_yield()
                                        ? LANE_CHANGE_YIELD_MS : LANE_CHANGE_DELAY_MS;
                printf("[LANE] right signal -> change in %ums\n", g_lane_change_delay);
            } else if (qt == EVT_LANE_CHANGE_LEFT_OFF || qt == EVT_LANE_CHANGE_RIGHT_OFF) {
                if (g_lane_change_pending) {
                    g_lane_change_pending = 0;   /* 신호 끄면 예약 취소 */
                    printf("[LANE] signal off -> canceled\n");
                }
            }
            g_last_qt_event = qt;
        }

        /* ★ 예약된 차선변경 실행: 대기시간이 지나면 실제로 lane 을 바꾼다 */
        if (g_lane_change_pending &&
            (nc_get_mono_time() - g_lane_change_ms) >= g_lane_change_delay) {
            int new_lane = g_my_lane + g_lane_change_dir;
            if (new_lane >= LANE_MIN && new_lane <= LANE_MAX) {
                printf("[LANE] lane %d -> %d\n", g_my_lane, new_lane);
                g_my_lane = new_lane;
            } else {
                printf("[LANE] cannot change (lane %d is edge)\n", g_my_lane);
            }
            g_lane_change_pending = 0;
        }
    }

    /* ★ [V2V] 매 프레임: 수신 이벤트 가져와 판단 + 판단 결과를 화면에 반영
     *   - v2v_comm.c 는 이벤트를 저장만 함 -> 여기서 poll 해서 nc_v2v_decide()로 판단.
     *   - 판단 결과(g_v2v_action)를 화면에 표시하고 유지시간 지나면 해제. */
    {
        V2VEventMsg rx;
        while (v2v_poll_event(&rx)) {   /* 밀린 이벤트 모두 처리(마지막이 최신) */
            save_peer_state(&rx);        /* 판단용 상태 저장(조용히) */
            nc_v2v_decide(&rx);
        }

        /* 판단 유지시간 만료 -> 행동 해제
         *   ★ 단, 목표정지가 진행 중이면(g_stop_active) 해제하지 않는다.
         *     여기서 NONE 으로 바꾸면 감속률이 STOP(급) -> SLOW(완만)로 떨어져
         *     실제로 안 서는 것처럼 보인다. */
        if (g_v2v_action != V2V_ACT_NONE && !g_stop_active) {
            uint64_t now = nc_get_mono_time();
            if (now - g_v2v_action_ms > V2V_STATE_HOLD_MS) {
                g_v2v_action = V2V_ACT_NONE;
            }
        }

        /* 행동별 화면 표시 */
        if (g_v2v_action != V2V_ACT_NONE) {
            glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
            float col_red[3]    = {1.0f, 0.0f, 0.0f};
            float col_orange[3] = {1.0f, 0.6f, 0.0f};
            const char *msg = "";
            float *col = col_red;
            switch (g_v2v_action) {
                case V2V_ACT_CAUTION:
                    msg = "V2V: CAUTION (keep speed)";
                    col = col_orange; break;
                case V2V_ACT_SLOW:
                    msg = "V2V: SLOWING DOWN (brake far behind)";
                    col = col_orange; break;
                case V2V_ACT_STOP:
                    msg = "V2V: EMERGENCY STOP (video brake, near)";
                    col = col_red; break;
                case V2V_ACT_WAIT:
                    msg = "V2V: WAIT - yield lane change (peer first)";
                    col = col_orange; break;
                case V2V_ACT_SLOW_CRUISE:
                    msg = "V2V: SLOW CRUISE (-20, caution ahead)";
                    col = col_orange; break;
                default: break;
            }
            nc_opengl_draw_text(&font_38, msg, 10, 950, 1.0f,
                                col, WINDOW_WIDTH, WINDOW_HEIGHT, g_font_prog);
        }
    }

    /* ★ [시간축] position/speed 1초마다 갱신 (주기 송신과 같은 타이밍)
     *   - 평소: position += speed*SCALE (속도만큼 전진)
     *   - 목표정지(g_stop_active): speed를 감속률만큼 줄이고, 목표 position에서 멈춘다.
     *     추월 금지: position 이 목표를 넘지 않도록 clamp.
     *   [v9] position 갱신은 speed(의도한 속도)가 아니라 speed_final(기상·혼잡도 캡 반영,
     *     아래 v2v_max_speed_by_condition() 참고)을 씀. speed 자체는 이 캡과 무관하게
     *     그대로 갱신되니(v8.6의 자차 브레이크 반응 포함), 캡이 풀리면 speed_final도
     *     자동으로 speed까지 회복됨. */
    if ((framecnt % 30) == 0) {
        /* ★ 40초마다 리셋 (position 0 + 속도/상태 복구, 재실행 불필요) */
        if (framecnt > 0 && (framecnt % POS_RESET_FRAMES) == 0) {
            g_my_position = 0.0f;
            g_my_speed    = g_my_speed_init;   /* ★ 속도도 초기값으로 복구(정지 상태 풀림) */
            g_stop_active = 0;
            g_v2v_action  = V2V_ACT_NONE;
            printf("[SIM] reset -> pos=0, speed=%.1f (40s cycle)\n", g_my_speed);
        }
        if (g_stop_active) {
            /* ★ 감속을 먼저 보여준다: speed를 감속률만큼 줄이며 매초 로그.
             *   speed가 0이 될 때까지 감속 과정(60->40->20->0)을 다 출력.
             *   목표 도달/추월 방지는 감속과 별개로 clamp만. */
            float decel = (g_v2v_action == V2V_ACT_STOP) ? DECEL_STOP_PER_S : DECEL_SLOW_PER_S;
            g_my_speed -= decel;
            if (g_my_speed < 0.0f) g_my_speed = 0.0f;

            g_my_speed_final = fminf(g_my_speed, v2v_max_speed_by_condition(g_molit_weather, g_molit_congestion));  /* [v9] */
            g_my_position += g_my_speed_final * SPEED_TO_POS_SCALE;

            /* 추월 금지: 목표 넘으면 목표에 고정 */
            if (g_my_position > g_stop_target_pos)
                g_my_position = g_stop_target_pos;

            printf("[SIM] BRAKE decel: speed=%.1f speed_final=%.1f pos=%.1f (target=%.1f)\n",
                   g_my_speed, g_my_speed_final, g_my_position, g_stop_target_pos);

            /* 완전히 멈추면(speed=0) 정지 완료 */
            if (g_my_speed <= 0.0f) {
                g_stop_active = 0;
                printf("[SIM] fully stopped at pos=%.1f\n", g_my_position);
            }
        } else {
            /* ★ 서행(-20) 반영: 수신 판단(Qt SOS 받음 / 영상 brake 원거리)으로
             *   g_slow_pending 이 켜지면 한 번만 -20 하고, 그 속도로 계속 주행.
             *   ★ 서행은 "느리게 계속 달린다"는 뜻이므로 SPEED_SLOW_MIN 아래로
             *     내려가지 않는다(완전 정지는 STOP 판단에서만 일어난다). */
            if (g_slow_pending) {
                g_my_speed -= SPEED_SLOW_DELTA;
                if (g_my_speed < SPEED_SLOW_MIN) g_my_speed = SPEED_SLOW_MIN;
                g_slow_pending = 0;            /* 감속은 한 번만(중복 방지), 서행 유지 */
                printf("[SIM] slow cruise -> speed = %.1f\n", g_my_speed);
            }
            /* 평소/서행 주행: speed_final(캡 반영) 기준으로 전진 */
            g_my_speed_final = fminf(g_my_speed, v2v_max_speed_by_condition(g_molit_weather, g_molit_congestion));  /* [v9] */
            g_my_position += g_my_speed_final * SPEED_TO_POS_SCALE;
        }
    }

    /* ★ [V2V] 주기적 상태 broadcast (1초에 1회 = 30프레임)
     *   - event_type = 내 Qt 이벤트(시리얼에서 갱신, 평소=LEFT_OFF)
     *   - brake      = 영상 warning(0/1)
     *   - [v9] speed 필드는 speed_final(실제로 내는 속도) - 다른 차량의 차선변경
     *     양보 판단(ev->speed > g_my_speed_final)이 실제 속도끼리 비교하게 함. */
    if ((framecnt % 30) == 0) {
        V2VEventMsg st;
        st.magic          = V2V_MAGIC;
        st.vehicle_id     = g_my_vehicle_id;
        st.event_type     = g_last_qt_event;   /* 내 Qt 신호 */
        st.timestamp      = nc_get_mono_time();
        st.front_distance = g_out_front_distance;
        st.brake          = g_out_approach_warn;  /* 영상 warning */
        st.position       = g_my_position;
        st.lane           = g_my_lane;
        st.speed          = g_my_speed_final;   /* [v9] speed -> speed_final */
        v2v_send_event(&st);
    }

    framecnt++;
    fpscount+=1;
    char buftext[256];
    sprintf(buftext,"GL: %lums, Frame: %lums/%lufps", opengl_time, frametime, fpscount_00);

    glViewport(0,0, WINDOW_WIDTH, WINDOW_HEIGHT);
    float textcolor[3] = {1.0, 0.0, 0.0};
    nc_opengl_draw_text(&font_38, buftext, 10, 1020, 1.0f, textcolor, WINDOW_WIDTH, WINDOW_HEIGHT, g_font_prog);

    clock_gettime(CLOCK_MONOTONIC, &end);
    opengl_time = ((end.tv_sec - begin.tv_sec)*1000 + (end.tv_nsec - begin.tv_nsec)/1000000);
    frametime = ((end.tv_sec - set_time.tv_sec)*1000 + (end.tv_nsec - set_time.tv_nsec)/1000000);
    set_time = end;
    fpstime = fpstime + frametime;

    if(fpstime>=1000)
    {
        if(fpscount_00>0) fpscount_00 = (fpscount_00 + fpscount)/2;
        else              fpscount_00 = fpscount;
        fpscount=0;
        fpstime=0;
    }

    nc_wayland_display_draw(window,(void *)render);
}

int npu_init(st_npu_input_info *npu_input_info)
{
    if (nc_aiw_init_cnn() < 0 ) {
        fprintf(stderr, "nc_aiw_init_cnn() failure!!\n");
        return -1;
    }
#ifdef SHOW_YOLOV8_DETECT
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_YOLOV8_DET), NETWORK_YOLOV8_DET, nc_postprocess_yolov8_inference_result) < 0) return -1;
#endif
#ifdef SHOW_YOLOV5_DETECT
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_YOLOV5_DET), NETWORK_YOLOV5_DET, nc_postprocess_yolov5_inference_result) < 0) return -1;
#endif
#ifdef SHOW_PELEE_SEG
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_PELEE_SEG), NETWORK_PELEE_SEG, nc_postprocess_segmentation_inference_result) < 0) return -1;
#endif
#ifdef SHOW_PELEE_DETECT
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_PELEE_DET), NETWORK_PELEE_DET, nc_postprocess_pelee_inference_result) < 0) return -1;
#endif
#ifdef SHOW_UFLD_LANE
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_UFLD_LANE), NETWORK_UFLD_LANE, nc_postprocess_ufld_inference_result) < 0) return -1;
#endif
#ifdef SHOW_TRI_CHIMERA
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_TRI_CHIMERA), NETWORK_TRI_CHIMERA, nc_postprocess_trichimera_inference_result) < 0) return -1;
#endif
    if(nc_aiw_finish_network_builder() < 0 ) {
        fprintf(stderr, "nc_aiw_finish_network_builder() failure!!\n");
        return -1;
    }

    aiwTensorInfo in_tinfo;
#ifdef SHOW_YOLOV8_DETECT
    if(nc_get_cnn_network_input_resol(NETWORK_YOLOV8_DET, &in_tinfo) < 0) printf("failed to get input resolution\n");
#endif
#ifdef SHOW_YOLOV5_DETECT
    if(nc_get_cnn_network_input_resol(NETWORK_YOLOV5_DET, &in_tinfo) < 0) printf("failed to get input resolution\n");
#endif
#ifdef SHOW_PELEE_SEG
    if(nc_get_cnn_network_input_resol(NETWORK_PELEE_SEG, &in_tinfo) < 0) printf("failed to get input resolution\n");
#endif
#ifdef SHOW_PELEE_DETECT
    if(nc_get_cnn_network_input_resol(NETWORK_PELEE_DET, &in_tinfo) < 0) printf("failed to get input resolution\n");
#endif
#ifdef SHOW_UFLD_LANE
    if(nc_get_cnn_network_input_resol(NETWORK_UFLD_LANE, &in_tinfo) < 0) printf("failed to get input resolution\n");
#endif
#ifdef SHOW_TRI_CHIMERA
    if(nc_get_cnn_network_input_resol(NETWORK_TRI_CHIMERA, &in_tinfo) < 0) printf("failed to get input resolution\n");
#endif

    npu_input_info->w = in_tinfo.dim.w;
    npu_input_info->h = in_tinfo.dim.h;
    npu_input_info->rgb_size = in_tinfo.dim.w * in_tinfo.dim.h * RGB_CNT;

#ifdef USE_BYTETRACK
    for (int i = 0; i < VIDEO_MAX_CH; i++) {
        E_NETWORK_UID net_id;
#ifdef SHOW_PELEE_DETECT
        net_id = NETWORK_PELEE_DET;
#endif
#ifdef SHOW_YOLOV5_DETECT
        net_id = NETWORK_YOLOV5_DET;
#endif
#ifdef SHOW_YOLOV8_DETECT
        net_id = NETWORK_YOLOV8_DET;
#endif
#ifndef SHOW_PELEE_SEG
        if (nc_init_bytetrackers(CAP_FPS, i, net_id) != 0) {
            perror("nc_init_bytetrackers() error");
            return -1;
        }
#endif
    }
#endif

    printf("aiw finish\n");
    return 0;
}

static void signal_int()
{
    running = 0;
    nc_cnn_postprocess_stop();
    v2v_stop();                          /* ★ [V2V] 통신 종료 (블로킹 recv 즉시 해제) */
    serial_gui_stop();                   /* ★ [Qt] 시리얼 수신 종료 */
}

static void usage(int error_code)
{
    fprintf(stderr, "Usage: simple-egl [OPTIONS]\n\n"
        "  -f\tRun in fullscreen mode\n"
        "  -o\tCreate an opaque surface\n"
        "  -h\tThis help text\n\n");
    exit(error_code);
}

void *cnn_task(void *arg)
{
    (void) arg;
    printf("CNN TASK RUN!!\n");

    while (running) {
        stCnnData *cnn_data = NULL;
        if(receive_cnn_buf(&cnn_data) != -1){
            nc_aiw_run_cnn(cnn_data->ptr_cnn_buf, cnn_data->time_stamp_us, cnn_data->cam_ch, cnn_data->net_id);
            if(cnn_data) {
                free(cnn_data->ptr_cnn_buf);
                free(cnn_data);
            }
        }
    }

    printf("EXIT CNN_TASK!!\n");
    return NULL;
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);   /* 출력 버퍼링 끄기 -> 즉시 출력 */

    struct sigaction sigint;
    struct display display;
    struct window  window;
    int i, ret = 0;
    pthread_t p_thread[MAX_TASK_CNT];
    int task_cnt = 0;
    int thr_id;
    int status;

    for (i = 1; i < argc; i++) {
        if (strcmp("-f", argv[i]) == 0)      window.fullscreen = 1;
        else if (strcmp("-o", argv[i]) == 0) window.opaque = 1;
        else if (strcmp("-h", argv[i]) == 0) usage(EXIT_SUCCESS);
        /* ★ [v6] --id 는 자가 IP 감지(v2v_topology_init)로 대체되어 더 이상 안 씀(주석처리, 삭제 아님) */
        // else if (strcmp("--id", argv[i]) == 0 && i+1 < argc)
        //     g_my_vehicle_id = (uint32_t)atoi(argv[++i]);
        else if (strcmp("--position", argv[i]) == 0 && i+1 < argc)
            g_my_position = (float)atof(argv[++i]);
        else if (strcmp("--lane", argv[i]) == 0 && i+1 < argc)
            g_my_lane = atoi(argv[++i]);
        else if (strcmp("--speed", argv[i]) == 0 && i+1 < argc) {
            g_my_speed = (float)atof(argv[++i]);
            g_my_speed_init = g_my_speed;   /* 리셋 복구용 백업 */
            g_my_speed_final = g_my_speed;  /* [v9] 초기값도 speed와 동일하게 시작 */
        }
        else usage(EXIT_FAILURE);
    }

    /* ★ [v6] 실행 인자(--id) 대신 자가 IP 감지로 vehicle_id/peer 목록 자동 결정 */
    const char **v2v_peer_ips = NULL;
    int          v2v_peer_cnt = 0;
    if (v2v_topology_init(&g_my_vehicle_id, &v2v_peer_ips, &v2v_peer_cnt) < 0) {
        fprintf(stderr, "[v2v_topology] failed to determine vehicle_id from local IP\n");
        return -1;
    }

    printf("[cfg] id=%u position=%.1f lane=%d speed=%.1f\n",
           g_my_vehicle_id, g_my_position, g_my_lane, g_my_speed);

    /* ★ [V2V] topology 결정 후 통신 시작 */
    v2v_init(g_my_vehicle_id, v2v_peer_ips, v2v_peer_cnt);
    serial_gui_init("/dev/ttyS2");   /* ★ [Qt] 시리얼 수신 시작 (VM Qt -> 보드) */

    sigint.sa_handler = (sighandler_t)signal_int;
    sigemptyset(&sigint.sa_mask);
    sigint.sa_flags = SA_RESETHAND;
    sigaction(SIGINT, &sigint, NULL);

    memset(&display, 0, sizeof(display));
    memset(&window, 0, sizeof(window));
    memset(&v4l2_config, 0, sizeof(v4l2_config));

    set_v4l2_config();
    nc_init_path_localizer();

    window.display = &display;
    display.window = &window;
    window.window_size.width  = WINDOW_WIDTH;
    window.window_size.height = WINDOW_HEIGHT;

    set_viewport_config();
    nc_wayland_display_init(&display,(void *)render);

    ret = v4l2_initialize();
    if(ret < 0) {
        printf("Error v4l2_initialize\n");
        return -1;
    }

    gl_initialize(&window);
#ifdef USE_8MP_VI
    dsr_init();
#endif
#ifdef USE_ADAS_LD
    ld_opengl_program_set(&g_npu_prog);
    NC_ADAS_OPEN();
#endif

    for (int i = 0; i < VIDEO_MAX_CH; i++) {
        if (v4l2_config[i].video_buf.video_fd == -1){
        }
        else{
    #ifdef DETECT_NETWORK
            if(nc_tsfs_ff_create_buffers(i+DETECT_NETWORK, sizeof(pp_result_buf)) < 0) exit(1);
    #endif
    #ifdef SEGMENT_NETWORK
            if(nc_tsfs_ff_create_buffers(i+SEGMENT_NETWORK, sizeof(pp_result_buf)) < 0) exit(1);
    #endif
    #ifdef LANE_NETWORK
            if(nc_tsfs_ff_create_buffers(i+LANE_NETWORK, sizeof(pp_result_buf)) < 0) exit(1);
    #endif
        }
    }
    mq_unlink(MQ_NAME_CNN_BUF);

    if(npu_init(&npu_input_info) < 0) {
        printf("failed to init NPU\n");
        return -1;
    }

    printf("create tasks\n");
    thr_id = pthread_create(&p_thread[task_cnt++], NULL, cnn_task, (void *)NULL);
    if (thr_id < 0) { perror("thread create error : cnn_task"); exit(1); }

    cnn_postprocess_arg cnn_post_param;
    cnn_post_param.target_width = WINDOW_WIDTH;
    cnn_post_param.target_height = WINDOW_HEIGHT;
    thr_id = pthread_create(&p_thread[task_cnt++], NULL, nc_cnn_postprocess_task, (void *)&cnn_post_param);
    if (thr_id < 0) { perror("thread create error : nc_cnn_postprocess_task"); exit(1); }

#ifdef USE_ADAS_LD
    thr_id = pthread_create(&p_thread[task_cnt++], NULL, ld_task, NULL);
    if (thr_id < 0) { perror("thread create error : ld_task"); exit(1); }
#endif

    while (running && ret != -1)
    {
        ret = wl_display_dispatch(display.display);
    }

    for(int i =0; i< task_cnt; i++) {
        pthread_join(p_thread[i], (void **)&status);
    }

#ifdef USE_ADAS_LD
    NC_ADAS_CLOSE();
#endif

#ifdef USE_BYTETRACK
    for (int i = 0; i < VIDEO_MAX_CH; i++) {
        nc_deInit_bytetrackers(i);
    }
#endif

#ifdef USE_8MP_VI
    dsr_deinit();
#endif

    nc_wayland_display_destroy(&display);
    return 0;
}
