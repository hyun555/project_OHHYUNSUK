// SDK_files/wayland_npu_app/wayland_npu_app.c
/**
********************************************************************************
* Copyright (C) 2021 NEXTCHIP Inc. All rights reserved.
********************************************************************************
* @file     : wayland_npu_app.c
* @brief    : wayland_npu application (+ 전방 혼잡도 및 실시간 기상 분석 출력)
********************************************************************************
*/

/*
********************************************************************************
*                    INCLUDES
********************************************************************************
*/
#include <stdio.h>
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

#include "traffic_congestion.h"
#include "weather_analysis.h"  /* ★ 기상 분석 헤더 포함 */
#include "video_control.h"
#include "v2v_comm.h"           /* ★ [v7] MOLIT payload 송신용 V2V 통신 */
#include "v2v_topology.h"       /* ★ [v7] peer 연결 topology (자가 IP 감지) */
#include "v2v_molit_payload.h"  /* ★ [v7] 기상/혼잡도 -> V2VEventMsg 패킹+송신 */
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
*                    DEFINES
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
*                    VARIABLE DECLARATIONS
********************************************************************************
*/
st_npu_input_info npu_input_info;
struct gl_npu_program g_npu_prog;
GLuint g_seg_texture;
struct gl_font_program g_font_prog;
Font font_38;
Font font_24;

unsigned char* image_data;
int running = 1;

st_nc_v4l2_config v4l2_config[VIDEO_MAX_CH];
struct viewport g_viewport[VIDEO_MAX_CH];

#define USE_VIDEO_LOOPBACK
#define LOOPBACK_DEVICE_BASE   (11)
#define LOG_EVERY_N_FRAMES     (8)

/* ★ [v7] 국토교통부 보드 vehicle_id - v2v_topology_init()이 자가 IP 감지로 채움
 *   (기본값 100은 미사용 안전값, 매핑표상 이 보드는 항상 100으로 결정됨) */
static uint32_t g_my_vehicle_id = 100;

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
*                    FUNCTION DEFINITIONS
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
            case 0: g_viewport[i].x = 0;             g_viewport[i].y = WINDOW_HEIGHT/2; break;
            case 1: g_viewport[i].x = WINDOW_WIDTH/2; g_viewport[i].y = WINDOW_HEIGHT/2; break;
            case 2: g_viewport[i].x = 0;             g_viewport[i].y = 0;               break;
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
        v4l2_config[i].video_buf.video_device_num = LOOPBACK_DEVICE_BASE + i;
        v4l2_config[i].dma_mode                    = INTERLEAVE;
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
        if(v4l2_config[i].video_buf.video_fd < 0) {
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

/* ★ [메인 PD 수정] C++ 엄격한 enum 타입 에러 방지 및 깔끔한 출력 포맷 적용 */
static void nc_board1_front_car_analyze(uint32_t cam_ch, pp_result_buf *det_buf, unsigned char *raw_frame)
{
    if (!det_buf) return;
    (void)cam_ch;
    stCnnPostprocessingResults *r = &det_buf->cnn_result;
    stObjDrawInfo *d = &det_buf->draw_info;

    // 프레임 카운트 대신 시간(초) 기반 5초 간격 계산 로직으로 교체
    static time_t last_print_time = 0;
    time_t current_time = time(NULL);

    if (current_time - last_print_time >= 5) {
        TrafficCongestion_e congestion = Evaluate_Forward_Congestion(r, d, WINDOW_WIDTH);
        const char* status_str[] = {"원활", "보통", "혼잡", "정체"};

        int c_idx = (int)congestion;
        if (c_idx < 0 || c_idx > 3) c_idx = 0;

        const char* weather_result = Analyze_Weather_Condition(raw_frame, VIDEO_WIDTH, VIDEO_HEIGHT);

        { char __msg[256]; snprintf(__msg, sizeof(__msg), "[기상] %-22s | [혼잡도] %s", weather_result, status_str[c_idx]); announce(__msg); }

        /* ★ [v7] 분석 결과를 V2VEventMsg로 패킹해 V2V mesh에 송신 */
        v2v_molit_send_payload(g_my_vehicle_id, weather_result, congestion);

        last_print_time = current_time;
    }
}

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
                if(nc_v4l2_queue_buffer(v4l2_config[i].video_buf.video_fd, video_buf.index) == -1) {
                    printf("Error VIDIOC_QBUF buffer %d\n", video_buf.index);
                }

                glBindTexture(GL_TEXTURE_2D, window->gl.texture[i]);
#if defined(USE_8MP_VI) || defined(USE_VIDEO_LOOPBACK)
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, VIDEO_WIDTH, VIDEO_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, v4l2_config[i].video_buf.buffers[video_buf.index].start);
#else
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

                // 현재 프레임 버퍼 주소를 전달하여 기상 분석과 혼잡도를 동시에 수행
                unsigned char *current_frame = (unsigned char *)v4l2_config[ch].video_buf.buffers[0].start;
                nc_board1_front_car_analyze(ch, det_buf, current_frame);

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
    if (nc_aiw_add_network_to_builder(nc_localize_path((const char *)NETWORK_FILE_PELEE_DET), NETWORK_PELEE_DETECT, nc_postprocess_pelee_inference_result) < 0) return -1;
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
    v2v_stop();   /* ★ [v7] V2V 통신 종료 (블로킹 recv 즉시 해제) */
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
    (void)argc;
    (void)argv;
    setvbuf(stdout, NULL, _IONBF, 0);   /* 출력 버퍼링 끄기 */

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
        else usage(EXIT_FAILURE);
    }

    /* ★ [v7] 자가 IP 감지로 vehicle_id/peer 목록 자동 결정 후 V2V 통신 시작
     *   (이 보드는 매핑표상 항상 vehicle_id=100 -> peer 후보 0개, accept만) */
    const char **v2v_peer_ips = NULL;
    int          v2v_peer_cnt = 0;
    if (v2v_topology_init(&g_my_vehicle_id, &v2v_peer_ips, &v2v_peer_cnt) < 0) {
        fprintf(stderr, "[v2v_topology] failed to determine vehicle_id from local IP\n");
        return -1;
    }
    v2v_init(g_my_vehicle_id, v2v_peer_ips, v2v_peer_cnt);

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
    if (thr_id < 0) { perror("thread count error : ld_task"); exit(1); }
#endif

    // ===== [영상 전환 키보드 입력 스레드 시작] =====
    pthread_t video_tid;
    if (pthread_create(&video_tid, NULL, video_control_task, (void *)NULL) == 0) {
        pthread_detach(video_tid);
    }
    // ===============================================

    while (running && ret != -1) {
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
