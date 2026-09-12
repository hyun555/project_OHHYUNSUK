/**
********************************************************************************
* @file    : serial_gui.c
* @brief   : Qt(VM) -> 보드 시리얼 수신 구현
*            - /dev/ttyS2, 115200 8N1, 흐름제어 없음
*            - ★ 1글자 프로토콜: L/l(좌 ON/OFF) R/r(우 ON/OFF) E/e(SOS ON/OFF)
*              (긴 문자열은 시리얼에서 문자 유실이 잦아 1글자로 단순화)
*            - 받은 문자를 EventType 으로 변환해 전역에 저장 (스레드 안전)
*            - 판단/제어는 하지 않는다. wayland 가 값을 읽어서 처리.
********************************************************************************
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>

#include "serial_gui.h"

#define SERIAL_DEV_DEFAULT  "/dev/ttyS2"

static int             g_serial_fd = -1;
static char            g_sg_path[64] = "/dev/ttyS2";   /* 재오픈용 경로 보관 */
static volatile int    g_sg_running = 0;
static pthread_t       g_sg_tid;

/* 최근 Qt 이벤트 (평소 = LEFT_OFF: "아무것도 없음" 대표값) */
static pthread_mutex_t g_sg_mtx = PTHREAD_MUTEX_INITIALIZER;
static uint32_t        g_sg_event = EVT_LANE_CHANGE_LEFT_OFF;

/* ── 문자 -> EventType 매핑 (1글자 프로토콜) ──
 *   시리얼 문자 유실을 줄이기 위해 Qt는 1글자만 보낸다.
 *     L = 좌 ON    l = 좌 OFF
 *     R = 우 ON    r = 우 OFF
 *     E = SOS ON   e = SOS OFF
 *   반환 0xFFFFFFFF = 알 수 없는 문자(무시). */
static uint32_t str_to_event(const char *s)
{
    if (!s || s[0] == '\0') return 0xFFFFFFFFu;

    switch (s[0]) {
        case 'L': return EVT_LANE_CHANGE_LEFT_ON;
        case 'l': return EVT_LANE_CHANGE_LEFT_OFF;
        case 'R': return EVT_LANE_CHANGE_RIGHT_ON;
        case 'r': return EVT_LANE_CHANGE_RIGHT_OFF;
        case 'E': return EVT_EMERGENCY_BUTTON_ON;
        case 'e': return EVT_EMERGENCY_BUTTON_OFF;
        default:  return 0xFFFFFFFFu;   /* 알 수 없음(깨진 수신) */
    }
}

/* ── termios 설정 적용 (콘솔이 설정을 되돌리는 경우 재적용용) ── */
static void apply_termios(int fd)
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) return;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag &= ~PARENB;   /* 패리티 없음 */
    tty.c_cflag &= ~CSTOPB;   /* 정지비트 1 */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       /* 데이터 8비트 */
    tty.c_cflag |= (CLOCAL | CREAD);
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;  /* 하드웨어 흐름제어 끔 */
#endif
    cfmakeraw(&tty);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);   /* 소프트웨어 흐름제어 끔 */
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &tty);
}

/* ── 포트 열고 설정 (재연결에도 사용) ── */
static int open_and_setup(const char *path)
{
    int fd = open(path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    /* O_NONBLOCK 은 open 용도로만 쓰고, 이후엔 블로킹 해제 */
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);

    apply_termios(fd);
    tcflush(fd, TCIFLUSH);
    return fd;
}

/* ── 시리얼 수신 스레드 (1글자 프로토콜: 글자 하나 받으면 즉시 처리) ── */
static void *serial_gui_task(void *arg)
{
    (void)arg;
    int err_cnt = 0;

    while (g_sg_running) {
        /* 블로킹 read: 데이터가 올 때까지 대기.
         * (select/tcsetattr 를 주기적으로 호출하면 수신이 불안정해져 제거) */
        char c;
        ssize_t n = read(g_serial_fd, &c, 1);

        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            printf("[serial] read error: %s -> reopen\n", strerror(errno));
            close(g_serial_fd);
            usleep(200000);
            g_serial_fd = open_and_setup(g_sg_path);
            if (g_serial_fd < 0) usleep(500000);
            continue;
        }
        if (n == 0) {
            if (++err_cnt >= 20) {
                printf("[serial] EOF repeated -> reopen\n");
                close(g_serial_fd);
                usleep(200000);
                g_serial_fd = open_and_setup(g_sg_path);
                err_cnt = 0;
            }
            usleep(50000);
            continue;
        }
        err_cnt = 0;

        /* 개행/공백은 무시 (Qt가 \n 이나 \r\n 을 붙여 보내도 무해) */
        if (c == '\n' || c == '\r' || c == ' ' || c == '\t') continue;

        char s[2]; s[0] = c; s[1] = '\0';
        uint32_t ev = str_to_event(s);

        if (ev == 0xFFFFFFFFu) {
            printf("[serial] !! unknown char: '%c' (0x%02X, ignored)\n",
                   (c >= 32 && c < 127) ? c : '?', (unsigned char)c);
        } else {
            pthread_mutex_lock(&g_sg_mtx);
            g_sg_event = ev;
            pthread_mutex_unlock(&g_sg_mtx);
            printf("[serial] Qt event: '%c' -> type=%u\n", c, ev);
        }
    }
    printf("[serial] receiver thread exit\n");
    return NULL;
}

/* ── 초기화: 포트 열고 termios 설정 후 스레드 시작 ── */
int serial_gui_init(const char *dev)
{
    const char *path = (dev && dev[0]) ? dev : SERIAL_DEV_DEFAULT;

    /* 경로 보관(수신 중 문제 시 재오픈에 사용) */
    strncpy(g_sg_path, path, sizeof(g_sg_path) - 1);
    g_sg_path[sizeof(g_sg_path) - 1] = '\0';

    /* ★ 읽기 전용으로 연다. /dev/ttyS2 는 보드 콘솔이라 쓰기 권한까지 잡으면
     *   커널/앱 로그 출력과 충돌해 수신이 멈춘다. */
    g_serial_fd = open_and_setup(g_sg_path);
    if (g_serial_fd < 0) {
        perror("[serial] open");
        return -1;
    }

    g_sg_running = 1;
    if (pthread_create(&g_sg_tid, NULL, serial_gui_task, NULL) != 0) {
        perror("[serial] pthread_create");
        close(g_serial_fd);
        g_serial_fd = -1;
        g_sg_running = 0;
        return -1;
    }
    pthread_detach(g_sg_tid);

    printf("[serial] Qt serial receiver started on %s (115200 8N1)\n", path);
    return 0;
}

void serial_gui_stop(void)
{
    g_sg_running = 0;
    if (g_serial_fd >= 0) {
        close(g_serial_fd);
        g_serial_fd = -1;
    }
}

uint32_t serial_gui_get_event(void)
{
    uint32_t ev;
    pthread_mutex_lock(&g_sg_mtx);
    ev = g_sg_event;
    pthread_mutex_unlock(&g_sg_mtx);
    return ev;
}// qt 끊기는거 해결 코드 - 안되면 이전으로