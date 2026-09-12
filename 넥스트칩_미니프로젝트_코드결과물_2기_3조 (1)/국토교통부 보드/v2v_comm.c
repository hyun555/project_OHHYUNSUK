// SDK_files/wayland_npu_app/v2v_comm.c
/**
********************************************************************************
* @file    : v2v_comm.c
* @brief   : V2V TCP 통신 구현 - 순수 통신만 (판단 없음)
*            nc_streamer.c 패턴 재사용. 판단은 전부 wayland_npu_app.c.
*            [v6] 변경점: v2v_add_peer() 중복 IP 연결 방지, v2v_connect_peers()
*            단발 호출 -> 3초 주기 재시도 스레드로 변경(peer topology 라운드).
********************************************************************************
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>

#include "v2v_comm.h"

#define MAX_PEER_CNT 8
#define V2V_CONNECT_RETRY_SEC 3

typedef struct {
    int      fd;
    int      connected;
    uint32_t vehicle_id;
    char     ipv4_addr[INET_ADDRSTRLEN];
} V2VPeerInfo;

static pthread_mutex_t peer_mutex;
static V2VPeerInfo     g_peers[MAX_PEER_CNT];   /* 전역 - 자동 0 초기화 */

static int          listen_fd;
static volatile int running_v2v = 1;

/* [v6] dial 대상 목록 - v2v_init()에서 저장, 재시도 스레드가 참조 */
static const char **g_dial_peer_ips;
static int           g_dial_peer_cnt;

/* 수신 메시지 버퍼 (판단 아님, 저장만) */
static pthread_mutex_t v2v_evt_mutex = PTHREAD_MUTEX_INITIALIZER;
static V2VEventMsg     g_last_event;
static int             g_has_new_event = 0;

static int   v2v_add_peer(int fd, uint32_t vehicle_id, const char *ipv4_addr);
static void  v2v_remove_peer(int fd);
static int   v2v_peer_count(void);
static int   v2v_is_ip_connected(const char *ipv4_addr);
static ssize_t recv_all(int fd, void *buf, size_t len);
static void *v2v_peer_recv_task(void *arg);
static void *v2v_accept_task(void *arg);
static void  v2v_try_connect_one(const char *ip, uint32_t my_vehicle_id);
static void *v2v_connect_retry_task(void *arg);
static void  store_received_event(const V2VEventMsg *ev);

/* ── 연결 테이블 ────────────────────────────────────────── */
/* [v6] 같은 IP로 이미 connected인 항목이 있으면 거절(0 반환) - 중복 연결 방지.
 *   두 보드가 동시에 서로 dial해도 먼저 성사된 연결 하나만 남는다(peer_mutex로 보호). */
static int v2v_add_peer(int fd, uint32_t vehicle_id, const char *ipv4_addr)
{
    pthread_mutex_lock(&peer_mutex);

    for (int i = 0; i < MAX_PEER_CNT; i++) {
        if (g_peers[i].connected && strcmp(g_peers[i].ipv4_addr, ipv4_addr) == 0) {
            pthread_mutex_unlock(&peer_mutex);
            return 0;
        }
    }

    int added = 0;
    for (int i = 0; i < MAX_PEER_CNT; i++) {
        if (g_peers[i].fd == 0 && !g_peers[i].connected) {
            g_peers[i].fd = fd;
            g_peers[i].vehicle_id = vehicle_id;
            strncpy(g_peers[i].ipv4_addr, ipv4_addr, INET_ADDRSTRLEN);
            g_peers[i].connected = 1;
            added = 1;
            break;
        }
    }
    pthread_mutex_unlock(&peer_mutex);
    return added;
}

static void v2v_remove_peer(int fd)
{
    pthread_mutex_lock(&peer_mutex);
    for (int i = 0; i < MAX_PEER_CNT; i++) {
        if (g_peers[i].fd == fd) { memset(&g_peers[i], 0, sizeof(V2VPeerInfo)); break; }
    }
    pthread_mutex_unlock(&peer_mutex);
}

static int v2v_peer_count(void)
{
    pthread_mutex_lock(&peer_mutex);
    int cnt = 0;
    for (int i = 0; i < MAX_PEER_CNT; i++) if (g_peers[i].connected) cnt++;
    pthread_mutex_unlock(&peer_mutex);
    return cnt;
}

/* [v6] 재시도 스레드가 "이미 연결된 peer"를 skip하기 위해 사용 */
static int v2v_is_ip_connected(const char *ipv4_addr)
{
    pthread_mutex_lock(&peer_mutex);
    int found = 0;
    for (int i = 0; i < MAX_PEER_CNT; i++) {
        if (g_peers[i].connected && strcmp(g_peers[i].ipv4_addr, ipv4_addr) == 0) {
            found = 1;
            break;
        }
    }
    pthread_mutex_unlock(&peer_mutex);
    return found;
}

/* ── recv_all : TCP 경계 보장 ───────────────────────────── */
static ssize_t recv_all(int fd, void *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        ssize_t n = recv(fd, (char *)buf + got, len - got, 0);
        if (n <= 0) return n;
        got += (size_t)n;
    }
    return (ssize_t)got;
}

/* ── 수신 저장 + RECV 로그 (판단 아님) ──────────────────── */
static void store_received_event(const V2VEventMsg *ev)
{
    pthread_mutex_lock(&v2v_evt_mutex);
    g_last_event    = *ev;
    g_has_new_event = 1;
    pthread_mutex_unlock(&v2v_evt_mutex);

    printf("[v2v] RECV <- id=%u | type=%u front=%.1fm brake=%d pos=%.1f lane=%d speed=%.1f\n",
           ev->vehicle_id, ev->event_type, ev->front_distance, ev->brake,
           ev->position, ev->lane, ev->speed);
}

/* ── 피어별 수신 스레드 ─────────────────────────────────── */
static void *v2v_peer_recv_task(void *arg)
{
    int fd = (int)(intptr_t)arg;
    while (running_v2v) {
        V2VEventMsg ev;
        ssize_t n = recv_all(fd, &ev, sizeof(ev));
        if (n <= 0) {
            v2v_remove_peer(fd);
            close(fd);
            break;
        }
        if (ev.magic != V2V_MAGIC) continue;
        store_received_event(&ev);   /* 저장만, 판단은 앱이 */
    }
    return NULL;
}

/* ── 서버 accept 스레드 ─────────────────────────────────── */
static void *v2v_accept_task(void *arg)
{
    (void)arg;
    while (running_v2v) {
        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(listen_fd, &rset);
        struct timeval tv = {0, 100000};
        select(listen_fd + 1, &rset, NULL, NULL, &tv);

        if (FD_ISSET(listen_fd, &rset)) {
            struct sockaddr_in cli_addr;
            socklen_t len = sizeof(cli_addr);
            int fd = accept(listen_fd, (struct sockaddr *)&cli_addr, &len);
            if (fd < 0) continue;
            if (v2v_peer_count() >= MAX_PEER_CNT) { close(fd); continue; }

            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &cli_addr.sin_addr, ip, sizeof(ip));

            uint32_t peer_vehicle_id;
            if (recv_all(fd, &peer_vehicle_id, sizeof(peer_vehicle_id)) <= 0) {
                close(fd);
                continue;
            }

            if (!v2v_add_peer(fd, peer_vehicle_id, ip)) {
                printf("[v2v] duplicate peer rejected (accept): id=%u ip=%s\n", peer_vehicle_id, ip);
                close(fd);
                continue;
            }
            printf("[v2v] peer connected: id=%u ip=%s\n", peer_vehicle_id, ip);

            pthread_t tid;
            pthread_create(&tid, NULL, v2v_peer_recv_task, (void *)(intptr_t)fd);
            pthread_detach(tid);
        }
    }
    return NULL;
}

/* ── 클라이언트: peer 1개 연결 시도(1회) ────────────────── */
static void v2v_try_connect_one(const char *ip, uint32_t my_vehicle_id)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(V2V_PORT);
    inet_pton(AF_INET, ip, &addr.sin_addr);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return;
    }
    send(fd, &my_vehicle_id, sizeof(my_vehicle_id), MSG_NOSIGNAL);

    if (!v2v_add_peer(fd, 0, ip)) {
        printf("[v2v] duplicate peer rejected (dial): ip=%s\n", ip);
        close(fd);
        return;
    }
    printf("[v2v] connected to peer %s\n", ip);

    pthread_t tid;
    pthread_create(&tid, NULL, v2v_peer_recv_task, (void *)(intptr_t)fd);
    pthread_detach(tid);
}

/* ── 클라이언트: 3초 주기로 미연결 peer 재시도 ──────────── */
/* [v6] 단발 v2v_connect_peers() -> 재시도 스레드로 변경. 이미 connected인 peer는 skip. */
static void *v2v_connect_retry_task(void *arg)
{
    uint32_t my_vehicle_id = (uint32_t)(intptr_t)arg;
    while (running_v2v) {
        for (int i = 0; i < g_dial_peer_cnt; i++) {
            if (!running_v2v) break;
            if (v2v_is_ip_connected(g_dial_peer_ips[i])) continue;
            v2v_try_connect_one(g_dial_peer_ips[i], my_vehicle_id);
        }
        sleep(V2V_CONNECT_RETRY_SEC);
    }
    return NULL;
}

/* ── 송신: broadcast + SENT 로그 ────────────────────────── */
int v2v_send_event(const V2VEventMsg *ev)
{
    int ret = 0, sent_cnt = 0;
    pthread_mutex_lock(&peer_mutex);
    for (int i = 0; i < MAX_PEER_CNT; i++) {
        if (!g_peers[i].connected) continue;
        if (send(g_peers[i].fd, ev, sizeof(*ev), MSG_NOSIGNAL) < 0) {
            close(g_peers[i].fd);
            memset(&g_peers[i], 0, sizeof(V2VPeerInfo));
            ret = -1;
        } else {
            sent_cnt++;
        }
    }
    pthread_mutex_unlock(&peer_mutex);

    printf("[v2v] SENT -> %d peer(s) | type=%u front=%.1fm brake=%d pos=%.1f lane=%d speed=%.1f\n",
           sent_cnt, ev->event_type, ev->front_distance, ev->brake,
           ev->position, ev->lane, ev->speed);
    return ret;
}

/* ── 앱이 수신 메시지 가져감 ────────────────────────────── */
int v2v_poll_event(V2VEventMsg *out)
{
    int has = 0;
    pthread_mutex_lock(&v2v_evt_mutex);
    if (g_has_new_event) {
        *out = g_last_event;
        g_has_new_event = 0;
        has = 1;
    }
    pthread_mutex_unlock(&v2v_evt_mutex);
    return has;
}

/* ── 초기화 / 종료 ──────────────────────────────────────── */
void v2v_init(uint32_t my_vehicle_id, const char **peer_ips, int peer_cnt)
{
    signal(SIGPIPE, SIG_IGN);
    pthread_mutex_init(&peer_mutex, NULL);

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    int val = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(V2V_PORT);
    bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr));
    listen(listen_fd, 8);

    pthread_t accept_tid;
    pthread_create(&accept_tid, NULL, v2v_accept_task, NULL);
    pthread_detach(accept_tid);

    /* [v6] dial 목록 저장 후 3초 주기 재시도 스레드 기동 (peer_ips는 호출자 소유 정적 버퍼) */
    g_dial_peer_ips = peer_ips;
    g_dial_peer_cnt = peer_cnt;

    pthread_t retry_tid;
    pthread_create(&retry_tid, NULL, v2v_connect_retry_task, (void *)(intptr_t)my_vehicle_id);
    pthread_detach(retry_tid);

    printf("[v2v] init done (my_id=%u, port=%d, peers=%d)\n",
           my_vehicle_id, V2V_PORT, peer_cnt);
}

void v2v_stop(void)
{
    running_v2v = 0;
    shutdown(listen_fd, SHUT_RDWR);
    pthread_mutex_lock(&peer_mutex);
    for (int i = 0; i < MAX_PEER_CNT; i++)
        if (g_peers[i].connected) shutdown(g_peers[i].fd, SHUT_RDWR);
    pthread_mutex_unlock(&peer_mutex);
}
