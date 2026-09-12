// SDK_files/wayland_npu_app/v2v_topology.c
/**
********************************************************************************
* @file    : v2v_topology.c
* @brief   : V2V peer 연결 topology 구현
*            - nc_get_local_IPv4() (기존 SDK 유틸, nc_utils.c) 재사용해 자가 IP 감지
*            - CONTEXT.md IP/vehicle_id 매핑표 기준, vehicle_id가 나보다 큰 보드만
*              peer 후보(dial 대상)로 계산 - 각 쌍은 낮은 id 쪽에서만 단방향으로
*              연결을 시작해 상호 동시 dial 경합(dedup 판단 불일치로 양쪽 다
*              끊기는 문제)을 원천 차단한다. 가장 큰 id(국토교통부=100)는 이
*              규칙만으로 자동 peer 후보 0개(dial 안 하고 accept만).
********************************************************************************
*/
#include <stdio.h>
#include <string.h>

#include "v2v_topology.h"
#include "nc_utils.h"   /* nc_get_local_IPv4() */

#define V2V_TOPOLOGY_IF_NAME    "eth0"
#define V2V_TOPOLOGY_MOLIT_ID   (100u)
#define V2V_TOPOLOGY_IP_BUF_LEN (16)

typedef struct {
    uint32_t vehicle_id;
    char     ip[V2V_TOPOLOGY_IP_BUF_LEN];
} V2VTopologyEntry;

/* IP/vehicle_id 매핑표 - 보드 구성 변경(대수 조정 등) 시 이 표만 갱신한다.
 *   (CONTEXT.md "IP/vehicle_id 매핑 + U-Boot IP 설정 절차" 참고) */
static const V2VTopologyEntry g_topology_table[] = {
    { 1,                    "192.168.13.101" },
    { 2,                    "192.168.13.102" },
    { 3,                    "192.168.13.103" },
    { V2V_TOPOLOGY_MOLIT_ID, "192.168.13.100" },
};
#define V2V_TOPOLOGY_TABLE_CNT \
    (int)(sizeof(g_topology_table) / sizeof(g_topology_table[0]))

/* peer 후보 IP 포인터 버퍼 - 매핑표 크기보다 1 작으면 충분(자기 자신 제외) */
static const char *g_peer_ip_ptrs[V2V_TOPOLOGY_TABLE_CNT];

int v2v_topology_init(uint32_t *out_my_vehicle_id,
                       const char ***out_peer_ips,
                       int *out_peer_cnt)
{
    char my_ip[V2V_TOPOLOGY_IP_BUF_LEN];

    if (nc_get_local_IPv4(V2V_TOPOLOGY_IF_NAME, my_ip) < 0) {
        printf("[v2v_topology] nc_get_local_IPv4(%s) failed\n", V2V_TOPOLOGY_IF_NAME);
        return -1;
    }

    int my_idx = -1;
    for (int i = 0; i < V2V_TOPOLOGY_TABLE_CNT; i++) {
        if (strcmp(my_ip, g_topology_table[i].ip) == 0) {
            my_idx = i;
            break;
        }
    }
    if (my_idx < 0) {
        printf("[v2v_topology] local IP %s not found in mapping table\n", my_ip);
        return -1;
    }

    uint32_t my_vehicle_id = g_topology_table[my_idx].vehicle_id;
    *out_my_vehicle_id = my_vehicle_id;

    /* [v6 수정] 상호 동시 dial 경합 방지: vehicle_id가 나보다 큰 보드만 dial한다.
     *   각 쌍은 낮은 vehicle_id 쪽에서만 단방향으로 연결을 시작하므로, 두 보드가
     *   동시에 서로 dial해서 v2v_add_peer() 중복 판단이 양쪽에서 다르게 갈리는
     *   경합(둘 다 끊김) 자체가 발생하지 않는다. 상대가 재부팅돼도 낮은 id 쪽의
     *   재시도 루프가 다시 잡아주므로 복구력은 그대로 유지된다.
     *   매핑표에서 가장 큰 id(국토교통부=100)는 이 규칙만으로 자동으로
     *   peer 후보 0개(dial 안 함, accept만)가 된다. */
    int cnt = 0;
    for (int i = 0; i < V2V_TOPOLOGY_TABLE_CNT; i++) {
        if (g_topology_table[i].vehicle_id > my_vehicle_id) {
            g_peer_ip_ptrs[cnt++] = g_topology_table[i].ip;
        }
    }

    *out_peer_ips = g_peer_ip_ptrs;
    *out_peer_cnt = cnt;

    printf("[v2v_topology] my_ip=%s -> vehicle_id=%u, peer_cnt=%d\n",
           my_ip, *out_my_vehicle_id, cnt);
    return 0;
}
