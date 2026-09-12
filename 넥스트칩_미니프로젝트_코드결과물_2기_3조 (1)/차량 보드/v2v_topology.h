// SDK_files/wayland_npu_app/v2v_topology.h
/**
********************************************************************************
* @file    : v2v_topology.h
* @brief   : V2V peer 연결 topology - IP/vehicle_id 매핑 + 자가 IP 감지
*            - 보드가 늘거나 줄어도 실행 인자(--id)나 소스 하드코딩 없이,
*              nc_get_local_IPv4()로 감지한 자가 IP만으로 vehicle_id와
*              peer 후보 목록을 자동 결정한다.
*            - 차량 보드(1~3)는 100% 동일 코드로 동작해야 하므로, 이 파일의
*              매핑표만 보드 구성 변경 시 갱신 대상이다.
********************************************************************************
*/
#ifndef V2V_TOPOLOGY_H
#define V2V_TOPOLOGY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 자가 IP 감지 -> vehicle_id 결정 + peer 후보 IP 목록 계산.
 *   out_my_vehicle_id : 매핑표에서 찾은 내 vehicle_id
 *   out_peer_ips       : peer 후보 IP 배열(내부 정적 버퍼, 호출자가 free 불필요)
 *   out_peer_cnt       : peer 후보 개수(국토교통부 보드=100이면 항상 0)
 * 반환값: 성공 0, 자가 IP가 매핑표에 없으면 -1. */
int v2v_topology_init(uint32_t *out_my_vehicle_id,
                       const char ***out_peer_ips,
                       int *out_peer_cnt);

#ifdef __cplusplus
}
#endif

#endif /* V2V_TOPOLOGY_H */
