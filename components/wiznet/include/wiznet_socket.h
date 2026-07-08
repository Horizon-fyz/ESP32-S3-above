/**
 * @file wiznet_socket.h
 * @brief W5500 套接字 API 命名空间包装 (wiz_ 前缀)
 *
 * ioLibrary 的 socket/listen/send/recv/close 与 lwip/BSD 的同名,
 * 在 ESP-IDF 工程中会因 static inline 冲突导致编译失败.
 * 本头文件以 wiz_ 前缀重新导出 ioLibrary 符号, 用户应使用:
 *
 *   wiz_socket(sn, proto, port, flag)
 *   wiz_listen(sn)
 *   wiz_send(sn, buf, len) / wiz_recv(sn, buf, len)
 *   wiz_close(sn)
 *
 * 同时透传 ioLibrary 的状态/常量宏 (SOCK_ESTABLISHED, SOCK_BUSY, SOCK_OK,
 * SF_TCP_NODELAY, Sn_MR_TCP, getSn_SR 等), 供应用层使用.
 */

#ifndef WIZNET_SOCKET_H_
#define WIZNET_SOCKET_H_

#include <stdint.h>
#include "wiznet_conf.h"
#include "wizchip_conf.h"
/* 使用带目录前缀的相对路径, 避免与 lwip 的 <sys/socket.h> 冲突 */
#include "../Ethernet/socket.h"  /* 透传 ioLibrary 常量/状态/寄存器宏 */

#ifdef __cplusplus
extern "C" {
#endif

/* 重新导出与 lwip 冲突的 8 个 API 为 wiz_ 前缀. */
int8_t  wiz_socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag);
int8_t  wiz_close(uint8_t sn);
int8_t  wiz_listen(uint8_t sn);
int8_t  wiz_disconnect(uint8_t sn);
int32_t wiz_send(uint8_t sn, uint8_t *buf, uint16_t len);
int32_t wiz_recv(uint8_t sn, uint8_t *buf, uint16_t len);
int32_t wiz_sendto(uint8_t sn, uint8_t *buf, uint16_t len,
                   uint8_t *addr, uint16_t port);
int32_t wiz_recvfrom(uint8_t sn, uint8_t *buf, uint16_t len,
                     uint8_t *addr, uint16_t *port);

#ifdef __cplusplus
}
#endif

#endif /* WIZNET_SOCKET_H_ */
