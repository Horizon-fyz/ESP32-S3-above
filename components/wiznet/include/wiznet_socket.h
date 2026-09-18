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
/* ⚠️ ioLibrary 的 socket.h 里声明了 `int8_t close(uint8_t sn);` —— 与 newlib 的
 *    `int close(int)` (经 <unistd.h> 带进来, main.c 的 linenoise.h 就会引到它)
 *    是同名不同签名, 同一翻译单元里同时可见会报:
 *      error: conflicting types for 'close'; have 'int(int)'
 *    这里在包含 ioLibrary 头期间把该名字临时改掉 (只影响这次 include 出来的**声明**),
 *    出栈后立即恢复 —— 于是存在的是 POSIX 的 close, 应用层一律用 wiz_close() 关闭 socket。
 *    注: socket.h 内部并不调用 close(), 所以改名不影响它自身内容。 */
#define close iolib_socket_close_decl_
#include "../Ethernet/socket.h"  /* 透传 ioLibrary 常量/状态/寄存器宏 */
#undef close

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
