/**
 * @file wiznet_socket.c
 * @brief wiz_ 前缀的 ioLibrary 套接字 API 包装实现
 *
 * 解决 ioLibrary 函数与 lwip/BSD 同名 static inline 冲突问题.
 * 本文件先 #include "socket.h" (ioLibrary), 然后定义 wiz_ 前缀的
 * 转发函数, 供应用层使用.
 */

#include "wiznet_socket.h"

/* ioLibrary 的 socket.h 在本文件中必须放在所有可能引入 lwip/sockets.h
 * 的头文件之前, 否则 lwip 的 static inline 会污染本翻译单元. */
#include "socket.h"

int8_t wiz_socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag)
{
    return socket(sn, protocol, port, flag);
}

int8_t wiz_close(uint8_t sn)
{
    return close(sn);
}

int8_t wiz_listen(uint8_t sn)
{
    return listen(sn);
}

int8_t wiz_disconnect(uint8_t sn)
{
    return disconnect(sn);
}

int32_t wiz_send(uint8_t sn, uint8_t *buf, uint16_t len)
{
    return send(sn, buf, len);
}

int32_t wiz_recv(uint8_t sn, uint8_t *buf, uint16_t len)
{
    return recv(sn, buf, len);
}

int32_t wiz_sendto(uint8_t sn, uint8_t *buf, uint16_t len,
                   uint8_t *addr, uint16_t port)
{
    return sendto(sn, buf, len, addr, port);
}

int32_t wiz_recvfrom(uint8_t sn, uint8_t *buf, uint16_t len,
                     uint8_t *addr, uint16_t *port)
{
    return recvfrom(sn, buf, len, addr, port);
}
