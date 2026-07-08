/*
 * important.c
 * ---------------------------------------------------------------
 *  WIZnet ioLibrary Driver  - 关键代码提取与速查手册
 *  Source : /home/user/下载/ioLibrary_Driver-master
 *  Chips  : W5100 / W5100S / W5200 / W5300 / W5500 / W6100 / W6300
 *
 *  本文件并不是一个能独立编译的工程，而是把整个库中最核心的
 *  宏定义、数据结构、寄存器、API 原型和使用模板汇总到一起，
 *  方便开发时快速参考。
 * ---------------------------------------------------------------
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ===============================================================
 *  1. 目标芯片与接口模式 (来自 wizchip_conf.h)
 * =============================================================== */

/* --- 1.1 选择 WIZCHIP 型号 ---
 *   #define _WIZCHIP_   W5500     // W5100 / W5100S / W5200 / W5300 / W5500 / W6100 / W6300
 */
#define W5100     5100
#define W5100S    5100+5
#define W5200     5200
#define W5300     5300
#define W5500     5500
#define W6100     6100
#define W6300     6300

/* --- 1.2 主机接口模式 ---
 *   库内部根据 _WIZCHIP_ 自动选择，外部仅可重新定义。
 */
#define _WIZCHIP_IO_MODE_NONE_     0x0000
#define _WIZCHIP_IO_MODE_BUS_      0x0100   /* 总线接口 */
#define _WIZCHIP_IO_MODE_SPI_      0x0200   /* SPI  接口 */

#define _WIZCHIP_IO_MODE_BUS_DIR_  (_WIZCHIP_IO_MODE_BUS_ + 1)   /* 直接总线 */
#define _WIZCHIP_IO_MODE_BUS_INDIR_(_WIZCHIP_IO_MODE_BUS_ + 2)   /* 间接总线 */
#define _WIZCHIP_IO_MODE_SPI_VDM_  (_WIZCHIP_IO_MODE_SPI_ + 1)   /* SPI 可变长 */
#define _WIZCHIP_IO_MODE_SPI_FDM_  (_WIZCHIP_IO_MODE_SPI_ + 2)   /* SPI 定长   */
#define _WIZCHIP_IO_MODE_SPI_5500_ (_WIZCHIP_IO_MODE_SPI_ + 3)   /* W5500 风格 */
#define _WIZCHIP_IO_MODE_SPI_QSPI_ (_WIZCHIP_IO_MODE_SPI_ + 4)   /* QSPI       */

/* --- 1.3 PHY 访问方式 --- */
#define _PHY_IO_MODE_PHYCR_        0x0000   /* 寄存器访问 PHY */
#define _PHY_IO_MODE_MII_          0x0010   /* MDC/MDIO 访问  */
#define _PHY_IO_MODE_              _PHY_IO_MODE_MII_

/* 数据宽度（仅 W5300） */
#ifndef _WIZCHIP_IO_BUS_WIDTH_
#define _WIZCHIP_IO_BUS_WIDTH_     16
#endif

/* QSPI 模式（仅 W6300） */
#define QSPI_SINGLE_MODE           (0x00 << 6)
#define QSPI_DUAL_MODE             (0x01 << 6)
#define QSPI_QUAD_MODE             (0x02 << 6)

typedef uint8_t  iodata_t;      /* SPI/8bit 总线数据宽度 */
typedef int16_t  datasize_t;    /* W6100/W6300 数据长度类型 */

/* ===============================================================
 *  2. 缓冲区与协议相关常量
 * =============================================================== */

/* W5500 内部块地址偏移 */
#define WIZCHIP_CREG_BLOCK         0x00
#define WIZCHIP_SREG_BLOCK(N)      (1+4*(N))
#define WIZCHIP_TXBUF_BLOCK(N)     (2+4*(N))
#define WIZCHIP_RXBUF_BLOCK(N)     (3+4*(N))
#define WIZCHIP_OFFSET_INC(ADDR,N) ((ADDR) + ((N)<<8))

/* SPI 控制字 (W5500 风格) */
#define _W5500_SPI_READ_           (0x00 << 2)
#define _W5500_SPI_WRITE_          (0x01 << 2)

/* Socket 模式 (Sn_MR) */
#define Sn_MR_CLOSE                0x00
#define Sn_MR_TCP                  0x01
#define Sn_MR_UDP                  0x02
#define Sn_MR_IPRAW                0x03
#define Sn_MR_MACRAW               0x04
#define Sn_MR_PPPOE                0x05
#define Sn_MR_ND                   0x20   /* W6100/W6300: ND (No Delayed ack) */
#define Sn_MR_MULTI                0x80   /* 多播标志 */

/* Socket 命令 (Sn_CR) */
#define Sn_CR_OPEN                 0x01
#define Sn_CR_LISTEN               0x02
#define Sn_CR_CONNECT              0x04
#define Sn_CR_DISCON               0x08
#define Sn_CR_CLOSE                0x10
#define Sn_CR_SEND                 0x20
#define Sn_CR_SEND_MAC             0x21
#define Sn_CR_SEND_KEEP            0x22
#define Sn_CR_RECV                 0x40

/* Socket 状态 (Sn_SR) */
#define SOCK_CLOSED                0x00
#define SOCK_INIT                  0x13
#define SOCK_LISTEN                0x14
#define SOCK_SYNSENT               0x15
#define SOCK_SYNRECV               0x16
#define SOCK_ESTABLISHED           0x17
#define SOCK_FIN_WAIT              0x18
#define SOCK_CLOSING               0x1A
#define SOCK_TIME_WAIT             0x1B
#define SOCK_CLOSE_WAIT            0x1C
#define SOCK_LAST_ACK              0x1D
#define SOCK_UDP                   0x22
#define SOCK_IPRAW                 0x32
#define SOCK_MACRAW                0x42
#define SOCK_PPPOE                 0x5F

/* Socket 中断标志 (Sn_IR) */
#define Sn_IR_SEND_OK              0x10
#define Sn_IR_TIMEOUT              0x08
#define Sn_IR_RECV                 0x04
#define Sn_IR_DISCON               0x02
#define Sn_IR_CON                  0x01

/* Socket 选项 (getsockopt/setsockopt) */
#define SO_FLAG                    0x00
#define SO_TTL                     0x01
#define SO_TOS                     0x02
#define SO_MSS                     0x03
#define SO_DESTIP                  0x04
#define SO_DESTPORT                0x05
#define SO_KEEPALIVESEND           0x06
#define SO_KEEPALIVEAUTO           0x07
#define SO_SENDBUF                 0x08
#define SO_RECVBUF                 0x09
#define SO_STATUS                  0x0A
#define SO_REMAINED                0x0B
#define SO_PACKINFO                0x0C
#define SO_MODE                    0x0D
#define SO_INT_MASK                0x0E

/* 通用返回值 */
#define SOCK_OK                    1
#define SOCK_BUSY                  0
#define SOCK_FAIL                 -1
#define SOCKERR_NOT_TCP            -2
#define SOCKERR_NOT_UDP            -3
/* ... 其它 SOCKERR_xxx 见 socket.h */

/* 网络模式 (wizchip_setnetmode) */
typedef enum {
    NETINFO_STATIC  = 0,
    NETINFO_DHCP    = 1,
    NETINFO_AUTOIP  = 2,
    NETINFO_DHCP6   = 3,
    NETINFO_AUTOCONF= 4,
} netmode_type;

/* ===============================================================
 *  3. 核心数据结构
 * =============================================================== */

/* 3.1 PHY 配置 */
typedef struct wiz_PhyConf_t {
    uint8_t by;       /* PHY 地址 (一般 0)  */
    uint8_t mode;     /* @ref PHY_CONFBY_HW / _SW  */
    uint8_t speed;    /* PHY_SPEED_10M / _100M / _AUTO */
    uint8_t duplex;   /* PHY_DUPLEX_HALF / _FULL / _AUTO */
} wiz_PhyConf;

/* 3.2 网络信息 */
typedef struct wiz_NetInfo_t {
    uint8_t  mac[6];   /* MAC 地址        */
    uint8_t  ip[4];    /* 本机 IP         */
    uint8_t  sn[4];    /* 子网掩码        */
    uint8_t  gw[4];    /* 默认网关        */
    uint8_t  dns[4];   /* DNS 服务器      */
    dhcp_mode dhcp;    /* NETINFO_STATIC / _DHCP ... */
} wiz_NetInfo;

/* 3.3 Socket 地址 */
typedef struct wiz_sockaddr_t {
    uint8_t  sa_family;   /* AF_INET = 2 等        */
    uint8_t  sa_data[14]; /* port(2) + addr(4) + zero(8) */
} wiz_sockaddr;

/* 3.4 中断相关 */
typedef struct intr_kind {
    uint8_t ir;     /* 公共中断 */
    uint8_t sir;    /* 套接字中断 */
    uint16_t intr_mask;
} intr_kind;

/* 3.5 网路超时 */
typedef struct wiz_NetTimeout_t {
    uint8_t  retry_cnt;     /* 重传次数     */
    uint16_t time_100us;    /* RTR 计数单位 */
} wiz_NetTimeout;

/* 3.6 ctlwizchip / ctlnetwork 命令类型 */
typedef enum {
    CW_RESET_WIZCHIP    = 0x01,
    CW_INIT_WIZCHIP     = 0x02,
    CW_CLR_INTERRUPT    = 0x03,
    CW_GET_INTERRUPT    = 0x04,
    CW_SET_INTRMASK     = 0x05,
    CW_GET_INTRMASK     = 0x06,
    CW_SET_PHYCONF      = 0x07,
    CW_GET_PHYCONF      = 0x08,
    CW_GET_PHYSTATUS    = 0x09,
    CW_RESET_PHY        = 0x0A,
    CW_SET_PHYPOWMODE   = 0x0B,
    CW_GET_PHYPOWMODE   = 0x0C,
} ctlwizchip_type;

typedef enum {
    CN_SET_NETINFO  = 0x01,
    CN_GET_NETINFO  = 0x02,
    CN_SET_NETMODE  = 0x03,
    CN_GET_NETMODE  = 0x04,
    CN_SET_TIMEOUT  = 0x05,
    CN_GET_TIMEOUT  = 0x06,
} ctlnetwork_type;

/* ===============================================================
 *  4. 关键 API 原型  (来自 wizchip_conf.h / socket.h)
 * =============================================================== */

/* ---- 4.1 WIZCHIP 初始化与控制 ---- */
int8_t  wizchip_init(uint8_t* txsize, uint8_t* rxsize);
int8_t  ctlwizchip(ctlwizchip_type cwtype, void* arg);
int8_t  ctlnetwork(ctlnetwork_type cntype, void* arg);

void    wizchip_setnetinfo(wiz_NetInfo* pnetinfo);
void    wizchip_getnetinfo(wiz_NetInfo* pnetinfo);
void    wizchip_setnetmode(netmode_type netmode);
netmode_type wizchip_getnetmode(void);
void    wizchip_settimeout(wiz_NetTimeout* nettime);
void    wizchip_gettimeout(wiz_NetTimeout* nettime);

intr_kind wizchip_getinterrupt(void);
void      wizchip_clrinterrupt(intr_kind interrupt);
intr_kind wizchip_getinterruptmask(void);
void      wizchip_setinterruptmask(intr_kind intr_mask);

void    wizphy_setphyconf(wiz_PhyConf* phyconf);
void    wizphy_getphyconf(wiz_PhyConf* phyconf);
void    wizphy_getphystat(wiz_PhyConf* phyconf);

/* ---- 4.2 用户必须实现的 SPI 回调 ----
 *  以下函数在 wizchip_conf.c 里以 weak/extern 形式声明，
 *  工程里必须自行实现，库才能工作。
 */
void     WIZCHIP_SELECT(void);     /* 片选拉低  */
void     WIZCHIP_DESELECT(void);   /* 片选拉高  */
uint8_t  WIZCHIP_READ(void);       /* 读一字节  */
void     WIZCHIP_WRITE(uint8_t wb);/* 写一字节  */
void     WIZCHIP_READ_BUF(uint8_t* pBuf, uint16_t len);
void     WIZCHIP_WRITE_BUF(uint8_t* pBuf, uint16_t len);

#if (_WIZCHIP_ >= 5500)
uint8_t  WIZCHIP_READ_SPI(uint8_t addr);
void     WIZCHIP_WRITE_SPI(uint8_t addr, uint8_t wb);
void     WIZCHIP_READ_BUF_SPI(uint8_t addr, uint8_t* pBuf, uint16_t len);
void     WIZCHIP_WRITE_BUF_SPI(uint8_t addr, uint8_t* pBuf, uint16_t len);
#endif

/* ---- 4.3 BSD 风格 Socket API (来自 socket.h) ---- */
int8_t   socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag);
int8_t   close(uint8_t sn);
int8_t   listen(uint8_t sn);
int8_t   connect(uint8_t sn, uint8_t* addr, uint16_t port);
int8_t   disconnect(uint8_t sn);
int8_t   send(uint8_t sn, uint8_t* buf, uint16_t len);
int8_t   recv(uint8_t sn, uint8_t* buf, uint16_t len);
int8_t   sendto(uint8_t sn, uint8_t* buf, uint16_t len, uint8_t* addr, uint16_t port);
int8_t   recvfrom(uint8_t sn, uint8_t* buf, uint16_t len, uint8_t* addr, uint16_t* port);

int8_t   getsockopt(uint8_t sn, sockopt_type sotype, void* arg);
int8_t   setsockopt(uint8_t sn, sockopt_type sotype, void* arg);

/* ===============================================================
 *  5. 典型用法模板
 * =============================================================== */

/*
 * ---------------- 5.1 初始化 W5500 (SPI) ----------------
 *   1) 在 wizchip_conf.h 里
 *        #define _WIZCHIP_  W5500
 *        #define _WIZCHIP_IO_MODE_  _WIZCHIP_IO_MODE_SPI_VDM_
 *   2) main.c 中:
 */
static void W5500_Init_Example(void)
{
    /* 8 个 socket 各 2KB */
    uint8_t txsize[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
    uint8_t rxsize[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };

    /* 注册 SPI 回调 */
    reg_wizchip_cs_cbfunc  (WIZCHIP_SELECT, WIZCHIP_DESELECT);
    reg_wizchip_spi_cbfunc (WIZCHIP_READ_SPI, WIZCHIP_WRITE_SPI);
    reg_wizchip_spibuf_cbfunc(WIZCHIP_READ_BUF_SPI,
                              WIZCHIP_WRITE_BUF_SPI);

    /* 初始化 WIZCHIP */
    wizchip_init(txsize, rxsize);

    /* 配置网络信息 */
    wiz_NetInfo netinfo = {
        .mac  = { 0x00, 0x08, 0xDC, 0x01, 0x02, 0x03 },
        .ip   = { 192, 168, 1, 100 },
        .sn   = { 255, 255, 255, 0 },
        .gw   = { 192, 168, 1, 1 },
        .dns  = { 8, 8, 8, 8 },
        .dhcp = NETINFO_STATIC,
    };
    wizchip_setnetinfo(&netinfo);
}

/*
 * ---------------- 5.2 TCP Server 模板 ----------------
 */
static void TCP_Server_Example(void)
{
    uint8_t sock = 0;
    uint8_t buf[1024];

    if (socket(sock, Sn_MR_TCP, 5000, SF_TCP_NODELAY) != sock)
        return;

    if (listen(sock) != SOCK_OK) { close(sock); return; }

    for (;;) {
        uint8_t sr = getSn_SR(sock);
        if (sr == SOCK_ESTABLISHED) {
            int32_t n = recv(sock, buf, sizeof(buf));
            if (n > 0) send(sock, buf, n);
        } else if (sr == SOCK_CLOSED) {
            socket(sock, Sn_MR_TCP, 5000, SF_TCP_NODELAY);
            listen(sock);
        }
    }
}

/*
 * ---------------- 5.3 UDP 模板 ----------------
 */
static void UDP_Example(void)
{
    uint8_t sock = 2;
    uint8_t buf[512];
    uint8_t rip[4];
    uint16_t rport;

    socket(sock, Sn_MR_UDP, 5001, 0);

    for (;;) {
        int32_t n = recvfrom(sock, buf, sizeof(buf), rip, &rport);
        if (n > 0) sendto(sock, buf, n, rip, rport);
    }
}

/* ===============================================================
 *  6. Internet 协议模块入口摘要
 * =============================================================== */

/*
 * 6.1 DHCP 客户端    Internet/DHCP/dhcp.h
 *     DHCP_init(sock, gDATABUF);
 *     DHCP_run();           // 状态机推进
 */
typedef enum { DHCP_FAILED = 0, DHCP_RUNNING, DHCP_IP_ASSIGN, DHCP_IP_CHANGED, DHCP_IP_LEASED } DHCP_STATUS;

/* 6.2 DNS 客户端     Internet/DNS/dns.h
 *     DNS_init(sock, gDATABUF);
 *     DNS_run(gDATABUF, domain_name, ip_from_dns);
 */
typedef enum { DNS_OK = 0, DNS_ERROR, DNS_MAX_TRIES } DNS_STATUS;

/* 6.3 SNTP 客户端   Internet/SNTP/sntp.h
 *     SNTP_init(sock, gDATABUF, ntp_server_ip, port, timeout);
 *     SNTP_run();
 */

/* 6.4 MQTT 客户端   Internet/MQTT/MQTTClient.h
 *     MQTTClientInit(&client, ...);
 *     MQTTConnect(&client, &connectData);
 *     MQTTPublish(&client, topicName, &message);
 *     MQTTYield(&client, timeout_ms);
 */

/* 6.5 HTTP Server   Internet/httpServer/httpServer.h
 *     reg_httpServer_webContent((uint8_t *)"index.html", data, len);
 *     httpServer_init(txsize, rxsize);
 *     while (1) { httpServer_run(); }
 */

/* 6.6 TFTP 客户端   Internet/TFTP/tftp.h
 *     TFTP_init(server_ip, server_port, sock);
 *     TFTP_read_file_request(filename);   // 或 _write
 */

/* 6.7 SNMP Agent    Internet/SNMP/snmp.h
 *     SNMP_init(sock, gDATABUF, snmp_agent);
 *     SNMP_run();
 */

/* 6.8 AAC (SLAAC)   Internet/AAC/AddressAutoConfig.h
 *     AAC_init(...);
 *     AAC_run();
 */

/* 6.9 DHCPv6        Internet/DHCP6/dhcpv6.h
 *     DHCP6_init(sock, ...);
 *     DHCP6_run();
 */

/* ===============================================================
 *  7. 关键目录结构
 * =============================================================== */
/*
 *  ioLibrary_Driver
 *  ├── Application
 *  │   ├── loopback          (loopback.c/.h)
 *  │   └── multicast         (multicast.c/.h)
 *  ├── Ethernet
 *  │   ├── W5100  W5100S  W5200  W5300  W5500  W6100  W6300
 *  │   │       (xxx.c / xxx.h)
 *  │   ├── socket.c / socket.h
 *  │   └── wizchip_conf.c / wizchip_conf.h
 *  └── Internet
 *      ├── AAC          (AddressAutoConfig)
 *      ├── DHCP / DHCP6
 *      ├── DNS
 *      ├── httpServer
 *      ├── MQTT         (MQTTClient + MQTTPacket)
 *      ├── SNMP
 *      ├── SNTP
 *      └── TFTP
 */

/* ===============================================================
 *  8. 其它常用寄存器/字段
 * =============================================================== */

/* 通用寄存器 */
#define MR                0x0000    /* 模式寄存器        */
#define GAR0              0x0001    /* 网关 IP           */
#define SUBR0             0x0005    /* 子网掩码          */
#define SHAR0             0x0009    /* MAC 地址          */
#define SIPR0             0x000F    /* 本机 IP           */
#define IR                0x0015    /* 中断              */
#define IMR               0x0016    /* 中断屏蔽          */
#define RTR0              0x0017    /* 重传时间          */
#define RCR               0x0019    /* 重传次数          */
#define PHYCFGR           0x002E    /* PHY 配置 (W5500)  */
#define VERSIONR          0x0039    /* 芯片版本          */

/* Socket n 寄存器偏移 (相对 Sn_REG_BASE) */
#define Sn_MR             0x0000
#define Sn_CR             0x0001
#define Sn_IR             0x0002
#define Sn_SR             0x0003
#define Sn_PORT0          0x0004
#define Sn_DHAR0          0x0006
#define Sn_DIPR0          0x000C
#define Sn_DPORT0         0x0010
#define Sn_MSSR0          0x0012
#define Sn_TOS            0x0015
#define Sn_TTL            0x0016
#define Sn_RXBUF_SIZE     0x001E
#define Sn_TXBUF_SIZE     0x001F
#define Sn_TX_FSR0        0x0020
#define Sn_TX_RD0         0x0022
#define Sn_TX_WR0         0x0024
#define Sn_RX_RSR0        0x0026
#define Sn_RX_RD0         0x0028
#define Sn_RX_WR0         0x002A
/* …… 其余字段见 w5x00 / w6x00 的头文件 */

/* ===============================================================
 *  9. 简易自检 (编译/链接用, 无实际硬件)
 * =============================================================== */
int important_self_check(void)
{
    printf("=== WIZnet ioLibrary 关键摘要 ===\n");
    printf("W5100=%d W5100S=%d W5200=%d W5300=%d "
           "W5500=%d W6100=%d W6300=%d\n",
           W5100, W5100S, W5200, W5300, W5500, W6100, W6300);

    wiz_NetInfo ni = {0};
    ni.ip[0] = 192; ni.ip[1] = 168; ni.ip[2] = 1; ni.ip[3] = 100;
    ni.dhcp  = NETINFO_STATIC;

    printf("默认 IP : %d.%d.%d.%d\n", ni.ip[0], ni.ip[1], ni.ip[2], ni.ip[3]);
    printf("SOCK_ESTABLISHED=0x%02X  SOCK_CLOSED=0x%02X\n",
           SOCK_ESTABLISHED, SOCK_CLOSED);
    return 0;
}

/* ===============================================================
 *  10. 关键源文件清单
 * =============================================================== */
/*
 *  Ethernet/wizchip_conf.c      芯片配置 / 协议无关 API 实现
 *  Ethernet/socket.c            BSD Socket API 实现
 *  Ethernet/W5500/w5500.c       W5500 寄存器访问
 *  Ethernet/W6100/w6100.c       W6100 寄存器访问
 *  Ethernet/W6300/w6300.c       W6300 寄存器访问
 *  Internet/DHCP/dhcp.c         DHCP 客户端
 *  Internet/DHCP6/dhcpv6.c      DHCPv6 客户端
 *  Internet/DNS/dns.c           DNS 客户端
 *  Internet/SNTP/sntp.c         SNTP 客户端
 *  Internet/MQTT/MQTTClient.c   MQTT 客户端
 *  Internet/httpServer/*.c      HTTP 服务器
 *  Internet/TFTP/tftp.c         TFTP 客户端
 *  Internet/SNMP/snmp.c         SNMP Agent
 *  Internet/AAC/AddressAutoConfig.c  IPv6 SLAAC
 *  Application/loopback/*.c     TCP/UDP 回环示例
 *  Application/multicast/*.c    组播收发示例
 */
