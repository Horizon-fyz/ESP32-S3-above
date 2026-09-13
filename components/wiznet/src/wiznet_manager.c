/**
 * @file wiznet_manager.c
 * @brief W5500 + ioLibrary 网络管理器实现
 */

#include "wiznet_manager.h"
#include "wiznet_spi.h"
#include "wiznet_socket.h"   /* wiz_close 等命名空间包装 */
#include "wiznet_conf.h"
#include "wizchip_conf.h"
#include "../Ethernet/socket.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "wiznet_mgr";

static wiz_NetInfo s_net_info = {0};
static bool        s_initialized = false;

/* ========== 默认配置 ========== */
wiznet_manager_config_t wiznet_manager_get_default_config(void)
{
    wiznet_manager_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* 板载 W5500 (v5.10 换板 ESP32-S3-ETH: 引脚由板卡固定, 不再用外置模块) */
    cfg.spi_sck_gpio  = 13;    /* ETH_CLK  */
    cfg.spi_mosi_gpio = 11;    /* ETH_MOSI */
    cfg.spi_miso_gpio = 12;    /* ETH_MISO */
    cfg.spi_cs_gpio   = 14;    /* ETH_CS   */
    cfg.spi_rst_gpio  = 9;     /* ETH_RST (独立 GPIO, 可硬复位) */
    cfg.spi_int_gpio  = 10;    /* ETH_INT, 中断驱动 */

    cfg.spi_host     = SPI2_HOST;
    /* W5500 SPI 时钟最高 33.3MHz (datasheet), 但 polling 模式下受 CPU 速率限制.
     * 改用 SPI_DMA_DISABLED (polling) 后, 实际最高 ~20MHz.
     * W5500 寄存器访问量小, 20MHz 完全够用. */
    cfg.spi_clock_hz = 20 * 1000 * 1000;   /* 20 MHz (polling 模式) */

    /* 默认静态 IP, 主流路由器网段 */
    cfg.ip[0]      = 192; cfg.ip[1]      = 168; cfg.ip[2]      = 29;   cfg.ip[3]      = 10;
    cfg.netmask[0] = 255; cfg.netmask[1] = 255; cfg.netmask[2] = 255;  cfg.netmask[3] = 0;
    cfg.gateway[0] = 192; cfg.gateway[1] = 168; cfg.gateway[2] = 29;   cfg.gateway[3] = 1;
    cfg.dns[0]     = 8;   cfg.dns[1]     = 8;   cfg.dns[2]     = 8;    cfg.dns[3]     = 8;

    /* 8 socket 共享 16KB SRAM, 各 2KB (默认配置) */
    for (int i = 0; i < WIZNET_SOCK_MAX; i++) {
        cfg.tx_buf_size_kb[i] = 2;
        cfg.rx_buf_size_kb[i] = 2;
    }

    return cfg;
}

/* ========== 初始化 ========== */
esp_err_t wiznet_manager_init(const wiznet_manager_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "W5500 初始化中 (TOE 模式)...");

    /* 1. 初始化 SPI 总线
     *
     * ⚠️ 使用 SPI_DMA_DISABLED (polling 模式) 而非 DMA 模式
     *    原因: ESP-IDF v5.5 的 SPI master 驱动在 DMA 模式下, 每次 transaction
     *          都会调用 heap_caps_aligned_alloc 分配 DMA 描述符/缓冲区.
     *          W5500 状态查询 (getSn_SR 等) 高频调用 1 字节 SPI 传输时,
     *          反复分配/释放会导致堆碎片化, 最终阻塞在 malloc 上,
     *          触发 Task WDT (30s 超时). Backtrace 实测确认此问题.
     *
     *          改用 polling 后:
     *            - 无动态内存分配, 不会有碎片化
     *            - SPI 时钟降到 20MHz (CPU polling 速率上限)
     *            - W5500 寄存器访问 1~4 字节, polling 完全够用 (延迟 < 1µs/字节)
     *            - 大数据突发 (wiz_recv 2KB) 也 < 1ms, 满足 50Hz 控制需求
     */
    spi_bus_config_t buscfg = {
        .miso_io_num   = cfg->spi_miso_gpio,
        .mosi_io_num   = cfg->spi_mosi_gpio,
        .sclk_io_num   = cfg->spi_sck_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t ret = spi_bus_initialize(cfg->spi_host, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. 初始化 SPI 桥接层 (注册 ioLibrary 回调) */
    wiznet_spi_config_t spi_cfg = {
        .spi_host = cfg->spi_host,
        .cs_gpio  = cfg->spi_cs_gpio,
        .rst_gpio = cfg->spi_rst_gpio,
        .int_gpio = cfg->spi_int_gpio,
        .clock_hz = cfg->spi_clock_hz,
    };
    ret = wiznet_spi_init(&spi_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 3. 初始化 WIZCHIP (配置 8 个 socket 的 TX/RX buffer) */
    uint8_t txsize[WIZNET_SOCK_MAX], rxsize[WIZNET_SOCK_MAX];
    for (int i = 0; i < WIZNET_SOCK_MAX; i++) {
        txsize[i] = cfg->tx_buf_size_kb[i];
        rxsize[i] = cfg->rx_buf_size_kb[i];
    }
    ret = wizchip_init(txsize, rxsize);
    if (ret != 0) {
        ESP_LOGE(TAG, "wizchip_init failed: %d", ret);
        return ESP_FAIL;
    }

    /* 4. 配置网络信息 */
    memcpy(s_net_info.mac,  cfg->mac,     6);
    memcpy(s_net_info.ip,   cfg->ip,      4);
    memcpy(s_net_info.sn,   cfg->netmask, 4);
    memcpy(s_net_info.gw,   cfg->gateway, 4);
    memcpy(s_net_info.dns,  cfg->dns,     4);
    s_net_info.dhcp = NETINFO_STATIC;

    /* MAC 全 0 时使用默认地址 */
    bool mac_all_zero = true;
    for (int i = 0; i < 6; i++) {
        if (s_net_info.mac[i] != 0) { mac_all_zero = false; break; }
    }
    if (mac_all_zero) {
        s_net_info.mac[0] = 0x02;
        s_net_info.mac[1] = 0x00;
        s_net_info.mac[2] = 0x00;
        s_net_info.mac[3] = 0x00;
        s_net_info.mac[4] = 0x00;
        s_net_info.mac[5] = 0x01;
    }
    wizchip_setnetinfo(&s_net_info);

    /* 5. 配置 PHY: 软件强制 100M FULL
     *
     * 与项目参数文档一致 (PHY 模式 = 100M FULL).
     * 通过 PHYCR 寄存器直接设置, 无需 MDC/MDIO. */
    wiz_PhyConf phyconf = {
        .by     = PHY_CONFBY_SW,
        .mode   = PHY_MODE_MANUAL,
        .speed  = PHY_SPEED_100,
        .duplex = PHY_DUPLEX_FULL,
    };
    wizphy_setphyconf(&phyconf);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 6. 软件复位 W5500 确保状态干净 */
    wizchip_sw_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    /* 复位后会清空 netinfo 和 phy 配置, 重新设置 */
    wizchip_setnetinfo(&s_net_info);
    wizphy_setphyconf(&phyconf);

    /* 6. 验证芯片 */
    wizchip_getnetinfo(&s_net_info);
    /* 不再重复打印 IP, 留给 app_main 统一输出 */

    s_initialized = true;
    return ESP_OK;
}

void wiznet_manager_deinit(void)
{
    if (!s_initialized) return;
    /* 关闭所有 socket (使用 ioLibrary 的 close, 避免与 lwip 命名冲突) */
    for (uint8_t sn = 0; sn < WIZNET_SOCK_MAX; sn++) {
        wiz_close(sn);
    }
    /* 释放 SPI 设备 */
    spi_device_handle_t dev = wiznet_spi_get_device();
    if (dev) {
        spi_bus_remove_device(dev);
    }
    s_initialized = false;
    ESP_LOGI(TAG, "W5500 deinitialized");
}

/* ========== 状态查询 ========== */
bool wiznet_manager_wait_for_link(uint32_t timeout_ms)
{
    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        if (wizphy_getphylink() == PHY_LINK_ON) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

bool wiznet_manager_is_link_up(void)
{
    if (!s_initialized) {
        return false;   /* 防御: 避免在 W5500 未初始化时访问导致崩溃 */
    }
    return wizphy_getphylink() == PHY_LINK_ON;
}

uint8_t wiznet_manager_get_link_speed(void)
{
    wiz_PhyConf phyconf;
    wizphy_getphystat(&phyconf);
    return (phyconf.speed == PHY_SPEED_100) ? 100 : 10;
}

esp_err_t wiznet_manager_get_ip_info(esp_netif_ip_info_t *ip_info)
{
    if (ip_info == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized)  return ESP_ERR_INVALID_STATE;

    memset(ip_info, 0, sizeof(*ip_info));
    /* esp_netif_ip_info_t 字段为 uint32_t, 网络字节序 */
    ip_info->ip.addr      = ((uint32_t)s_net_info.ip[0]) |
                            ((uint32_t)s_net_info.ip[1] << 8) |
                            ((uint32_t)s_net_info.ip[2] << 16) |
                            ((uint32_t)s_net_info.ip[3] << 24);
    ip_info->netmask.addr = ((uint32_t)s_net_info.sn[0]) |
                            ((uint32_t)s_net_info.sn[1] << 8) |
                            ((uint32_t)s_net_info.sn[2] << 16) |
                            ((uint32_t)s_net_info.sn[3] << 24);
    ip_info->gw.addr      = ((uint32_t)s_net_info.gw[0]) |
                            ((uint32_t)s_net_info.gw[1] << 8) |
                            ((uint32_t)s_net_info.gw[2] << 16) |
                            ((uint32_t)s_net_info.gw[3] << 24);
    return ESP_OK;
}

void *wiznet_manager_get_wizchip(void)
{
    return (void *)&WIZCHIP;
}
