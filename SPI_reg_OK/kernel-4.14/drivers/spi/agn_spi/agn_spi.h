/* SPDX-License-Identifier: GPL-2.0 */
/*
 * agn_spi.h - AGN lightweight SPI0 driver for MT6739
 *
 * 针对小数据量、高频次SPI通信场景优化的专属驱动。
 * 开机/唤醒时预配置所有静态寄存器，运行时仅需最少IO操作完成收发。
 */

#ifndef _AGN_SPI_H
#define _AGN_SPI_H

#include <linux/types.h>

/*
 * SPI配置参数结构体
 * 在驱动初始化时设置，运行时可通过 agn_spi_reconfigure() 修改
 */
struct agn_spi_config {
	u32 speed_hz;		/* SPI时钟频率, 默认1MHz */
	u8  cpol;		/* 时钟极性: 0=空闲低, 1=空闲高 */
	u8  cpha;		/* 时钟相位: 0=第一边沿采样, 1=第二边沿采样 */
	u8  tx_mlsb;		/* TX位序: 1=MSB优先, 0=LSB优先 */
	u8  rx_mlsb;		/* RX位序: 1=MSB优先, 0=LSB优先 */
	u8  cs_pol;		/* CS极性: 0=低有效, 1=高有效 */
	u8  sample_sel;		/* 采样选择 */
	u16 cs_setuptime;	/* CS建立时间(时钟周期), 0=自动 */
	u16 cs_holdtime;	/* CS保持时间(时钟周期), 0=自动 */
	u16 cs_idletime;	/* CS空闲时间(时钟周期), 0=自动 */
};

/* 默认配置 */
#define AGN_SPI_DEFAULT_CONFIG { \
	.speed_hz     = 1000000, \
	.cpol         = 0, \
	.cpha         = 0, \
	.tx_mlsb      = 1, \
	.rx_mlsb      = 1, \
	.cs_pol       = 0, \
	.sample_sel   = 0, \
	.cs_setuptime = 0, \
	.cs_holdtime  = 0, \
	.cs_idletime  = 0, \
}

/* 最大FIFO传输长度 */
#define AGN_SPI_MAX_FIFO_SIZE	32

/* 传输超时(毫秒) */
#define AGN_SPI_TIMEOUT_MS	1000

/*
 * agn_spi_transfer - 全双工SPI传输
 * @tx_buf: 发送缓冲区, NULL则发送dummy数据
 * @rx_buf: 接收缓冲区, NULL则丢弃接收数据
 * @len:    传输字节数 (1~32)
 *
 * 返回值: 0成功, 负数错误码
 *
 * 这是最核心的接口，IO操作已最小化:
 *   写TX_DATA(1-2次) + 更新CFG1(1次) + 设ACT(1次) + 等中断 + 读RX_DATA(1-2次)
 */
int agn_spi_transfer(const void *tx_buf, void *rx_buf, u32 len);

/*
 * agn_spi_write - 仅发送数据
 * @tx_buf: 发送缓冲区
 * @len:    发送字节数 (1~32)
 */
int agn_spi_write(const void *tx_buf, u32 len);

/*
 * agn_spi_read - 仅接收数据 (发送dummy字节)
 * @rx_buf: 接收缓冲区
 * @len:    接收字节数 (1~32)
 */
int agn_spi_read(void *rx_buf, u32 len);

/*
 * agn_spi_write_then_read - 先发后收 (两段式)
 * @tx_buf: 发送缓冲区
 * @tx_len: 发送字节数
 * @rx_buf: 接收缓冲区
 * @rx_len: 接收字节数
 *
 * 注意: 这会发起两次SPI传输(先发后收), CS在两次之间会deassert
 * 适用于"发命令+读响应"的场景
 */
int agn_spi_write_then_read(const void *tx_buf, u32 tx_len,
			    void *rx_buf, u32 rx_len);

/*
 * agn_spi_reconfigure - 运行时重新配置SPI参数
 * @cfg: 新配置, NULL则恢复默认
 *
 * 用于运行时切换SPI速率/模式等参数
 */
int agn_spi_reconfigure(const struct agn_spi_config *cfg);

/*
 * agn_spi_is_ready - 检查驱动是否就绪
 *
 * 返回值: 1就绪, 0未就绪
 */
int agn_spi_is_ready(void);

#endif /* _AGN_SPI_H */
