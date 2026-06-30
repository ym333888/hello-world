// SPDX-License-Identifier: GPL-2.0
/*
 * agn_spi.c - AGN lightweight SPI0 driver for MT6739 (kernel 4.14)
 *
 * 设计目标: 小数据量(<=32字节)、高频次(几百次/秒)场景下的低开销SPI驱动
 *
 * 优化策略:
 *   1. probe/resume时预配置所有静态寄存器(CFG0/CFG2/CMD)
 *   2. 仅使用FIFO模式，避免DMA配置开销
 *   3. 每次传输仅执行: 写TX数据 + 更新CFG1(packet长度) + 启动 + 等中断 + 读RX数据
 *   4. 使用completion等待中断，避免忙等消耗CPU
 *
 * DTS配置说明:
 *   需要禁用标准spi0节点或修改compatible避免冲突，并添加本驱动节点:
 *
 *   agn_spi0: agn-spi@1100a000 {
 *       compatible = "agn,mt6739-spi0";
 *       reg = <0 0x1100a000 0 0x1000>;
 *       interrupts = <GIC_SPI 118 IRQ_TYPE_LEVEL_LOW>;
 *       clocks = <&topckgen CLK_TOP_SYSPLL3_D2>,
 *                <&topckgen CLK_TOP_SPI_SEL>,
 *                <&infracfg_ao CLK_INFRA_SPI0>;
 *       clock-names = "parent-clk", "sel-clk", "spi-clk";
 *       agn,spi-speed = <1000000>;  // 可选, 默认1MHz
 *       agn,spi-mode = <0>;        // 可选, SPI mode 0-3
 *       status = "okay";
 *   };
 *
 *   同时将原spi0节点设置 status = "disabled" 以避免驱动冲突:
 *   spi0: spi@1100a000 {
 *       ...
 *       status = "disabled";
 *   };
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include "agn_spi.h"

/* ======================== 寄存器定义 ======================== */

#define SPI_CFG0_REG		0x0000
#define SPI_CFG1_REG		0x0004
#define SPI_TX_SRC_REG		0x0008
#define SPI_RX_DST_REG		0x000c
#define SPI_TX_DATA_REG		0x0010
#define SPI_RX_DATA_REG		0x0014
#define SPI_CMD_REG		0x0018
#define SPI_STATUS0_REG		0x001c
#define SPI_STATUS1_REG		0x0020
#define SPI_PAD_SEL_REG		0x0024
#define SPI_CFG2_REG		0x0028
#define SPI_TX_SRC_REG_64	0x002c
#define SPI_RX_DST_REG_64	0x0030

/* CFG0 位域 (enhance_timing模式, MT6739使用) */
#define SPI_CFG0_CS_HOLD_OFFSET		0
#define SPI_CFG0_CS_SETUP_OFFSET	16

/* CFG2 位域 (enhance_timing模式) */
#define SPI_CFG2_SCK_HIGH_OFFSET	0
#define SPI_CFG2_SCK_LOW_OFFSET		16

/* CFG1 位域 (通用) */
#define SPI_CFG1_CS_IDLE_OFFSET		0
#define SPI_CFG1_PACKET_LOOP_OFFSET	8
#define SPI_CFG1_PACKET_LENGTH_OFFSET	16

#define SPI_CFG1_CS_IDLE_MASK		0xff
#define SPI_CFG1_PACKET_LOOP_MASK	0xff00
#define SPI_CFG1_PACKET_LENGTH_MASK	0x3ff0000

/* CMD 位定义 */
#define SPI_CMD_ACT		BIT(0)
#define SPI_CMD_RESUME		BIT(1)
#define SPI_CMD_RST		BIT(2)
#define SPI_CMD_PAUSE_EN	BIT(4)
#define SPI_CMD_DEASSERT	BIT(5)
#define SPI_CMD_SAMPLE_SEL	BIT(6)
#define SPI_CMD_CS_POL		BIT(7)
#define SPI_CMD_CPHA		BIT(8)
#define SPI_CMD_CPOL		BIT(9)
#define SPI_CMD_RX_DMA		BIT(10)
#define SPI_CMD_TX_DMA		BIT(11)
#define SPI_CMD_TXMSBF		BIT(12)
#define SPI_CMD_RXMSBF		BIT(13)
#define SPI_CMD_RX_ENDIAN	BIT(14)
#define SPI_CMD_TX_ENDIAN	BIT(15)
#define SPI_CMD_FINISH_IE	BIT(16)
#define SPI_CMD_PAUSE_IE	BIT(17)

/* STATUS0 */
#define SPI_PAUSE_INT_STATUS	BIT(1)

/* FIFO大小限制 */
#define MTK_SPI_MAX_FIFO_SIZE	32

/* ======================== 驱动私有数据 ======================== */

struct agn_spi_data {
	void __iomem		*base;		/* SPI0寄存器基地址 */
	struct clk		*parent_clk;	/* 父时钟 */
	struct clk		*sel_clk;	/* 选择时钟 */
	struct clk		*spi_clk;	/* SPI工作时钟 */
	int			irq;		/* 中断号 */
	struct completion	done;		/* 传输完成量 */
	struct mutex		lock;		/* 互斥锁(防并发) */
	spinlock_t		irq_lock;	/* 中断锁 */
	struct agn_spi_config	cfg;		/* 当前配置 */
	u32			spi_clk_hz;	/* 缓存的SPI时钟频率 */
	bool			ready;		/* 驱动就绪标志 */

	/* 预计算的CFG1基础值(不含packet_length和packet_loop) */
	u32			cfg1_base;
	/* 预计算的CFG0值 */
	u32			cfg0_val;
	/* 预计算的CFG2值 */
	u32			cfg2_val;
	/* 预计算的CMD值(不含ACT/RESUME/DMA位) */
	u32			cmd_base;
};

static struct agn_spi_data *g_spi;  /* 全局实例(仅SPI0一个) */

/* ======================== 日志控制 ======================== */

#define AGN_SPI_DBG	0  /* 设为1开启调试日志 */

#if AGN_SPI_DBG
#define spi_dbg(fmt, ...) pr_info("[agn_spi] %s: " fmt, __func__, ##__VA_ARGS__)
#else
#define spi_dbg(fmt, ...) do {} while (0)
#endif

#define spi_err(fmt, ...) pr_err("[agn_spi] %s: " fmt, __func__, ##__VA_ARGS__)
#define spi_info(fmt, ...) pr_info("[agn_spi] " fmt, ##__VA_ARGS__)

/* ======================== 寄存器操作辅助 ======================== */

static inline u32 agn_readl(struct agn_spi_data *d, u32 reg)
{
	return readl(d->base + reg);
}

static inline void agn_writel(struct agn_spi_data *d, u32 val, u32 reg)
{
	writel(val, d->base + reg);
}

/* ======================== SPI复位 ======================== */

static void agn_spi_hw_reset(struct agn_spi_data *d)
{
	u32 val = agn_readl(d, SPI_CMD_REG);

	val |= SPI_CMD_RST;
	agn_writel(d, val, SPI_CMD_REG);

	val &= ~SPI_CMD_RST;
	agn_writel(d, val, SPI_CMD_REG);
}

/* ======================== 寄存器预配置 ======================== */

/*
 * 根据当前config计算并预配置所有静态寄存器
 * 在probe和resume时调用
 */
static int agn_spi_preconfigure(struct agn_spi_data *d)
{
	u32 spi_clk_hz, div, sck_time, cs_time;
	u16 cs_setup, cs_hold, cs_idle;
	u32 cfg0, cfg1, cfg2, cmd;

	/* 获取SPI时钟频率 */
	spi_clk_hz = clk_get_rate(d->spi_clk);
	d->spi_clk_hz = spi_clk_hz;
	spi_info("spi_clk = %u Hz, target speed = %u Hz\n",
		 spi_clk_hz, d->cfg.speed_hz);

	/* 计算分频和时序参数 */
	if (d->cfg.speed_hz < spi_clk_hz / 2)
		div = DIV_ROUND_UP(spi_clk_hz, d->cfg.speed_hz);
	else
		div = 1;

	sck_time = (div + 1) / 2;
	cs_time = sck_time * 2;

	/* CS时序: 用户指定值 > 自动计算值 */
	cs_setup = d->cfg.cs_setuptime ? d->cfg.cs_setuptime : (u16)cs_time;
	cs_hold  = d->cfg.cs_holdtime  ? d->cfg.cs_holdtime  : (u16)cs_time;
	cs_idle  = d->cfg.cs_idletime  ? d->cfg.cs_idletime  : (u16)cs_time;

	/* ---- 计算CFG2 (enhance_timing模式) ---- */
	/* CFG2[15:0]  = SCK高电平时间-1 */
	/* CFG2[31:16] = SCK低电平时间-1 */
	cfg2 = (((sck_time - 1) & 0xffff) << SPI_CFG2_SCK_HIGH_OFFSET)
	     | (((sck_time - 1) & 0xffff) << SPI_CFG2_SCK_LOW_OFFSET);

	/* ---- 计算CFG0 (enhance_timing模式) ---- */
	/* CFG0[15:0]  = CS hold时间-1 */
	/* CFG0[31:16] = CS setup时间-1 */
	cfg0 = (((cs_hold - 1) & 0xffff) << SPI_CFG0_CS_HOLD_OFFSET)
	     | (((cs_setup - 1) & 0xffff) << SPI_CFG0_CS_SETUP_OFFSET);

	/* ---- 计算CFG1基础值 ---- */
	/* CFG1[7:0]   = CS idle时间-1 */
	/* CFG1[15:8]  = packet_loop (运行时更新) */
	/* CFG1[25:16] = packet_length (运行时更新) */
	cfg1 = ((cs_idle - 1) & 0xff) << SPI_CFG1_CS_IDLE_OFFSET;

	/* ---- 计算CMD基础值 ---- */
	cmd = 0;

	/* 时钟相位/极性 */
	if (d->cfg.cpha)
		cmd |= SPI_CMD_CPHA;
	if (d->cfg.cpol)
		cmd |= SPI_CMD_CPOL;

	/* 位序 */
	if (d->cfg.tx_mlsb)
		cmd |= SPI_CMD_TXMSBF;
	if (d->cfg.rx_mlsb)
		cmd |= SPI_CMD_RXMSBF;

	/* 端序: ARM是小端 */
	cmd &= ~SPI_CMD_TX_ENDIAN;
	cmd &= ~SPI_CMD_RX_ENDIAN;

	/* CS极性和采样选择 */
	if (d->cfg.cs_pol)
		cmd |= SPI_CMD_CS_POL;
	if (d->cfg.sample_sel)
		cmd |= SPI_CMD_SAMPLE_SEL;

	/* 使能完成和暂停中断 */
	cmd |= SPI_CMD_FINISH_IE | SPI_CMD_PAUSE_IE;

	/* 禁用DMA (仅使用FIFO) */
	cmd &= ~(SPI_CMD_TX_DMA | SPI_CMD_RX_DMA);

	/* CS deassert模式: 关闭 */
	cmd &= ~SPI_CMD_DEASSERT;

	/* 保存预计算值 */
	d->cfg0_val = cfg0;
	d->cfg1_base = cfg1;
	d->cfg2_val = cfg2;
	d->cmd_base = cmd;

	/* ---- 写入静态寄存器 ---- */
	agn_writel(d, cfg0, SPI_CFG0_REG);
	agn_writel(d, cfg2, SPI_CFG2_REG);

	/* CFG1先写基础值, packet部分运行时更新 */
	agn_writel(d, cfg1, SPI_CFG1_REG);

	/* CMD: 先写基础值, 带PAUSE_EN(CS deassert) */
	agn_writel(d, cmd | SPI_CMD_PAUSE_EN, SPI_CMD_REG);

	/* 硬件复位到IDLE状态 */
	agn_spi_hw_reset(d);

	spi_info("preconfig done: CFG0=0x%08x CFG1=0x%08x CFG2=0x%08x CMD=0x%08x\n",
		 cfg0, cfg1, cfg2, cmd);

	return 0;
}

/* ======================== 中断处理 ======================== */

static irqreturn_t agn_spi_interrupt(int irq, void *dev_id)
{
	struct agn_spi_data *d = dev_id;
	u32 status;

	spin_lock(&d->irq_lock);

	status = agn_readl(d, SPI_STATUS0_REG);
	spi_dbg("IRQ: STATUS0=0x%08x\n", status);

	complete(&d->done);

	spin_unlock(&d->irq_lock);
	return IRQ_HANDLED;
}

/* ======================== 核心传输函数 ======================== */

/*
 * agn_spi_transfer - 全双工SPI传输 (核心热路径)
 *
 * 最小IO路径 (~6-8次寄存器访问):
 *   1. 清除PAUSE_EN (CS active)       - 1次写
 *   2. 写TX_DATA (按4字节块+余数)      - 1~2次写
 *   3. 更新CFG1 packet_length          - 1次写(读-改-写)
 *   4. 设置ACT启动传输                 - 1次写
 *   5. 等待中断completion              - 无IO
 *   6. 读RX_DATA (按4字节块+余数)      - 1~2次读
 *   7. 设置PAUSE_EN (CS deassert)      - 1次写
 */
int agn_spi_transfer(const void *tx_buf, void *rx_buf, u32 len)
{
	struct agn_spi_data *d = g_spi;
	u32 cnt, remainder, reg_val;
	int ret;
	unsigned long timeout;

	if (!d || !d->ready) {
		spi_err("driver not ready\n");
		return -ENODEV;
	}
	if (len == 0 || len > MTK_SPI_MAX_FIFO_SIZE) {
		spi_err("invalid length %u (max %d)\n", len, MTK_SPI_MAX_FIFO_SIZE);
		return -EINVAL;
	}

	mutex_lock(&d->lock);

	reinit_completion(&d->done);
	agn_spi_hw_reset(d);

	/* Step 1: CS active - 清除PAUSE_EN, 同时确保无ACT */
	reg_val = d->cmd_base;  /* 不含PAUSE_EN, 不含ACT */
	agn_writel(d, reg_val, SPI_CMD_REG);

	/* Step 2: 写TX数据到FIFO */
	cnt = len / 4;
	remainder = len % 4;

	if (tx_buf) {
		if (cnt > 0)
			iowrite32_rep(d->base + SPI_TX_DATA_REG, tx_buf, cnt);
		if (remainder > 0) {
			u32 tmp = 0;
			memcpy(&tmp, (const u8 *)tx_buf + (cnt * 4), remainder);
			agn_writel(d, tmp, SPI_TX_DATA_REG);
		}
	} else {
		/* 无TX数据, 发送dummy (0x00) */
		u32 dummy_cnt = (len + 3) / 4;
		u32 i;
		for (i = 0; i < dummy_cnt; i++)
			agn_writel(d, 0, SPI_TX_DATA_REG);
	}

	/* Step 3: 更新CFG1的packet_length (packet_loop=0, 即1次) */
	/* 对于 <=32字节, packet_length=len, packet_loop=1 */
	reg_val = d->cfg1_base;
	reg_val |= ((len - 1) & 0x3ff) << SPI_CFG1_PACKET_LENGTH_OFFSET;
	/* packet_loop = 1, 即 (1-1)=0, 无需设置 */
	agn_writel(d, reg_val, SPI_CFG1_REG);

	/* Step 4: 启动传输 - 设置ACT位 */
	reg_val = agn_readl(d, SPI_CMD_REG);
	reg_val |= SPI_CMD_ACT;
	agn_writel(d, reg_val, SPI_CMD_REG);

	spi_dbg("transfer started: len=%u\n", len);

	/* Step 5: 等待中断 */
	timeout = wait_for_completion_timeout(&d->done,
					      msecs_to_jiffies(AGN_SPI_TIMEOUT_MS));
	if (timeout == 0) {
		spi_err("transfer timeout! len=%u STATUS0=0x%08x CMD=0x%08x\n",
			len, agn_readl(d, SPI_STATUS0_REG),
			agn_readl(d, SPI_CMD_REG));
		agn_spi_hw_reset(d);
		ret = -ETIMEDOUT;
		goto out;
	}

	/* Step 6: 读RX数据 */
	if (rx_buf) {
		cnt = len / 4;
		remainder = len % 4;

		if (cnt > 0)
			ioread32_rep(d->base + SPI_RX_DATA_REG, rx_buf, cnt);
		if (remainder > 0) {
			reg_val = agn_readl(d, SPI_RX_DATA_REG);
			memcpy((u8 *)rx_buf + (cnt * 4), &reg_val, remainder);
		}
	}

	/* Step 7: CS deassert - 设置PAUSE_EN */
	reg_val = d->cmd_base | SPI_CMD_PAUSE_EN;
	agn_writel(d, reg_val, SPI_CMD_REG);

	ret = 0;
	spi_dbg("transfer done: len=%u OK\n", len);

out:
	mutex_unlock(&d->lock);
	return ret;
}
EXPORT_SYMBOL(agn_spi_transfer);

int agn_spi_write(const void *tx_buf, u32 len)
{
	return agn_spi_transfer(tx_buf, NULL, len);
}
EXPORT_SYMBOL(agn_spi_write);

int agn_spi_read(void *rx_buf, u32 len)
{
	return agn_spi_transfer(NULL, rx_buf, len);
}
EXPORT_SYMBOL(agn_spi_read);

int agn_spi_write_then_read(const void *tx_buf, u32 tx_len,
			    void *rx_buf, u32 rx_len)
{
	int ret;

	ret = agn_spi_transfer(tx_buf, NULL, tx_len);
	if (ret)
		return ret;

	return agn_spi_transfer(NULL, rx_buf, rx_len);
}
EXPORT_SYMBOL(agn_spi_write_then_read);

/* ======================== 运行时重配置 ======================== */

int agn_spi_reconfigure(const struct agn_spi_config *cfg)
{
	struct agn_spi_data *d = g_spi;
	int ret;

	if (!d || !d->ready)
		return -ENODEV;

	mutex_lock(&d->lock);

	if (cfg)
		d->cfg = *cfg;
	else {
		struct agn_spi_config def = AGN_SPI_DEFAULT_CONFIG;
		d->cfg = def;
	}

	ret = agn_spi_preconfigure(d);

	mutex_unlock(&d->lock);
	return ret;
}
EXPORT_SYMBOL(agn_spi_reconfigure);

int agn_spi_is_ready(void)
{
	return (g_spi && g_spi->ready) ? 1 : 0;
}
EXPORT_SYMBOL(agn_spi_is_ready);

/* ======================== sysfs调试接口 ======================== */

static ssize_t spi_dump_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct agn_spi_data *d = dev_get_drvdata(dev);
	int n = 0;

	if (!d)
		return 0;

	n += scnprintf(buf + n, PAGE_SIZE - n,
		"=== AGN SPI0 Register Dump ===\n"
		"CFG0   = 0x%08x\n"
		"CFG1   = 0x%08x\n"
		"CFG2   = 0x%08x\n"
		"CMD    = 0x%08x\n"
		"STATUS0= 0x%08x\n"
		"STATUS1= 0x%08x\n"
		"spi_clk= %u Hz\n"
		"speed  = %u Hz\n"
		"mode   = CPOL=%d CPHA=%d\n"
		"ready  = %d\n",
		agn_readl(d, SPI_CFG0_REG),
		agn_readl(d, SPI_CFG1_REG),
		agn_readl(d, SPI_CFG2_REG),
		agn_readl(d, SPI_CMD_REG),
		agn_readl(d, SPI_STATUS0_REG),
		agn_readl(d, SPI_STATUS1_REG),
		d->spi_clk_hz,
		d->cfg.speed_hz,
		d->cfg.cpol, d->cfg.cpha,
		d->ready);

	return n;
}
static DEVICE_ATTR_RO(spi_dump);

static ssize_t spi_speed_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct agn_spi_data *d = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n", d ? d->cfg.speed_hz : 0);
}

static ssize_t spi_speed_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct agn_spi_data *d = dev_get_drvdata(dev);
	u32 speed;

	if (!d)
		return -ENODEV;
	if (kstrtou32(buf, 10, &speed) || speed == 0)
		return -EINVAL;

	d->cfg.speed_hz = speed;
	agn_spi_reconfigure(&d->cfg);

	return count;
}
static DEVICE_ATTR_RW(spi_speed);

static ssize_t spi_mode_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct agn_spi_data *d = dev_get_drvdata(dev);
	u8 mode = 0;

	if (d) {
		if (d->cfg.cpol)
			mode |= 2;
		if (d->cfg.cpha)
			mode |= 1;
	}
	return scnprintf(buf, PAGE_SIZE, "%u\n", mode);
}

static ssize_t spi_mode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct agn_spi_data *d = dev_get_drvdata(dev);
	u8 mode;

	if (!d)
		return -ENODEV;
	if (kstrtou8(buf, 10, &mode) || mode > 3)
		return -EINVAL;

	d->cfg.cpol = (mode >> 1) & 1;
	d->cfg.cpha = mode & 1;
	agn_spi_reconfigure(&d->cfg);

	return count;
}
static DEVICE_ATTR_RW(spi_mode);

static struct attribute *agn_spi_attrs[] = {
	&dev_attr_spi_dump.attr,
	&dev_attr_spi_speed.attr,
	&dev_attr_spi_mode.attr,
	NULL,
};

static struct attribute_group agn_spi_attr_group = {
	.name = "agn_spi",
	.attrs = agn_spi_attrs,
};

/* ======================== 时钟管理 ======================== */

static int agn_spi_clk_enable(struct agn_spi_data *d)
{
	int ret;

	ret = clk_prepare_enable(d->spi_clk);
	if (ret) {
		spi_err("failed to enable spi_clk: %d\n", ret);
		return ret;
	}

	ret = clk_set_parent(d->sel_clk, d->parent_clk);
	if (ret) {
		spi_err("failed to set parent clk: %d\n", ret);
		clk_disable_unprepare(d->spi_clk);
		return ret;
	}

	return 0;
}

static void agn_spi_clk_disable(struct agn_spi_data *d)
{
	clk_disable_unprepare(d->spi_clk);
}

/* ======================== Platform Driver ======================== */

static int agn_spi_probe(struct platform_device *pdev)
{
	struct agn_spi_data *d;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	struct agn_spi_config def_cfg = AGN_SPI_DEFAULT_CONFIG;
	u32 spi_mode = 0;
	int ret;

	spi_info("probe start\n");

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	/* 初始化基础结构 */
	mutex_init(&d->lock);
	spin_lock_init(&d->irq_lock);
	init_completion(&d->done);
	d->cfg = def_cfg;

	/* 读取DTS可选属性 */
	of_property_read_u32(np, "agn,spi-speed", &d->cfg.speed_hz);
	of_property_read_u32(np, "agn,spi-mode", &spi_mode);
	d->cfg.cpol = (spi_mode >> 1) & 1;
	d->cfg.cpha = spi_mode & 1;

	/* 映射寄存器 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		spi_err("no memory resource\n");
		return -ENODEV;
	}
	d->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(d->base)) {
		spi_err("ioremap failed\n");
		return PTR_ERR(d->base);
	}

	/* 获取中断 */
	d->irq = platform_get_irq(pdev, 0);
	if (d->irq < 0) {
		spi_err("no IRQ resource\n");
		return d->irq;
	}

	/* 获取时钟 */
	d->parent_clk = devm_clk_get(&pdev->dev, "parent-clk");
	if (IS_ERR(d->parent_clk)) {
		spi_err("failed to get parent-clk\n");
		return PTR_ERR(d->parent_clk);
	}
	d->sel_clk = devm_clk_get(&pdev->dev, "sel-clk");
	if (IS_ERR(d->sel_clk)) {
		spi_err("failed to get sel-clk\n");
		return PTR_ERR(d->sel_clk);
	}
	d->spi_clk = devm_clk_get(&pdev->dev, "spi-clk");
	if (IS_ERR(d->spi_clk)) {
		spi_err("failed to get spi-clk\n");
		return PTR_ERR(d->spi_clk);
	}

	/* 使能时钟 */
	ret = agn_spi_clk_enable(d);
	if (ret)
		return ret;

	/* 注册中断 */
	ret = devm_request_irq(&pdev->dev, d->irq, agn_spi_interrupt,
			       IRQF_TRIGGER_NONE, "agn-spi0", d);
	if (ret) {
		spi_err("request IRQ %d failed: %d\n", d->irq, ret);
		goto err_clk;
	}

	/* 预配置所有寄存器 */
	ret = agn_spi_preconfigure(d);
	if (ret)
		goto err_clk;

	/* 创建sysfs调试接口 */
	dev_set_drvdata(&pdev->dev, d);
	ret = sysfs_create_group(&pdev->dev.kobj, &agn_spi_attr_group);
	if (ret)
		spi_info("sysfs create failed: %d (non-fatal)\n", ret);

	/* 设置全局指针, 标记就绪 */
	g_spi = d;
	d->ready = true;

	spi_info("probe OK: base=%pK irq=%d speed=%uHz mode=%d\n",
		 d->base, d->irq, d->cfg.speed_hz, spi_mode);

	return 0;

err_clk:
	agn_spi_clk_disable(d);
	return ret;
}

static int agn_spi_remove(struct platform_device *pdev)
{
	struct agn_spi_data *d = dev_get_drvdata(&pdev->dev);

	if (!d)
		return 0;

	d->ready = false;
	g_spi = NULL;

	sysfs_remove_group(&pdev->dev.kobj, &agn_spi_attr_group);
	agn_spi_hw_reset(d);
	agn_spi_clk_disable(d);

	spi_info("removed\n");
	return 0;
}

/* ======================== 电源管理 ======================== */

#ifdef CONFIG_PM_SLEEP
static int agn_spi_suspend(struct device *dev)
{
	struct agn_spi_data *d = dev_get_drvdata(dev);

	if (!d)
		return 0;

	d->ready = false;
	agn_spi_hw_reset(d);
	agn_spi_clk_disable(d);

	spi_info("suspended\n");
	return 0;
}

static int agn_spi_resume(struct device *dev)
{
	struct agn_spi_data *d = dev_get_drvdata(dev);
	int ret;

	if (!d)
		return 0;

	/* 重新使能时钟 */
	ret = agn_spi_clk_enable(d);
	if (ret)
		return ret;

	/* 重新预配置所有寄存器 */
	ret = agn_spi_preconfigure(d);
	if (ret) {
		agn_spi_clk_disable(d);
		return ret;
	}

	d->ready = true;
	spi_info("resumed\n");
	return 0;
}
#endif

static const struct dev_pm_ops agn_spi_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(agn_spi_suspend, agn_spi_resume)
};

/* ======================== DTS匹配 ======================== */

static const struct of_device_id agn_spi_of_match[] = {
	{ .compatible = "agn,mt6739-spi0", },
	{ },
};
MODULE_DEVICE_TABLE(of, agn_spi_of_match);

static struct platform_driver agn_spi_driver = {
	.driver = {
		.name = "agn-spi",
		.owner = THIS_MODULE,
		.of_match_table = agn_spi_of_match,
		.pm = &agn_spi_pm,
	},
	.probe = agn_spi_probe,
	.remove = agn_spi_remove,
};
module_platform_driver(agn_spi_driver);

MODULE_DESCRIPTION("AGN lightweight SPI0 driver for MT6739");
MODULE_AUTHOR("AGN");
MODULE_LICENSE("GPL v2");
