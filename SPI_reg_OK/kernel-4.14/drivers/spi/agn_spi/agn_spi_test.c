// SPDX-License-Identifier: GPL-2.0
/*
 * agn_spi_test.c - 测试驱动 for agn_spi
 *
 * 功能:
 *   1. 创建 /dev/agn_spi_test 设备节点
 *   2. 通过 ioctl 设置参数并触发循环SPI收发测试
 *   3. 通过 ioctl 读取测试结果(次数/成功率/耗时)
 *   4. 支持同步(阻塞)和异步(内核线程)两种测试模式
 *
 * 使用方式:
 *   加载: insmod agn_spi_test.ko
 *   卸载: rmmod agn_spi_test
 *
 *   用户空间测试程序通过 ioctl 与驱动交互:
 *     AGN_SPI_TEST_START   - 启动测试(同步,阻塞直到完成)
 *     AGN_SPI_TEST_START_ASYNC - 启动异步测试(内核线程)
 *     AGN_SPI_TEST_STOP    - 停止异步测试
 *     AGN_SPI_TEST_STATUS  - 查询测试状态/结果
 *     AGN_SPI_TEST_SET_CFG - 设置测试参数
 *     AGN_SPI_TEST_SINGLE  - 单次收发(可指定TX数据)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/math64.h>

#include "agn_spi.h"

/* ======================== ioctl定义 ======================== */

#define AGN_SPI_TEST_MAGIC	'A'

/* 测试参数结构体 */
struct agn_spi_test_param {
	__u32 mode;		/* 测试模式: 0=全双工, 1=仅TX, 2=仅RX, 3=write_then_read */
	__u32 count;		/* 循环次数, 0=无限(仅异步模式可用) */
	__u32 tx_len;		/* TX数据长度 (1~32) */
	__u32 rx_len;		/* RX数据长度 (1~32), write_then_read模式用 */
	__u32 delay_us;		/* 每次传输间隔(微秒), 0=无间隔 */
	__u32 tx_pattern;	/* TX填充模式: 0=递增, 1=固定值, 2=全0xFF */
	__u8  tx_fixed_val;	/* tx_pattern=1时的固定值 */
	__u32 speed_hz;		/* SPI速率, 0=不改 */
	__u8  spi_mode;		/* SPI mode 0~3, 0xFF=不改 */
};

/* 测试结果结构体 */
struct agn_spi_test_result {
	__u32 state;		/* 0=idle, 1=running, 2=done, 3=error */
	__u32 total;		/* 总请求次数 */
	__u32 success;		/* 成功次数 */
	__u32 fail;		/* 失败次数 */
	__u64 total_time_us;	/* 总耗时(微秒) */
	__u64 min_time_us;	/* 最小单次耗时 */
	__u64 max_time_us;	/* 最大单次耗时 */
	__u64 avg_time_us;	/* 平均单次耗时 */
	__u32 last_err;		/* 最后一次错误码 */
	__u8  last_rx[32];	/* 最后一次接收的数据 */
	__u32 last_rx_len;	/* 最后接收数据长度 */
};

/* 单次收发结构体 */
struct agn_spi_test_single {
	__u32 tx_len;
	__u32 rx_len;		/* 0=仅发送 */
	__u8  tx_data[32];
	__u8  rx_data[32];
	__s32 ret;		/* 返回值 */
};

#define AGN_SPI_TEST_SET_CFG	_IOW(AGN_SPI_TEST_MAGIC, 1, struct agn_spi_test_param)
#define AGN_SPI_TEST_START	_IOWR(AGN_SPI_TEST_MAGIC, 2, struct agn_spi_test_result)
#define AGN_SPI_TEST_START_ASYNC _IOW(AGN_SPI_TEST_MAGIC, 3, struct agn_spi_test_param)
#define AGN_SPI_TEST_STOP	_IO(AGN_SPI_TEST_MAGIC, 4)
#define AGN_SPI_TEST_STATUS	_IOR(AGN_SPI_TEST_MAGIC, 5, struct agn_spi_test_result)
#define AGN_SPI_TEST_SINGLE	_IOWR(AGN_SPI_TEST_MAGIC, 6, struct agn_spi_test_single)

/* ======================== 驱动私有数据 ======================== */

struct agn_spi_test_data {
	struct mutex lock;

	/* 测试参数 */
	struct agn_spi_test_param param;

	/* 测试结果 */
	struct agn_spi_test_result result;

	/* 异步测试线程 */
	struct task_struct *thread;
	int stop_flag;
};

static struct agn_spi_test_data *g_test;

/* ======================== 时间辅助 ======================== */
static unsigned long get_time_us(void)
{
    ktime_t now = ktime_get();
    return ktime_to_us(now);      // 直接返回微秒数
}
/* ======================== 填充TX数据 ======================== */

static void fill_tx_data(u8 *buf, u32 len, u32 pattern, u8 fixed_val, u32 seq)
{
	u32 i;

	switch (pattern) {
	case 0: /* 递增 */
		for (i = 0; i < len; i++)
			buf[i] = (u8)((seq + i) & 0xFF);
		break;
	case 1: /* 固定值 */
		memset(buf, fixed_val, len);
		break;
	case 2: /* 全0xFF */
		memset(buf, 0xFF, len);
		break;
	default:
		for (i = 0; i < len; i++)
			buf[i] = (u8)((seq + i) & 0xFF);
		break;
	}
}

/* ======================== 单次传输执行 ======================== */

static int do_one_transfer(struct agn_spi_test_param *param,
			   u8 *tx_buf, u8 *rx_buf, u32 seq)
{
	int ret;

	fill_tx_data(tx_buf, param->tx_len, param->tx_pattern,
		     param->tx_fixed_val, seq);

	switch (param->mode) {
	case 0: /* 全双工 */
		ret = agn_spi_transfer(tx_buf, rx_buf, param->tx_len);
		break;
	case 1: /* 仅TX */
		ret = agn_spi_write(tx_buf, param->tx_len);
		break;
	case 2: /* 仅RX */
		ret = agn_spi_read(rx_buf, param->rx_len > 0 ? param->rx_len : param->tx_len);
		break;
	case 3: /* write_then_read */
		ret = agn_spi_write_then_read(tx_buf, param->tx_len,
					      rx_buf, param->rx_len);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* ======================== 同步测试(阻塞) ======================== */

static int run_sync_test(struct agn_spi_test_data *td)
{
	struct agn_spi_test_param *p = &td->param;
	struct agn_spi_test_result *r = &td->result;
	u8 tx_buf[32] = {0};
	u8 rx_buf[32] = {0};
	u64 t_start, t_single;
	u32 i;
	int ret;

	if (!agn_spi_is_ready()) {
		pr_err("[agn_spi_test] agn_spi driver not ready\n");
		return -ENODEV;
	}

	if (p->tx_len == 0 || p->tx_len > 32) {
		pr_err("[agn_spi_test] invalid tx_len %u\n", p->tx_len);
		return -EINVAL;
	}
	if (p->mode == 3 && (p->rx_len == 0 || p->rx_len > 32)) {
		pr_err("[agn_spi_test] invalid rx_len %u for write_then_read\n", p->rx_len);
		return -EINVAL;
	}

	/* 可选: 修改SPI速率/模式 */
	if (p->speed_hz > 0 || (p->spi_mode != 0xFF)) {
		struct agn_spi_config cfg;
		/* 读当前配置再修改, 这里用默认值作为基础 */
		memset(&cfg, 0, sizeof(cfg));
		cfg.speed_hz = p->speed_hz > 0 ? p->speed_hz : 1000000;
		if (p->spi_mode != 0xFF) {
			cfg.cpol = (p->spi_mode >> 1) & 1;
			cfg.cpha = p->spi_mode & 1;
		}
		cfg.tx_mlsb = 1;
		cfg.rx_mlsb = 1;
		agn_spi_reconfigure(&cfg);
	}

	/* 初始化结果 */
	memset(r, 0, sizeof(*r));
	r->state = 1; /* running */
	r->total = p->count;
	r->min_time_us = ~0ULL;

	pr_info("[agn_spi_test] sync test start: mode=%u count=%u tx_len=%u delay=%uus\n",
		p->mode, p->count, p->tx_len, p->delay_us);

	t_start = get_time_us();

	for (i = 0; i < p->count; i++) {
		u64 t1 = get_time_us();

		ret = do_one_transfer(p, tx_buf, rx_buf, i);

		t_single = get_time_us() - t1;

		if (ret == 0) {
			r->success++;
			/* 统计耗时 */
			r->total_time_us += t_single;
			if (t_single < r->min_time_us)
				r->min_time_us = t_single;
			if (t_single > r->max_time_us)
				r->max_time_us = t_single;
		} else {
			r->fail++;
			r->last_err = (u32)ret;
		}

		/* 保存最后一次RX数据 */
		if (rx_buf[0] || rx_buf[1]) {
			u32 copy_len = (p->mode == 3) ? p->rx_len : p->tx_len;
			if (copy_len > 32)
				copy_len = 32;
			memcpy(r->last_rx, rx_buf, copy_len);
			r->last_rx_len = copy_len;
		}

		if (p->delay_us > 0)
			udelay(p->delay_us);
	}

	r->total_time_us = get_time_us() - t_start;
	r->state = 2; /* done */

	if (r->success)
		r->avg_time_us = div_u64(r->total_time_us, r->success);
	else
		r->avg_time_us = 0;   // 或其他合理的默认值

	pr_info("[agn_spi_test] sync test done: total=%u ok=%u fail=%u "
		"time=%lluus avg=%lluus min=%lluus max=%lluus\n",
		r->total, r->success, r->fail,
		r->total_time_us, r->avg_time_us,
		r->min_time_us, r->max_time_us);

	return 0;
}

/* ======================== 异步测试线程 ======================== */

static int async_test_thread(void *data)
{
	struct agn_spi_test_data *td = data;
	struct agn_spi_test_param *p = &td->param;
	struct agn_spi_test_result *r = &td->result;
	u8 tx_buf[32] = {0};
	u8 rx_buf[32] = {0};
	u64 t1, t_single;
	u32 seq = 0;
	int ret;

	pr_info("[agn_spi_test] async thread started: mode=%u count=%u tx_len=%u\n",
		p->mode, p->count, p->tx_len);

	r->state = 1; /* running */

	while (!kthread_should_stop() && !td->stop_flag) {
		/* 检查是否达到目标次数 (count=0表示无限) */
		if (p->count > 0 && seq >= p->count)
			break;

		t1 = get_time_us();
		ret = do_one_transfer(p, tx_buf, rx_buf, seq);
		t_single = get_time_us() - t1;

		if (ret == 0) {
			r->success++;
			r->total_time_us += t_single;
			if (t_single < r->min_time_us)
				r->min_time_us = t_single;
			if (t_single > r->max_time_us)
				r->max_time_us = t_single;
		} else {
			r->fail++;
			r->last_err = (u32)ret;
			r->state = 3; /* error */
			break;
		}

		/* 保存最后一次RX */
		{
			u32 copy_len = (p->mode == 3) ? p->rx_len : p->tx_len;
			if (copy_len == 0)
				copy_len = p->tx_len;
			if (copy_len > 32)
				copy_len = 32;
			memcpy(r->last_rx, rx_buf, copy_len);
			r->last_rx_len = copy_len;
		}

		seq++;

		if (p->delay_us > 0) {
			/* 使用可中断延迟, 以便能快速响应stop */
			usleep_range(p->delay_us, p->delay_us + 100);
		}
	}

	if (r->state == 1)
		r->state = 2; /* done (正常结束) */

	r->total = seq;
	if (r->success)
		r->avg_time_us = div_u64(r->total_time_us, r->success);
	else
		r->avg_time_us = 0;   // 或其他合理的默认值

	pr_info("[agn_spi_test] async thread done: total=%u ok=%u fail=%u\n",
		r->total, r->success, r->fail);

	return 0;
}

/* ======================== 文件操作 ======================== */

static int agn_spi_test_open(struct inode *inode, struct file *file)
{
	file->private_data = g_test;
	return 0;
}

static int agn_spi_test_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long agn_spi_test_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	struct agn_spi_test_data *td = file->private_data;
	int ret = 0;

	if (!td)
		return -ENODEV;

	switch (cmd) {

	case AGN_SPI_TEST_SET_CFG: {
		struct agn_spi_test_param p;
		if (copy_from_user(&p, (void __user *)arg, sizeof(p)))
			return -EFAULT;

		mutex_lock(&td->lock);

		/* 检查是否正在运行 */
		if (td->result.state == 1) {
			pr_err("[agn_spi_test] test is running, stop first\n");
			mutex_unlock(&td->lock);
			return -EBUSY;
		}

		/* 参数校验 */
		if (p.tx_len == 0 || p.tx_len > 32) {
			mutex_unlock(&td->lock);
			return -EINVAL;
		}
		if (p.mode > 3) {
			mutex_unlock(&td->lock);
			return -EINVAL;
		}

		td->param = p;
		pr_info("[agn_spi_test] config set: mode=%u count=%u tx_len=%u "
			"delay=%uus pattern=%u speed=%u spi_mode=%u\n",
			p.mode, p.count, p.tx_len, p.delay_us,
			p.tx_pattern, p.speed_hz, p.spi_mode);

		mutex_unlock(&td->lock);
		break;
	}

	case AGN_SPI_TEST_START: {
		struct agn_spi_test_result res;

		mutex_lock(&td->lock);

		/* 检查是否正在运行 */
		if (td->result.state == 1 || td->thread) {
			pr_err("[agn_spi_test] test is running, stop first\n");
			mutex_unlock(&td->lock);
			return -EBUSY;
		}

		/* 检查agn_spi就绪 */
		if (!agn_spi_is_ready()) {
			pr_err("[agn_spi_test] agn_spi driver not ready\n");
			mutex_unlock(&td->lock);
			return -ENODEV;
		}

		/* 从用户空间读取参数(可选覆盖) */
		if (arg) {
			struct agn_spi_test_param p;
			if (copy_from_user(&p, (void __user *)arg, sizeof(p))) {
				mutex_unlock(&td->lock);
				return -EFAULT;
			}
			td->param = p;
		}

		/* 同步执行 */
		ret = run_sync_test(td);
		res = td->result;

		mutex_unlock(&td->lock);

		/* 返回结果到用户空间 */
		if (copy_to_user((void __user *)arg, &res, sizeof(res)))
			return -EFAULT;
		break;
	}

	case AGN_SPI_TEST_START_ASYNC: {
		struct agn_spi_test_param p;

		if (arg) {
			if (copy_from_user(&p, (void __user *)arg, sizeof(p)))
				return -EFAULT;
		}

		mutex_lock(&td->lock);

		if (td->result.state == 1 || td->thread) {
			pr_err("[agn_spi_test] test is running, stop first\n");
			mutex_unlock(&td->lock);
			return -EBUSY;
		}

		if (!agn_spi_is_ready()) {
			pr_err("[agn_spi_test] agn_spi driver not ready\n");
			mutex_unlock(&td->lock);
			return -ENODEV;
		}

		if (arg)
			td->param = p;

		/* 初始化结果 */
		memset(&td->result, 0, sizeof(td->result));
		td->result.state = 1;
		td->result.min_time_us = ~0ULL;
		td->stop_flag = 0;

		/* 可选: 修改SPI速率/模式 */
		if (td->param.speed_hz > 0 || (td->param.spi_mode != 0xFF)) {
			struct agn_spi_config cfg;
			memset(&cfg, 0, sizeof(cfg));
			cfg.speed_hz = td->param.speed_hz > 0 ? td->param.speed_hz : 1000000;
			if (td->param.spi_mode != 0xFF) {
				cfg.cpol = (td->param.spi_mode >> 1) & 1;
				cfg.cpha = td->param.spi_mode & 1;
			}
			cfg.tx_mlsb = 1;
			cfg.rx_mlsb = 1;
			agn_spi_reconfigure(&cfg);
		}

		td->thread = kthread_run(async_test_thread, td, "agn_spi_test");
		if (IS_ERR(td->thread)) {
			ret = PTR_ERR(td->thread);
			td->thread = NULL;
			td->result.state = 3;
			pr_err("[agn_spi_test] kthread_run failed: %d\n", ret);
		}

		mutex_unlock(&td->lock);
		break;
	}

	case AGN_SPI_TEST_STOP: {
		struct task_struct *t;

		mutex_lock(&td->lock);
		t = td->thread;
		if (!t) {
			pr_info("[agn_spi_test] no running test\n");
			mutex_unlock(&td->lock);
			break;
		}
		td->stop_flag = 1;
		mutex_unlock(&td->lock);

		kthread_stop(t);

		mutex_lock(&td->lock);
		td->thread = NULL;
		mutex_unlock(&td->lock);

		pr_info("[agn_spi_test] async test stopped\n");
		break;
	}

	case AGN_SPI_TEST_STATUS: {
		struct agn_spi_test_result res;

		mutex_lock(&td->lock);
		res = td->result;
		mutex_unlock(&td->lock);

		if (copy_to_user((void __user *)arg, &res, sizeof(res)))
			return -EFAULT;
		break;
	}

	case AGN_SPI_TEST_SINGLE: {
		struct agn_spi_test_single s;
		u8 tx_buf[32] = {0};
		u8 rx_buf[32] = {0};

		if (copy_from_user(&s, (void __user *)arg, sizeof(s)))
			return -EFAULT;

		if (s.tx_len == 0 || s.tx_len > 32)
			return -EINVAL;

		mutex_lock(&td->lock);

		if (td->result.state == 1 || td->thread) {
			mutex_unlock(&td->lock);
			return -EBUSY;
		}

		memcpy(tx_buf, s.tx_data, s.tx_len);

		if (s.rx_len > 0 && s.rx_len <= 32) {
			s.ret = agn_spi_write_then_read(tx_buf, s.tx_len,
							rx_buf, s.rx_len);
			memcpy(s.rx_data, rx_buf, s.rx_len);
		} else {
			s.ret = agn_spi_transfer(tx_buf, rx_buf, s.tx_len);
			memcpy(s.rx_data, rx_buf, s.tx_len);
			s.rx_len = s.tx_len;
		}

		mutex_unlock(&td->lock);

		if (copy_to_user((void __user *)arg, &s, sizeof(s)))
			return -EFAULT;

		print_hex_dump(KERN_INFO, "[agn_spi_test] TX: ",
			       DUMP_PREFIX_NONE, 16, 1,
			       s.tx_data, s.tx_len, false);
		if (s.ret == 0)
			print_hex_dump(KERN_INFO, "[agn_spi_test] RX: ",
				       DUMP_PREFIX_NONE, 16, 1,
				       s.rx_data, s.rx_len, false);
		break;
	}

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

/* ======================== 设备文件操作 ======================== */

static const struct file_operations agn_spi_test_fops = {
	.owner = THIS_MODULE,
	.open = agn_spi_test_open,
	.release = agn_spi_test_release,
	.unlocked_ioctl = agn_spi_test_ioctl,
	.compat_ioctl = agn_spi_test_ioctl,
};

/* ======================== misc设备注册 ======================== */

static struct miscdevice agn_spi_test_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "agn_spi_test",
	.fops = &agn_spi_test_fops,
};

/* ======================== 模块初始化/退出 ======================== */

static int __init agn_spi_test_init(void)
{
	int ret;

	g_test = kzalloc(sizeof(*g_test), GFP_KERNEL);
	if (!g_test)
		return -ENOMEM;

	mutex_init(&g_test->lock);

	/* 默认参数 */
	g_test->param.mode = 0;        /* 全双工 */
	g_test->param.count = 100;     /* 100次 */
	g_test->param.tx_len = 4;      /* 4字节 */
	g_test->param.rx_len = 4;
	g_test->param.delay_us = 0;    /* 无间隔 */
	g_test->param.tx_pattern = 0;  /* 递增 */
	g_test->param.speed_hz = 0;    /* 不改 */
	g_test->param.spi_mode = 0xFF; /* 不改 */

	/* 初始化结果 */
	g_test->result.state = 0; /* idle */

	ret = misc_register(&agn_spi_test_dev);
	if (ret) {
		pr_err("[agn_spi_test] misc_register failed: %d\n", ret);
		kfree(g_test);
		g_test = NULL;
		return ret;
	}

	pr_info("[agn_spi_test] loaded, device: /dev/agn_spi_test\n");
	return 0;
}

static void __exit agn_spi_test_exit(void)
{
	if (!g_test)
		return;

	/* 停止正在运行的异步测试 */
	if (g_test->thread) {
		g_test->stop_flag = 1;
		kthread_stop(g_test->thread);
		g_test->thread = NULL;
	}

	misc_deregister(&agn_spi_test_dev);
	kfree(g_test);
	g_test = NULL;

	pr_info("[agn_spi_test] unloaded\n");
}

module_init(agn_spi_test_init);
module_exit(agn_spi_test_exit);

MODULE_DESCRIPTION("AGN SPI test driver");
MODULE_AUTHOR("AGN");
MODULE_LICENSE("GPL v2");
