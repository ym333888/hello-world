/*
 * agn_spi_test_app.c - 用户空间测试程序 for agn_spi驱动
 *
 * 编译: gcc -o agn_spi_test_app agn_spi_test_app.c
 * 使用: ./agn_spi_test_app [命令] [参数]
 *
 * 命令:
 *   sync [次数] [长度] [延迟us]  - 同步循环测试
 *   async [次数] [长度] [延迟us] - 异步循环测试(后台)
 *   stop                         - 停止异步测试
 *   status                       - 查看测试结果
 *   single <hex_bytes>           - 单次发送指定数据
 *   wtr <tx_hex> <rx_len>       - write_then_read测试
 *
 * 示例:
 *   ./agn_spi_test_app sync 1000 4 0       # 1000次,4字节,无间隔
 *   ./agn_spi_test_app sync 100 8 100      # 100次,8字节,100us间隔
 *   ./agn_spi_test_app async 0 4 1000      # 无限次,4字节,1ms间隔(后台跑)
 *   ./agn_spi_test_app stop                # 停止
 *   ./agn_spi_test_app status              # 看结果
 *   ./agn_spi_test_app single AA BB CC DD  # 发送4字节
 *   ./agn_spi_test_app wtr 80 00 8        # 发0x80 0x00后读8字节
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <time.h>

/* ======================== 从内核头文件复制的定义 ======================== */

#define AGN_SPI_TEST_MAGIC	'A'

struct agn_spi_test_param {
	uint32_t mode;		/* 0=全双工, 1=仅TX, 2=仅RX, 3=write_then_read */
	uint32_t count;
	uint32_t tx_len;
	uint32_t rx_len;
	uint32_t delay_us;
	uint32_t tx_pattern;	/* 0=递增, 1=固定值, 2=全0xFF */
	uint8_t  tx_fixed_val;
	uint32_t speed_hz;	/* 0=不改 */
	uint8_t  spi_mode;	/* 0xFF=不改 */
};

struct agn_spi_test_result {
	uint32_t state;		/* 0=idle, 1=running, 2=done, 3=error */
	uint32_t total;
	uint32_t success;
	uint32_t fail;
	uint64_t total_time_us;
	uint64_t min_time_us;
	uint64_t max_time_us;
	uint64_t avg_time_us;
	uint32_t last_err;
	uint8_t  last_rx[32];
	uint32_t last_rx_len;
};

struct agn_spi_test_single {
	uint32_t tx_len;
	uint32_t rx_len;
	uint8_t  tx_data[32];
	uint8_t  rx_data[32];
	int32_t  ret;
};

#define AGN_SPI_TEST_SET_CFG	_IOW(AGN_SPI_TEST_MAGIC, 1, struct agn_spi_test_param)
#define AGN_SPI_TEST_START	_IOWR(AGN_SPI_TEST_MAGIC, 2, struct agn_spi_test_result)
#define AGN_SPI_TEST_START_ASYNC _IOW(AGN_SPI_TEST_MAGIC, 3, struct agn_spi_test_param)
#define AGN_SPI_TEST_STOP	_IO(AGN_SPI_TEST_MAGIC, 4)
#define AGN_SPI_TEST_STATUS	_IOR(AGN_SPI_TEST_MAGIC, 5, struct agn_spi_test_result)
#define AGN_SPI_TEST_SINGLE	_IOWR(AGN_SPI_TEST_MAGIC, 6, struct agn_spi_test_single)

/* 手动定义_IOW/_IOR/_IOWR (某些NDK环境没有) */
#ifndef _IOW
#define _IOC_NRBITS   8
#define _IOC_TYPEBITS 8
#define _IOC_SIZEBITS 14
#define _IOC_DIRBITS  2
#define _IOC_NRSHIFT  0
#define _IOC_TYPESHIFT (_IOC_NRSHIFT + _IOC_NRBITS)
#define _IOC_SIZESHIFT (_IOC_TYPESHIFT + _IOC_TYPEBITS)
#define _IOC_DIRSHIFT  (_IOC_SIZESHIFT + _IOC_SIZEBITS)
#define _IOC_NONE  0U
#define _IOC_WRITE 1U
#define _IOC_READ  2U
#define _IOC(dir,type,nr,size) \
    (((dir)  << _IOC_DIRSHIFT) | \
     ((type) << _IOC_TYPESHIFT) | \
     ((nr)   << _IOC_NRSHIFT) | \
     ((size) << _IOC_SIZESHIFT))
#define _IOW(type,nr,size) _IOC(_IOC_WRITE,(type),(nr),sizeof(size))
#define _IOR(type,nr,size) _IOC(_IOC_READ,(type),(nr),sizeof(size))
#define _IOWR(type,nr,size) _IOC(_IOC_READ|_IOC_WRITE,(type),(nr),sizeof(size))
#define _IO(type,nr) _IOC(_IOC_NONE,(type),(nr),0)
#endif

#define DEV_PATH "/dev/agn_spi_test"

/* ======================== 辅助函数 ======================== */

static int hex_char_to_val(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

static int parse_hex_bytes(int argc, char **argv, int start, uint8_t *buf, int max)
{
	int i, len = 0;
	for (i = start; i < argc && len < max; i++) {
		char *s = argv[i];
		uint8_t val = 0;
		if (strlen(s) == 2) {
			int h = hex_char_to_val(s[0]);
			int l = hex_char_to_val(s[1]);
			if (h < 0 || l < 0) {
				fprintf(stderr, "invalid hex: %s\n", s);
				return -1;
			}
			val = (h << 4) | l;
		} else if (strlen(s) == 1) {
			int v = hex_char_to_val(s[0]);
			if (v < 0) {
				fprintf(stderr, "invalid hex: %s\n", s);
				return -1;
			}
			val = v;
		} else {
			fprintf(stderr, "invalid hex (too long): %s\n", s);
			return -1;
		}
		buf[len++] = val;
	}
	return len;
}

static void print_result(struct agn_spi_test_result *r)
{
	const char *state_str[] = {"idle", "running", "done", "error"};
	int i;

	printf("=== SPI Test Result ===\n");
	printf("State     : %s\n", r->state < 4 ? state_str[r->state] : "unknown");
	printf("Total     : %u\n", r->total);
	printf("Success   : %u\n", r->success);
	printf("Fail      : %u\n", r->fail);
	printf("Last Error: %d\n", (int)r->last_err);
	printf("\n");
	printf("Total Time: %llu us (%.2f ms)\n",
		(unsigned long long)r->total_time_us,
		r->total_time_us / 1000.0);
	printf("Min Time  : %llu us\n", (unsigned long long)r->min_time_us);
	printf("Max Time  : %llu us\n", (unsigned long long)r->max_time_us);
	printf("Avg Time  : %llu us\n", (unsigned long long)r->avg_time_us);

	if (r->success > 0) {
		double throughput = (double)r->success / (r->total_time_us / 1000000.0);
		printf("Throughput: %.1f transfers/sec\n", throughput);
	}

	if (r->last_rx_len > 0) {
		printf("\nLast RX (%u bytes): ", r->last_rx_len);
		for (i = 0; i < (int)r->last_rx_len; i++)
			printf("%02X ", r->last_rx[i]);
		printf("\n");
	}
}

/* ======================== 命令实现 ======================== */

static int cmd_sync(int fd, int argc, char **argv)
{
	struct agn_spi_test_result res;
	struct agn_spi_test_param p;

	memset(&p, 0, sizeof(p));
	p.mode = 0;        /* 全双工 */
	p.count = argc > 2 ? atoi(argv[2]) : 100;
	p.tx_len = argc > 3 ? atoi(argv[3]) : 4;
	p.delay_us = argc > 4 ? atoi(argv[4]) : 0;
	p.tx_pattern = 0;
	p.speed_hz = 0;
	p.spi_mode = 0xFF;

	if (p.tx_len < 1 || p.tx_len > 32) {
		fprintf(stderr, "tx_len must be 1~32\n");
		return -1;
	}

	printf("Starting sync test: count=%u len=%u delay=%uus ...\n",
		p.count, p.tx_len, p.delay_us);

	memcpy(&res, &p, sizeof(p));  /* ioctl会返回结果在同一块内存 */
	if (ioctl(fd, AGN_SPI_TEST_START, &p) < 0) {
		perror("ioctl START");
		return -1;
	}

	/* 重新获取结果 */
	if (ioctl(fd, AGN_SPI_TEST_STATUS, &res) < 0) {
		perror("ioctl STATUS");
		return -1;
	}

	print_result(&res);
	return 0;
}

static int cmd_async(int fd, int argc, char **argv)
{
	struct agn_spi_test_param p;

	memset(&p, 0, sizeof(p));
	p.mode = 0;
	p.count = argc > 2 ? atoi(argv[2]) : 0; /* 0=无限 */
	p.tx_len = argc > 3 ? atoi(argv[3]) : 4;
	p.delay_us = argc > 4 ? atoi(argv[4]) : 1000;
	p.tx_pattern = 0;
	p.speed_hz = 0;
	p.spi_mode = 0xFF;

	if (p.tx_len < 1 || p.tx_len > 32) {
		fprintf(stderr, "tx_len must be 1~32\n");
		return -1;
	}

	printf("Starting async test: count=%s len=%u delay=%uus\n",
		p.count == 0 ? "infinite" : (char[16]){0},
		p.tx_len, p.delay_us);

	/* 重新打印 */
	if (p.count == 0)
		printf("Starting async test: count=infinite len=%u delay=%uus\n",
			p.tx_len, p.delay_us);
	else
		printf("Starting async test: count=%u len=%u delay=%uus\n",
			p.count, p.tx_len, p.delay_us);

	if (ioctl(fd, AGN_SPI_TEST_START_ASYNC, &p) < 0) {
		perror("ioctl START_ASYNC");
		return -1;
	}

	printf("Async test started in background. Use 'status' to check, 'stop' to halt.\n");
	return 0;
}

static int cmd_stop(int fd)
{
	if (ioctl(fd, AGN_SPI_TEST_STOP) < 0) {
		perror("ioctl STOP");
		return -1;
	}
	printf("Test stopped.\n");
	return 0;
}

static int cmd_status(int fd)
{
	struct agn_spi_test_result res;

	if (ioctl(fd, AGN_SPI_TEST_STATUS, &res) < 0) {
		perror("ioctl STATUS");
		return -1;
	}

	print_result(&res);
	return 0;
}

static int cmd_single(int fd, int argc, char **argv)
{
	struct agn_spi_test_single s;
	int len, i;

	if (argc < 3) {
		fprintf(stderr, "Usage: %s single <hex_bytes...>\n", argv[0]);
		fprintf(stderr, "  e.g.: %s single AA BB CC DD\n", argv[0]);
		return -1;
	}

	memset(&s, 0, sizeof(s));
	len = parse_hex_bytes(argc, argv, 2, s.tx_data, 32);
	if (len <= 0) {
		fprintf(stderr, "parse hex failed\n");
		return -1;
	}

	s.tx_len = len;
	s.rx_len = 0; /* 全双工, rx_len同tx_len */

	printf("Sending %d bytes: ", len);
	for (i = 0; i < len; i++)
		printf("%02X ", s.tx_data[i]);
	printf("\n");

	if (ioctl(fd, AGN_SPI_TEST_SINGLE, &s) < 0) {
		perror("ioctl SINGLE");
		return -1;
	}

	printf("Result: %d\n", s.ret);
	if (s.ret == 0) {
		printf("RX (%u bytes): ", s.rx_len);
		for (i = 0; i < (int)s.rx_len; i++)
			printf("%02X ", s.rx_data[i]);
		printf("\n");
	}

	return s.ret;
}

static int cmd_wtr(int fd, int argc, char **argv)
{
	struct agn_spi_test_single s;
	int tx_len, rx_len, i;

	if (argc < 4) {
		fprintf(stderr, "Usage: %s wtr <tx_hex...> <rx_len>\n", argv[0]);
		fprintf(stderr, "  e.g.: %s wtr 80 00 8\n", argv[0]);
		return -1;
	}

	rx_len = atoi(argv[argc - 1]);
	if (rx_len < 1 || rx_len > 32) {
		fprintf(stderr, "rx_len must be 1~32\n");
		return -1;
	}

	memset(&s, 0, sizeof(s));
	tx_len = parse_hex_bytes(argc - 1, argv, 2, s.tx_data, 32);
	if (tx_len <= 0) {
		fprintf(stderr, "parse hex failed\n");
		return -1;
	}

	s.tx_len = tx_len;
	s.rx_len = rx_len;

	printf("WTR: TX %d bytes [", tx_len);
	for (i = 0; i < tx_len; i++)
		printf("%02X ", s.tx_data[i]);
	printf("], RX %d bytes\n", rx_len);

	if (ioctl(fd, AGN_SPI_TEST_SINGLE, &s) < 0) {
		perror("ioctl SINGLE");
		return -1;
	}

	printf("Result: %d\n", s.ret);
	if (s.ret == 0) {
		printf("RX (%u bytes): ", s.rx_len);
		for (i = 0; i < (int)s.rx_len; i++)
			printf("%02X ", s.rx_data[i]);
		printf("\n");
	}

	return s.ret;
}

/* ======================== 主函数 ======================== */

static void usage(const char *prog)
{
	printf("AGN SPI Test Tool\n\n");
	printf("Usage: %s <command> [args...]\n\n", prog);
	printf("Commands:\n");
	printf("  sync [count] [len] [delay_us]  - 同步循环测试 (默认: 100 4 0)\n");
	printf("  async [count] [len] [delay_us] - 异步测试 (count=0为无限)\n");
	printf("  stop                           - 停止异步测试\n");
	printf("  status                         - 查看测试结果\n");
	printf("  single <hex...>                - 单次全双工收发\n");
	printf("  wtr <tx_hex...> <rx_len>       - 先发后收\n");
	printf("\nExamples:\n");
	printf("  %s sync 1000 4 0         # 1000次全双工,4字节,无间隔\n", prog);
	printf("  %s async 0 4 1000        # 无限次后台跑,1ms间隔\n", prog);
	printf("  %s stop                  # 停止\n", prog);
	printf("  %s status                # 看结果\n", prog);
	printf("  %s single AA BB CC DD    # 发4字节看回读\n", prog);
	printf("  %s wtr 80 00 8           # 发2字节命令,读8字节响应\n", prog);
}

int main(int argc, char **argv)
{
	int fd, ret = 0;

	if (argc < 2) {
		usage(argv[0]);
		return 1;
	}

	fd = open(DEV_PATH, O_RDWR);
	if (fd < 0) {
		perror("open " DEV_PATH);
		fprintf(stderr, "Make sure agn_spi_test.ko is loaded\n");
		return 1;
	}

	if (strcmp(argv[1], "sync") == 0) {
		ret = cmd_sync(fd, argc, argv);
	} else if (strcmp(argv[1], "async") == 0) {
		ret = cmd_async(fd, argc, argv);
	} else if (strcmp(argv[1], "stop") == 0) {
		ret = cmd_stop(fd);
	} else if (strcmp(argv[1], "status") == 0) {
		ret = cmd_status(fd);
	} else if (strcmp(argv[1], "single") == 0) {
		ret = cmd_single(fd, argc, argv);
	} else if (strcmp(argv[1], "wtr") == 0) {
		ret = cmd_wtr(fd, argc, argv);
	} else if (strcmp(argv[1], "help") == 0 || strcmp(argv[1], "-h") == 0) {
		usage(argv[0]);
	} else {
		fprintf(stderr, "Unknown command: %s\n", argv[1]);
		usage(argv[0]);
		ret = 1;
	}

	close(fd);
	return ret;
}
