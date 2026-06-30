╭─ ⚕ Hermes ───────────────────────────────────────────────────────────────────────────────────────────────╮
    测试驱动和用户空间测试程序都已写好，文件清单：

      E:\YM\work\H1504T_SR800\SPI寄存器\agn_spi.c        -- SPI0轻量驱动
      E:\YM\work\H1504T_SR800\SPI寄存器\agn_spi.h        -- 驱动头文件
      E:\YM\work\H1504T_SR800\SPI寄存器\agn_spi_test.c   -- 测试驱动(内核模块)
      E:\YM\work\H1504T_SR800\SPI寄存器\agn_spi_test_app.c -- 用户空间测试工具

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    一、测试驱动 agn_spi_test.c 功能
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    创建设备节点 /dev/agn_spi_test，通过 ioctl 控制：

      ioctl命令                    功能
      ──────────────────────────   ──────────────────────────
      AGN_SPI_TEST_SET_CFG         设置测试参数(模式/次数/长度/延迟/速率)
      AGN_SPI_TEST_START           同步测试(阻塞到完成，返回结果)
      AGN_SPI_TEST_START_ASYNC     异步测试(内核线程，非阻塞)
      AGN_SPI_TEST_STOP            停止异步测试
      AGN_SPI_TEST_STATUS          查询测试状态和结果
      AGN_SPI_TEST_SINGLE          单次收发(可指定TX数据)

    测试模式:
      mode=0  全双工 (agn_spi_transfer)
      mode=1  仅发送 (agn_spi_write)
      mode=2  仅接收 (agn_spi_read)
      mode=3  先发后收 (agn_spi_write_then_read)

    统计信息: 总次数、成功/失败次数、总耗时、最小/最大/平均单次耗时、最后一次RX数据

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    二、编译方法
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    内核模块(在你的内核编译环境中):

    方法1: 直接在Makefile中加入
      obj-m += agn_spi.o
      obj-m += agn_spi_test.o

    方法2: 或写成独立Makefile
      KDIR := /path/to/your/kernel
      PWD  := $(shell pwd)

      obj-m += agn_spi.o
      obj-m += agn_spi_test.o

      all:
          make -C $(KDIR) M=$(PWD) modules

      clean:
          make -C $(KDIR) M=$(PWD) clean

    用户空间程序(在Android NDK或交叉编译环境中):

    用Android NDK交叉编译
      /path/to/ndk/toolchains/arm-linux-androideabi-4.9/prebuilt/linux-x86_64/bin/arm-linux-androideabi-gcc \
          --sysroot=/path/to/ndk/platforms/android-21/arch-arm \
          -o agn_spi_test_app agn_spi_test_app.c

    或者push到设备上用设备上的gcc编译
      adb push agn_spi_test_app.c /data/local/tmp/
      adb shell "cd /data/local/tmp && gcc -o agn_spi_test_app agn_spi_test_app.c"

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    三、使用流程
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    1. 加载驱动
      insmod agn_spi.ko
      insmod agn_spi_test.ko

    2. 确认设备节点
      ls -l /dev/agn_spi_test

    3. 运行测试

    同步测试: 1000次全双工,每次4字节,无间隔
      ./agn_spi_test_app sync 1000 4 0

    同步测试: 500次,每次8字节,每次间隔100us
      ./agn_spi_test_app sync 500 8 100

    异步测试: 无限次后台跑,4字节,1ms间隔
      ./agn_spi_test_app async 0 4 1000

    查看异步测试进度
      ./agn_spi_test_app status

    停止异步测试
      ./agn_spi_test_app stop

    单次收发: 发送 0xAA 0xBB 0xCC 0xDD, 看回读数据
      ./agn_spi_test_app single AA BB CC DD

    发命令读响应: 发 0x80 0x00, 读8字节
      ./agn_spi_test_app wtr 80 00 8

    4. 卸载
      rmmod agn_spi_test
      rmmod agn_spi

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    四、预期输出示例
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

      $ ./agn_spi_test_app sync 1000 4 0
      Starting sync test: count=1000 len=4 delay=0us ...
      === SPI Test Result ===
      State     : done
      Total     : 1000
      Success   : 1000
      Fail      : 0
      Last Error: 0

      Total Time: 15234 us (15.23 ms)
      Min Time  : 12 us
      Max Time  : 45 us
      Avg Time  : 15 us
      Throughput: 65674.3 transfers/sec

      Last RX (4 bytes): 04 05 06 07

    这个吞吐量数据可以帮你评估 agn_spi 相对于标准SPI框架的性能提升。

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    五、dmesg 日志
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

      dmesg | grep agn_spi

      [agn_spi] probe OK: base=... irq=118 speed=1000000Hz mode=0
      [agn_spi_test] loaded, device: /dev/agn_spi_test
      [agn_spi_test] sync test start: mode=0 count=1000 tx_len=4 delay=0us
      [agn_spi_test] sync test done: total=1000 ok=1000 fail=0 time=15234us ...

    有问题或需要调整随时说。
╰──────────────────────────────────────────────────────────────────────────────────────────────────────────╯