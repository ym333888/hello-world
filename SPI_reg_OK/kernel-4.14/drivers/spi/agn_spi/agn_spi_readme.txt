╭─ ⚕ Hermes ───────────────────────────────────────────────────────────────────────────────────────────────╮
    两个文件已写入完成：

      E:\YM\work\H1504T_SR800\SPI寄存器\agn_spi.h
      E:\YM\work\H1504T_SR800\SPI寄存器\agn_spi.c

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    一、设计思路
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    与标准spi-mt65xx驱动的对比:

                         标准驱动          agn_spi驱动
      ─────────────────  ──────────────    ──────────────
      传输方式           FIFO + DMA         仅FIFO (<=32字节)
      每次传输IO次数     15~20+次           6~8次
      时钟管理           每次CS切换开关      probe开启，保持不关
      寄存器配置         每次传输都重配      probe/resume预配一次
      CS控制             GPIO + PAUSE_EN    仅PAUSE_EN
      SPI框架开销        经过完整消息队列    直接操作寄存器
      并发保护           框架自带            mutex

    每次 agn_spi_transfer() 的热路径IO操作:

      1. writel(CMD_REG, cmd_base)          -- CS active
      2. iowrite32_rep(TX_DATA_REG, buf)   -- 写TX数据
      3. writel(CFG1_REG, cfg1+pkt_len)    -- 更新packet长度
      4. readl+writel(CMD_REG, ACT)        -- 启动传输
      5. wait_for_completion()              -- 等中断(无IO)
      6. ioread32_rep(RX_DATA_REG, buf)    -- 读RX数据
      7. writel(CMD_REG, cmd_base+PAUSE_EN) -- CS deassert

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    二、提供的API接口
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

      agn_spi_transfer(tx, rx, len)           全双工, tx/rx可为NULL
      agn_spi_write(tx, len)                  仅发送
      agn_spi_read(rx, len)                   仅接收(发dummy)
      agn_spi_write_then_read(tx,tl,rx,rl)    先发后收(两段式)
      agn_spi_reconfigure(cfg)                运行时改速率/模式
      agn_spi_is_ready()                      检查驱动就绪

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    三、DTS修改要求
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    需要在你项目的DTS中做两处修改:

    1) 禁用标准SPI0驱动节点:

       spi0: spi@1100a000 {
           ...
           status = "disabled";   / 添加这行 /
       };

    2) 添加本驱动节点:

       agn_spi0: agn-spi@1100a000 {
           compatible = "agn,mt6739-spi0";
           reg = <0 0x1100a000 0 0x1000>;
           interrupts = <GIC_SPI 118 IRQ_TYPE_LEVEL_LOW>;
           clocks = <&topckgen CLK_TOP_SYSPLL3_D2>,
                    <&topckgen CLK_TOP_SPI_SEL>,
                    <&infracfg_ao CLK_INFRA_SPI0>;
           clock-names = "parent-clk", "sel-clk", "spi-clk";
           agn,spi-speed = <1000000>;   / 可选, 默认1MHz /
           agn,spi-mode = <0>;          / 可选, SPI mode 0~3 /
           status = "okay";
       };

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    四、调用示例
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    在你的外设驱动中:

      #include "agn_spi.h"

      // 检查就绪
      if (!agn_spi_is_ready())
          return -ENODEV;

      // 发送3字节命令
      u8 cmd[3] = {0x01, 0x02, 0x03};
      agn_spi_write(cmd, 3);

      // 全双工: 发4字节, 同时收4字节
      u8 tx[4] = {0xAA, 0xBB, 0xCC, 0xDD};
      u8 rx[4];
      agn_spi_transfer(tx, rx, 4);

      // 发命令后读响应
      u8 wr[2] = {0x80, 0x00};
      u8 rd[8];
      agn_spi_write_then_read(wr, 2, rd, 8);

      // 运行时改速率为2MHz
      struct agn_spi_config cfg = { .speed_hz = 2000000, .cpol = 0, .cpha = 0,
          .tx_mlsb = 1, .rx_mlsb = 1 };
      agn_spi_reconfigure(&cfg);

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    五、调试接口 (sysfs)
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

      cat /sys/devices/.../agn_spi/spi_dump    -- 打印所有寄存器
      cat /sys/devices/.../agn_spi/spi_speed   -- 读当前速率
      echo 2000000 > .../agn_spi/spi_speed     -- 改速率
      cat /sys/devices/.../agn_spi/spi_mode    -- 读SPI mode
      echo 3 > .../agn_spi/spi_mode            -- 改为mode 3

    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    六、注意事项
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    1. 传输长度限制 1~32 字节 (FIFO模式硬限制)
    2. 驱动使用mutex防并发，多线程调用会串行化
    3. suspend时关闭时钟，resume时重新预配置全部寄存器
    4. MT6739 的 must_tx=true，纯读操作也会发dummy数据(0x00)
    5. 如需开启调试日志，将 agn_spi.c 中 AGN_SPI_DBG 改为 1

    有什么需要调整的随时告诉我。
╰──────────────────────────────────────────────────────────────────────────────────────────────────────────╯