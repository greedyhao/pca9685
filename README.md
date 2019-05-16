# pca9685 软件包

## 介绍

`pca9685` 软件包是 RT-Thread 针对 I2C 并行口扩展电路 PCA9685T 推出的一个软件包，兼容 PCA9685A。使用这个软件包，可以在 RT-Thread 上非常方便的使用该器件，并且支持一个 I2C 总线上挂载多个 PCA9685T。

本文主要介绍该软件包的使用方式、API，以及 `MSH` 测试命令。

### 目录结构

```
pca9685
│   README.md                       // 软件包说明
│   pca9685.c                       // 源文件
│   pca9685.h                       // 头文件
│   pca9685_sample.c                // 软件包使用示例代码
│   SConscript                      // RT-Thread 默认的构建脚本
│   LICENSE                         // 许可证文件
```

### 许可证

pca9685 遵循 Apache-2.0 许可，详见 `LICENSE` 文件。

### 依赖

- RT_Thread 3.0+
- i2c 设备驱动

## 获取方式

使用 `pca9685 package` 需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages
    peripheral libraries and drivers  --->
        pca9685: Remote 8-bit I/O expander for I2C-bus  --->
```

进入 pca9685 软件包的配置菜单按自己的需求进行具体的配置

```
    --- pca9685: Remote 8-bit I/O expander for I2C-bus                           
        [*]   Enable pca9685 sample
           Version (latest)  --->
```

**Enable pca9685 sample** ：开启 pca9685  使用示例

配置完成后让 RT-Thread 的包管理器自动更新，或者使用 pkgs --update 命令更新包到 BSP 中。

## 使用方法

pca9685 软件包的使用流程一般如下：

1. 初始化 pca9685 设备 `pca9685_init`
2. 进行 PWM 的操作
   - 使用 API `pca9685_port_read/pca9685_port_write` 同时操作 16 路 PWM
   - 使用 API `pca9685_pin_read/pca9685_pin_write` 单独操作其中一 路 PWM

详细的使用方法可以参考[pca9685 示例程序](pca9685_sample.c) 。

## MSH 测试命令

如果开启了 pca9685 软件包的示例程序，就会导出 `pca9685_sample` 命令到控制台。调用之后默认会在 `i2c1`总线上探测地址为 `0x80` 的 PCA9685 设备，并会操作扩展端口的第 0 口进行测试。运行结果如下：

```
msh >pca9685_sample
[D/pca9685] pca9685 init done
The value of pca9685.P0 is 0
The value of pca9685.P0 is 1
msh >
```

## 注意事项

暂无。

## 联系方式

- 维护：[guozhanxin](https://github.com/Guozhanxin)
- 主页：<https://github.com/RT-Thread-packages/pca9685 >