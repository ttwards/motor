# Zephyr RTOS电机驱动
## 该项目是由南方科技大学机器人社支持的，希望以设备树形式描述机器人
### 最终的效果将如example.dts所示
目前我们仅完成了RM M3508电机的驱动
## TODO List
- 完成RM M3508,M2006电机驱动 
- 完成小米，达喵电机驱动
- 完成运动解算设备
- ......
#### 目前想法还并不是很成熟，球球大佬建议
## 如何在我的开发板上运行？（以Robomaster Developement Board C为例）
### Initialization

The first step is to initialize the workspace folder (``my-workspace``) where
the ``example-application`` and all Zephyr modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the example-application (main branch)
west init -m https://github.com/ttwards/motor --mr master my-workspace
# update Zephyr modules
cd my-workspace
west update
```
### Building and running

To build the application, run the following command:

```shell
cd motor
west build -b $BOARD motor
```

where `$BOARD` is the target board. Here you can use `robomaster_board_c`

A sample debug configuration is also provided. To apply it, run the following
command:

```shell
west build -b $BOARD blinky -- -DOVERLAY_CONFIG=debug.conf
```

Once you have built the application, run the following command to flash it:

```shell
west flash
```
If everything goes well, the LED on the board should be blinking
