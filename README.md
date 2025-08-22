# Zephyr RTOS 下位机框架
## 该项目希望以设备树形式描述机器人
### 最终的效果将如example.dts所示
The structure is as shown below
![structure](https://github.com/ttwards/motor/structure.png "Structure")
目前我们仅完成了RM M3508电机的驱动
## TODO List
- 完成RM M3508,M2006电机驱动✅
- 完成达喵电机驱动✅
- 完成运动解算设备
- ......
#### 目前想法还并不是很成熟，球球大佬建议
## 如何在我的开发板上运行？（以Robomaster Developement Board C为例）
### Initialization

First zephyr SDKs are needed.
```shell
sudo apt update
sudo apt upgrade
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
```
### You should have a virtual environment if Python tells you to do so

```shell
pip install west
```

Then initialize the workspace folder (``my-workspace``) where
the ``example-application`` and all Zephyr modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the example-application (main branch)
west init -m https://github.com/ttwards/motor --mr master my-workspace
# update Zephyr modules
cd my-workspace
west update
```

Then export a Zephyr CMake package. This allows CMake to automatically load boilerplate code required for building Zephyr applications.
```shell
west zephyr-export
pip install -r ./zephyr/scripts/requirements.txt
west sdk install
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
默认为stlink, 如果使用cmsis-dap请加上`--runner openocd`, Jlink则为`--runner jlink`
If everything goes well, the LED on the board should be blinking

Then you can have your motors running!

```shell
west build -p auto -b $BOARD motor 
```
You should have two motors set at 0x201 and 0x202, connected to CAN1.
If everything goes well, you should see them at rpm 1222.
The RPM is graphed via UART1

另外，VS Code在下栏会显示一系列tasks，你可以在`.vscode/tasks.json`中找到它们的设置

详细的文档请参考`Documents`文件夹

### App
我们的App都放在app目录下
`git submodule update --init`
从子仓库中抓取所有的数据找到父级仓库对应的那次子仓库的提交id并且检出到父项目的目录中。
```
好无聊逗逗梅总吧
    嘬嘬嘬𐃆 ˒˒ ͏                               
͏
͏                             ╱|、
                            (˚ˎ 。7 
                            |、˜ 〵 
                            じしˍ,_)ノ
```

