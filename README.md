# ManyController

ROS 2 无人机控制工作空间，包含 PX4 消息、公共工具、控制节点和仿真组件。

新机器从安装系统依赖、初始化第三方源码到编译工作空间的完整操作手册见
[README_INSTALL.md](README_INSTALL.md)。

本工程不再提交针对某台 Linux 电脑预编译的第三方 `.so`。固定版本的源码以 Git
子模块保存在 `3rdpart/src`，在每台目标机上原生编译并默认安装到 `/usr/local`。

新机器的基本流程：

```bash
git submodule update --init
git -C 3rdpart/src/acados submodule update --init \
  external/blasfeo external/hpipm
JOBS=2 ./3rdpart/build_all.sh

source /opt/ros/humble/setup.bash
colcon build --packages-up-to px4ctrl --cmake-clean-cache
```

系统包清单、固定版本、各依赖的用途、CMake 查找方式和另一台电脑的同步步骤见
[3rdpart/README.md](3rdpart/README.md)。
