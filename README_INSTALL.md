# ManyController：从零安装依赖并编译

本文用于在一台新的 Ubuntu 电脑或无人机伴随计算机上，从空白环境开始取得源码、
安装依赖、编译第三方数值库，并最终编译 ManyController ROS 2 工作空间。

推荐环境：

- Ubuntu 22.04；
- ROS 2 Humble；
- GCC/G++ 11 或 Ubuntu 22.04 默认编译器；
- x86_64 或 aarch64；
- 至少 10 GB 可用磁盘空间；
- 内存较小的无人机电脑使用 `JOBS=1` 或 `JOBS=2`。

不要把其他电脑上编译好的 `.so` 复制过来。不同机器的 CPU 架构、glibc、
libstdc++ 和编译选项可能不同，容易出现 `GLIBC_2.34`、未定义符号或加载失败。
本仓库保存固定版本的第三方源码，每台目标机都应原生编译。

## 1. 安装 ROS 2 Humble

ROS 2 Humble 的 Debian 包面向 Ubuntu 22.04。先按照 ROS 官方文档配置软件源并
安装 Humble：

- <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

推荐安装桌面版；无人机无桌面环境时可以安装基础版：

```bash
# 开发电脑，包含 RViz 等 GUI 工具
sudo apt install ros-humble-desktop ros-dev-tools

# 或：无人机伴随计算机的精简安装
sudo apt install ros-humble-ros-base ros-dev-tools
```

确认 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
ros2 --help >/dev/null
```

`ROS_DISTRO` 应输出 `humble`。可将环境初始化加入 shell：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

## 2. 安装系统工具和基础数值库

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake pkg-config \
  autoconf automake libtool gfortran \
  python3-pip python3-venv python3-rosdep \
  python3-colcon-common-extensions \
  libblas-dev liblapack-dev \
  libmumps-seq-dev libscotch-dev libmetis-dev \
  libsuitesparse-dev
```

这些系统包的作用如下：

| 系统包 | 用途 |
| --- | --- |
| `build-essential`、CMake、Autotools、Fortran | 编译 C/C++/Fortran 第三方源码 |
| BLAS、LAPACK | 基础稠密线性代数，Ipopt 使用 |
| 顺序版 MUMPS、Scotch、METIS | Ipopt 的稀疏线性系统求解链 |
| SuiteSparse | `px4ctrlrate_node` 使用 UMFPACK |
| colcon、rosdep | 构建和解析 ROS 2 工作空间 |

首次使用 rosdep 时执行一次：

```bash
sudo rosdep init
rosdep update
```

如果 `rosdep init` 提示配置文件已经存在，说明此前初始化过，直接执行
`rosdep update` 即可。

## 3. 克隆工程

新机器首次克隆：

```bash
cd ~
git clone https://github.com/zetabcd/ManyController.git manycontroller
cd manycontroller
```

不要使用普通 ZIP 下载，因为 ZIP 不包含 Git 子模块的提交信息。

## 4. 初始化固定版本的第三方源码

先取得七个顶层源码子模块：

```bash
git submodule sync
git submodule update --init
```

再只初始化 acados 当前构建需要的两个内部子模块：

```bash
git -C 3rdpart/src/acados submodule update --init \
  external/blasfeo external/hpipm
```

检查固定版本：

```bash
git submodule status 3rdpart/src/eigen \
  3rdpart/src/qdldl \
  3rdpart/src/osqp \
  3rdpart/src/nlopt \
  3rdpart/src/ipopt \
  3rdpart/src/casadi \
  3rdpart/src/acados
```

预期源码关系：

```text
ManyController
├── Eigen 3.4.0                 （头文件矩阵库）
├── QDLDL 0.1.8
│   └── 被 OSQP 1.0.0 使用      （QP 求解）
├── NLopt 2.7.1                 （非线性优化）
├── Ipopt 3.14.11
│   └── 系统 BLAS/LAPACK/MUMPS  （非线性规划）
├── CasADi 3.7.2
│   └── 使用已安装的 Ipopt 插件
└── acados 固定提交             （最优控制）
    ├── BLASFEO
    └── HPIPM
```

源码保存在 `3rdpart/src`，本机中间产物写入被 Git 忽略的 `3rdpart/build`。

## 5. 推荐路线：安装第三方库到 `/usr/local`

这里所说的“安装到根目录”是指 Linux 常规的本机安装前缀 `/usr/local`，不是把
文件直接安装到 `/`。`/usr/local/include`、`/usr/local/lib` 和其中的 CMake 包
配置是编译工具的标准搜索位置。

执行统一脚本：

```bash
cd ~/manycontroller
JOBS=2 ./3rdpart/build_all.sh
```

脚本按以下顺序执行：

1. Eigen；
2. OSQP，并强制使用本仓库的 QDLDL；
3. NLopt；
4. Ipopt，并连接系统 BLAS、LAPACK 和顺序版 MUMPS；
5. CasADi，并使用刚安装的 Ipopt；
6. acados、HPIPM 和 BLASFEO。

安装阶段需要写入 `/usr/local`，脚本会调用 `sudo`。源码不会被删除；中途失败后
修复原因并重新运行同一脚本即可。无人机内存不足时：

```bash
JOBS=1 ./3rdpart/build_all.sh
```

默认 `BLASFEO_TARGET=GENERIC`，适合跨 x86_64/aarch64 构建。确认目标 CPU 后才应
选择更激进的 BLASFEO 优化目标。

### 5.1 OSQP 版本为什么必须显式传入

OSQP `v1.0.0` 的上游 CMake 默认设置 `OSQP_VERSION=0.0.0`，不会自动从 Git 标签
获取版本。本工程的脚本显式使用：

```bash
-DOSQP_VERSION=1.0.0
```

否则安装出来的源码虽然是 1.0.0，`osqp-config-version.cmake` 却会报告 0.0.0，
进而被 `find_package(osqp 1.0 CONFIG REQUIRED)` 拒绝。

### 5.2 验证第三方库安装

```bash
grep 'set(PACKAGE_VERSION' \
  /usr/local/lib/cmake/osqp/osqp-config-version.cmake | head -1

find /usr/local/include -name IpIpoptApplication.hpp -print

ldconfig -p | grep -E \
  'lib(casadi|acados|hpipm|blasfeo|osqp|ipopt|nlopt)'
```

关键结果应包含：

```text
PACKAGE_VERSION "1.0.0"
/usr/local/include/coin-or/IpIpoptApplication.hpp
```

还可以检查 CMake 包配置：

```bash
find /usr/local/lib /usr/local/lib64 \
  \( -name '*Config.cmake' -o -name '*-config.cmake' \) \
  | grep -E 'Eigen|casadi|osqp|acados|nlopt'
```

## 6. 安装 ROS 2 包依赖

回到工作空间根目录：

```bash
cd ~/manycontroller
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys stservo_msgs
```

`px4_msgs`、`fms_utils`、`uav_utils`、`px4debug_msgs`、`joy_msgs` 和
`ratectrl_msgs` 都在当前工作空间内，由 colcon 按依赖顺序构建。

`quadsim_mujoco/package.xml` 目前保留了历史依赖 `stservo_msgs`，但当前模拟器源码
没有导入它，因此 rosdep 阶段跳过该键。它不影响 `px4ctrl` 控制器编译。

## 7. 编译控制器工程

首次编译或第三方路径发生变化时必须使用干净的 CMake 缓存：

```bash
cd ~/manycontroller
source /opt/ros/humble/setup.bash

colcon build \
  --packages-up-to px4ctrl \
  --cmake-clean-cache
```

低内存机器可限制并行任务：

```bash
MAKEFLAGS=-j2 colcon build \
  --executor sequential \
  --packages-up-to px4ctrl \
  --cmake-clean-cache
```

CMake 配置阶段应打印类似内容：

```text
px4ctrl Eigen: Eigen3::Eigen
px4ctrl OSQP: osqp::osqp
px4ctrl Ipopt include: /usr/local/include/coin-or
px4ctrl Ipopt library: /usr/local/lib/libipopt.so
px4ctrl NLopt: NLopt::nlopt
px4ctrl acados: acados
px4ctrl UMFPACK: /usr/lib/.../libumfpack.so
```

编译成功后加载工作空间：

```bash
source ~/manycontroller/install/setup.bash
ros2 pkg prefix px4ctrl
ros2 pkg executables px4ctrl
```

可执行程序包括：

- `px4ctrl_node`：外环、状态机和多种 NMPC 后端；
- `px4ctrlrate_node`：高频角速度环和电机分配；
- `gpttraj_visualizer_node`：轨迹可视化。

## 8. 可选：编译和运行 MuJoCo 模拟器

控制器编译不依赖 MuJoCo。需要仿真时额外安装：

```bash
sudo apt install -y ros-humble-cv-bridge python3-numpy python3-scipy
python3 -m pip install --user 'mujoco==3.2.3' glfw
```

然后编译模拟器及其工作空间依赖：

```bash
source /opt/ros/humble/setup.bash
cd ~/manycontroller
colcon build --packages-up-to quadsim_mujoco
source install/setup.bash
```

MuJoCo viewer 需要图形环境和 OpenGL；无显示器的无人机电脑通常只编译、运行控制
节点，不启动 viewer。

## 9. 可选路线：安装到独立目录

如果不希望写入 `/usr/local`，例如安装到 `/opt/manycontroller-deps`：

```bash
cd ~/manycontroller
PREFIX=/opt/manycontroller-deps JOBS=2 ./3rdpart/build_all.sh
```

安装前缀不是系统默认搜索路径，因此编译工程时要告诉 CMake：

```bash
source /opt/ros/humble/setup.bash
cd ~/manycontroller

colcon build \
  --packages-up-to px4ctrl \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_PREFIX_PATH=/opt/manycontroller-deps
```

变量的职责不能混淆：

| 变量 | 设置阶段 | 作用 |
| --- | --- | --- |
| `CMAKE_INSTALL_PREFIX` | 编译第三方库 | 决定库安装到哪里 |
| `PREFIX` | 调用本工程脚本 | 统一传给各库的安装前缀 |
| `CMAKE_PREFIX_PATH` | 编译 ManyController | 告诉 `find_package/find_path/find_library` 去哪里找 |
| `Foo_DIR` | 编译 ManyController | 精确指定某个 `FooConfig.cmake` 所在目录 |
| `LD_LIBRARY_PATH` | 运行程序 | 告诉动态加载器去哪里找 `.so` |

自定义前缀下运行程序前还要处理动态库路径：

```bash
export LD_LIBRARY_PATH=/opt/manycontroller-deps/lib:\
/opt/manycontroller-deps/lib64:${LD_LIBRARY_PATH:-}
source ~/manycontroller/install/setup.bash
```

若想永久配置，可由管理员创建 `/etc/ld.so.conf.d/manycontroller.conf`，内容为实际
存在的库目录，然后执行 `sudo ldconfig`。不要把开发电脑的绝对路径写进工程
CMakeLists，也不要把 `.so` 再复制回 `3rdpart`。

当只需指定一个包时，可使用：

```bash
colcon build --packages-up-to px4ctrl --cmake-clean-cache \
  --cmake-args \
    -Dosqp_DIR=/opt/manycontroller-deps/lib/cmake/osqp \
    -Dcasadi_DIR=/opt/manycontroller-deps/lib/cmake/casadi
```

不同库可能把配置安装到 `lib/cmake`、`lib64/cmake` 或带包名的子目录；先用
`find /opt/manycontroller-deps -name '*Config.cmake' -o -name '*-config.cmake'`
确认实际位置。

## 10. 在另一台电脑同步更新

```bash
cd ~/manycontroller
git pull --ff-only
git submodule sync --recursive
git submodule update --init
git -C 3rdpart/src/acados submodule update --init \
  external/blasfeo external/hpipm

JOBS=2 ./3rdpart/build_all.sh

source /opt/ros/humble/setup.bash
colcon build --packages-up-to px4ctrl --cmake-clean-cache
```

主仓库只记录每个子模块的精确提交。不要在目标机上单独把 OSQP、QDLDL 或
acados 子模块切换到别的分支，否则机器之间将不再使用同一套源码。

## 11. 常见错误

### 11.1 OSQP 配置显示 0.0.0

```text
Could not find a configuration file for package "osqp" ...
version: 0.0.0
```

这是旧版构建命令没有传 `OSQP_VERSION`。更新仓库后重新运行：

```bash
JOBS=2 ./3rdpart/build_all.sh
```

也可以只重建 OSQP，完整命令见 `3rdpart/README.md`。

### 11.2 找不到 `IpIpoptApplication.hpp`

先检查真实文件：

```bash
find /usr/local/include /usr/include \
  -name IpIpoptApplication.hpp -print
```

若文件在 `/usr/local/include/coin-or`，但编译命令仍出现旧的
`3rdpart/optimization/include/coin`，就是 CMake 缓存污染：

```bash
colcon build --packages-up-to px4ctrl --cmake-clean-cache
```

若完全找不到该文件，则 Ipopt 尚未安装完成，重新运行第三方构建脚本。

### 11.3 找不到 `Eigen/Eigen`

检查：

```bash
test -f /usr/local/include/eigen3/Eigen/Eigen && echo found
test -f /usr/include/eigen3/Eigen/Eigen && echo found
```

随后清理 CMake 缓存。不要把 include 改成某台机器的绝对路径。

### 11.4 QDLDL 出现 rebase 或 `origin/v0.1.8` 错误

不要使用 OSQP 构建目录里 FetchContent 临时克隆的 QDLDL。当前脚本通过
`FETCHCONTENT_SOURCE_DIR_QDLDL` 使用 `3rdpart/src/qdldl` 的固定提交。重新同步
子模块并用本工程脚本配置 OSQP。

### 11.5 出现 `pthread_join@GLIBC_2.34` 等错误

说明正在链接另一台电脑编译的二进制库。移除旧构建缓存，确认编译命令不再引用
旧 `3rdpart/optimization/lib`，然后在当前机器重新编译所有第三方源码。

### 11.6 编译通过但运行时找不到 `.so`

```bash
ldd install/px4ctrl/lib/px4ctrl/px4ctrl_node | grep 'not found'
```

使用 `/usr/local` 时执行 `sudo ldconfig`；使用自定义前缀时设置
`LD_LIBRARY_PATH` 或配置 `/etc/ld.so.conf.d`。

### 11.7 `qpOASES.hpp` 找不到

当前 px4ctrl 不直接使用 qpOASES API，不需要额外安装其 C++ 开发包。若本地代码
仍包含该头文件，说明使用了旧分支或保留了历史修改。CasADi 自带的 qpOASES 插件
也不等于系统可直接 include 的 `qpOASES.hpp` 开发包。

## 12. 完全重建与最终检查

第三方源码不变时通常不必重复编译。依赖版本、安装前缀或机器发生变化时，先重新
运行 `build_all.sh`，再使用 `--cmake-clean-cache`。

最终检查：

```bash
cd ~/manycontroller
source /opt/ros/humble/setup.bash
source install/setup.bash

git submodule status
ros2 pkg prefix px4ctrl
ros2 pkg executables px4ctrl
ldd install/px4ctrl/lib/px4ctrl/px4ctrl_node | grep 'not found' || true
```

最后一条没有输出，表示直接依赖的动态库均已找到。

更详细的库关系、CMake 查找机制和特殊构建方式见：

- [`3rdpart/README.md`](3rdpart/README.md)；
- [`latex/third_party_dependency_management.tex`](latex/third_party_dependency_management.tex)。
