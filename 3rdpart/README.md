# 第三方依赖：源码、本机编译与安装

`3rdpart/` 现在只管理第三方源码和统一构建脚本，不再保存从某台电脑复制来的
头文件、共享库或 Debian 安装目录。所有 C/C++ 库都在目标机上编译，并默认安装
到 `/usr/local`。这样可避免 x86_64/aarch64、glibc 版本和 C++ ABI 不一致导致的
链接错误。

## 目录布局

```text
3rdpart/
├── build_all.sh       # 按固定顺序编译并安装依赖
├── build/             # 本机临时构建目录，已被 Git 忽略
└── src/               # 固定提交的 Git 子模块，仅保存源码
    ├── eigen
    ├── qdldl
    ├── osqp
    ├── nlopt
    ├── ipopt
    ├── casadi
    └── acados
```

acados 内部只需要再初始化 `external/blasfeo` 和 `external/hpipm`。工程专用的
acados 生成代码不是第三方库，位于：

```text
src/realflight_modules/px4ctrl/generated/acados/px4ctrl_nmpc
```

这里提交的是 C/H/JSON 源文件；CMake 会在目标机上把它编译成静态目标，不提交
生成的 `.o` 或 `.so`。

## 固定版本与用途

| 依赖 | 固定版本 | 用途 | 安装方式 |
| --- | --- | --- | --- |
| Eigen | 3.4.0 | 矩阵与几何运算 | 源码安装到 `/usr/local` |
| QDLDL | 0.1.8 | OSQP 的内部线性求解器 | 源码交给 OSQP 构建 |
| OSQP | 1.0.0 | `gptmpc`、`gpttraj` | 源码安装到 `/usr/local` |
| NLopt | 2.7.1 | `nlopt+eigen` NMPC 后端 | 源码安装到 `/usr/local` |
| Ipopt | 3.14.11 | `ipopt+eigen` 和 CasADi 后端 | 源码安装到 `/usr/local` |
| CasADi | 3.7.2 | `ipopt+casadi` 后端 | 源码安装到 `/usr/local` |
| acados | `4c23274e4` | acados NMPC 后端 | 源码安装到 `/usr/local` |
| BLASFEO | acados 锁定提交 | acados 线性代数 | 随 acados 安装 |
| HPIPM | acados 锁定提交 | acados QP 求解 | 随 acados 安装 |

Ipopt 还使用发行版提供的 BLAS、LAPACK、顺序版 MUMPS 和 Scotch。UMFPACK 由
`px4ctrlrate_node` 直接使用。这些基础数值库仍作为系统包安装，不复制进仓库。
ROS 2、PX4 消息及工作空间内的消息/工具包也仍由 ROS 环境和当前工作空间提供。

不需要单独安装 qpOASES：当前 px4ctrl 源码没有调用 qpOASES API。过去出现
`qpOASES.hpp` 找不到，是历史遗留 include；CasADi 的 qpOASES 插件也不等价于
qpOASES C++ 开发包。

## 新机器首次安装

以下命令适用于 Ubuntu/Debian。先安装编译工具和由系统维护的基础库：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config autoconf automake libtool \
  gfortran libblas-dev liblapack-dev libmumps-seq-dev libscotch-dev \
  libmetis-dev libsuitesparse-dev python3-colcon-common-extensions
```

克隆主仓库后初始化顶层源码子模块，再初始化 acados 实际使用的两个内部依赖：

```bash
git submodule update --init
git -C 3rdpart/src/acados submodule update --init \
  external/blasfeo external/hpipm
```

然后原生编译并安装到 `/usr/local`：

```bash
JOBS=2 ./3rdpart/build_all.sh
```

无人机内存较小时建议 `JOBS=1` 或 `JOBS=2`。脚本会在安装阶段调用 `sudo`，最后
执行 `ldconfig`。源码会一直保留在 `3rdpart/src`，构建中间文件保留在被忽略的
`3rdpart/build`，所以失败后可以从断点继续。

如确实需要别的安装前缀：

```bash
PREFIX=/opt/manycontroller JOBS=2 ./3rdpart/build_all.sh
```

随后配置工程时把该前缀传给 CMake：

```bash
colcon build --packages-up-to px4ctrl --cmake-clean-cache \
  --cmake-args -DCMAKE_PREFIX_PATH=/opt/manycontroller
```

## CMake 如何找到库

`src/realflight_modules/px4ctrl/CMakeLists.txt` 不再拼接 `3rdpart` 路径，也不再设置
指向仓库二进制文件的 RPATH。它采用标准查找规则：

- `find_package(Eigen3 CONFIG REQUIRED)` -> `Eigen3::Eigen`；
- `find_package(casadi CONFIG REQUIRED)` -> `casadi::casadi`；
- `find_package(osqp 1.0 CONFIG REQUIRED)` -> `osqp::osqp`；
- `find_package(acados CONFIG REQUIRED)` -> `acados`、`hpipm`、`blasfeo`；
- `find_package(NLopt CONFIG REQUIRED)` -> `NLopt::nlopt`；
- `find_path`/`find_library` 查找 Ipopt 和 SuiteSparse/UMFPACK；
- 将工程内的 acados 生成 C 源码编译为 `px4ctrl_acados_ocp_solver`。

`/usr/local` 是 CMake 的标准系统前缀。安装脚本执行 `ldconfig` 后，编译和运行时
都无需写死用户目录。若旧构建缓存还记录着
`/home/.../manycontroller/3rdpart/...`，必须使用 `--cmake-clean-cache` 或删除工作
空间的 `build/px4ctrl` 后重新配置。

## 构建工作空间

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to px4ctrl --cmake-clean-cache
```

CMake 配置输出会打印最终使用的 Ipopt、NLopt 和 UMFPACK 路径。可以进一步检查：

```bash
ldconfig -p | grep -E 'casadi|acados|hpipm|blasfeo|osqp|ipopt|nlopt'
find /usr/local/include -maxdepth 3 \
  \( -name 'Eigen' -o -name 'osqp.h' -o -name 'IpIpoptApplication.hpp' \)
```

## 在另一台电脑同步更新

主仓库更新后执行：

```bash
git pull --ff-only
git submodule sync --recursive
git submodule update --init
git -C 3rdpart/src/acados submodule update --init \
  external/blasfeo external/hpipm
JOBS=2 ./3rdpart/build_all.sh
```

子模块记录的是精确提交，因此不会再出现 QDLDL 更新步骤把 `v0.1.8` 错当成
`origin/v0.1.8` 分支进行 rebase 的问题。`build_all.sh` 还通过
`FETCHCONTENT_SOURCE_DIR_QDLDL` 强制 OSQP 使用仓库中已锁定的 QDLDL 源码。

## 可选：重新生成 acados 模型代码

只有模型、代价或约束发生变化时才需要执行。现有生成 C 源码足以正常编译。
先准备 Python 环境并安装 acados_template：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install 'casadi==3.7.2' \
  ./3rdpart/src/acados/interfaces/acados_template
ACADOS_SOURCE_DIR="$PWD/3rdpart/src/acados" \
ACADOS_INSTALL_PREFIX=/usr/local \
python3 script/generate_px4ctrl_acados_nmpc.py
```

脚本的输出仍写入 px4ctrl 的 `generated/acados/px4ctrl_nmpc`。重新生成后应审查并
提交 C/H/JSON 的变化，不提交生成的目标文件或共享库。
