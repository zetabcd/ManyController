cmake_minimum_required(VERSION 3.5)
project(px4ctrl)

# sun: px4ctrl_node 运行外环与状态机，px4ctrlrate_node 运行高频角速度环和电机分配。
# sun: 两个可执行程序通过 /rates_thrust_setpoint 解耦，频率可分别配置。

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_ROOT = ${CMAKE_ROOT}")

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(fms_utils REQUIRED)
find_package(uav_utils REQUIRED)
find_package(px4debug_msgs REQUIRED)
find_package(joy_msgs REQUIRED)
find_package(ratectrl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# All nonlinear-solver dependencies are self-contained under the repository's
# 3rdpart directory.  Controller selection remains the single source macro in
# px4ctrlfsm.h; no CMake option is required.
get_filename_component(PX4CTRL_3RDPART_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdpart" ABSOLUTE)
set(PX4CTRL_OPTIMIZATION_ROOT "${PX4CTRL_3RDPART_ROOT}/optimization")
set(PX4CTRL_CASADI_ROOT "${PX4CTRL_3RDPART_ROOT}/casadi")
set(PX4CTRL_ACADOS_ROOT "${PX4CTRL_3RDPART_ROOT}/acados")
set(PX4CTRL_ACADOS_GENERATED
  "${PX4CTRL_ACADOS_ROOT}/generated/px4ctrl_nmpc")

find_path(IPOPT_INCLUDE_DIR NAMES IpIpoptApplication.hpp
  PATHS "${PX4CTRL_OPTIMIZATION_ROOT}/include/coin" NO_DEFAULT_PATH REQUIRED)
find_library(IPOPT_LIBRARY NAMES ipopt
  PATHS "${PX4CTRL_OPTIMIZATION_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_path(NLOPT_INCLUDE_DIR NAMES nlopt.hpp
  PATHS "${PX4CTRL_OPTIMIZATION_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(NLOPT_CXX_LIBRARY NAMES nlopt_cxx
  PATHS "${PX4CTRL_OPTIMIZATION_ROOT}/lib/x86_64-linux-gnu"
  NO_DEFAULT_PATH REQUIRED)
find_package(casadi CONFIG REQUIRED
  PATHS "${PX4CTRL_CASADI_ROOT}/cmake" NO_DEFAULT_PATH)
find_library(ACADOS_LIBRARY NAMES acados
  PATHS "${PX4CTRL_ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(HPIPM_LIBRARY NAMES hpipm
  PATHS "${PX4CTRL_ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(BLASFEO_LIBRARY NAMES blasfeo
  PATHS "${PX4CTRL_ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(PX4CTRL_ACADOS_SOLVER_LIBRARY
  NAMES acados_ocp_solver_px4ctrl_nmpc
  PATHS "${PX4CTRL_ACADOS_GENERATED}" NO_DEFAULT_PATH REQUIRED)

# gptmpc/gpttraj assemble QPs with Eigen and call the system OSQP C API.
find_path(OSQP_INCLUDE_DIR osqp/osqp.h PATHS /usr/local/include /usr/include REQUIRED)
find_library(OSQP_LIBRARY NAMES osqp PATHS /usr/local/lib /usr/lib REQUIRED)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${OSQP_INCLUDE_DIR}
)

# CasADi is isolated behind a plain-C bridge so no CasADi C++ type enters the
# public controller interface.
add_library(px4ctrl_casadi_nmpc_bridge STATIC
  src/solver_nmpc_casadi_bridge.cpp)
target_include_directories(px4ctrl_casadi_nmpc_bridge PRIVATE
  "${PX4CTRL_CASADI_ROOT}/include")
target_link_libraries(px4ctrl_casadi_nmpc_bridge PRIVATE casadi::casadi)

# sun: 顶层目标包含输入转换、状态机、外环控制器、参数和坐标变换实现。
add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
	src/input.cpp
  src/px4ctrlfsm.cpp
  src/controller.cpp
  src/px4ctrlparam.cpp
  src/frame_transforms.cpp
  src/ommpc_controller.cpp
  src/gptmpc.cpp
  src/gpttraj.cpp
  src/minimum_snap_trajectory.cpp
  src/barrel_roll_trajectory.cpp
  src/figure_eight_trajectory.cpp
  src/solver_nmpc_common.cpp
  src/solver_nmpc_ipopt_eigen.cpp
  src/solver_nmpc_nlopt_eigen.cpp
  src/solver_nmpc_casadi.cpp
  src/solver_nmpc_acados.cpp
)
ament_target_dependencies(px4ctrl_node 
  rclcpp 
  geometry_msgs
  sensor_msgs
  px4_msgs 
  fms_utils 
  uav_utils 
  px4debug_msgs 
  joy_msgs 
  ratectrl_msgs
  std_srvs
)
target_include_directories(px4ctrl_node SYSTEM PRIVATE
  "${IPOPT_INCLUDE_DIR}"
  "${NLOPT_INCLUDE_DIR}"
  "${PX4CTRL_ACADOS_GENERATED}"
  "${PX4CTRL_ACADOS_ROOT}/include"
  "${PX4CTRL_ACADOS_ROOT}/include/acados"
  "${PX4CTRL_ACADOS_ROOT}/include/blasfeo/include"
  "${PX4CTRL_ACADOS_ROOT}/include/hpipm/include")
target_compile_definitions(px4ctrl_node PRIVATE HAVE_CSTDDEF)
target_link_libraries(px4ctrl_node
  ${OSQP_LIBRARY}
  ${IPOPT_LIBRARY}
  ${NLOPT_CXX_LIBRARY}
  px4ctrl_casadi_nmpc_bridge
  ${PX4CTRL_ACADOS_SOLVER_LIBRARY}
  ${ACADOS_LIBRARY}
  ${HPIPM_LIBRARY}
  ${BLASFEO_LIBRARY}
  m)
# Solver plugins and their transitive libraries all live below 3rdpart.
target_link_options(px4ctrl_node PRIVATE "LINKER:--disable-new-dtags")
set_target_properties(px4ctrl_node PROPERTIES
  BUILD_RPATH
    "${PX4CTRL_OPTIMIZATION_ROOT}/lib;${PX4CTRL_OPTIMIZATION_ROOT}/lib/x86_64-linux-gnu;${PX4CTRL_ACADOS_GENERATED};${PX4CTRL_ACADOS_ROOT}/lib;/usr/local/lib;${PX4CTRL_CASADI_ROOT}"
  INSTALL_RPATH
    "${PX4CTRL_OPTIMIZATION_ROOT}/lib;${PX4CTRL_OPTIMIZATION_ROOT}/lib/x86_64-linux-gnu;${PX4CTRL_ACADOS_GENERATED};${PX4CTRL_ACADOS_ROOT}/lib;/usr/local/lib;${PX4CTRL_CASADI_ROOT}")

add_executable(gpttraj_visualizer_node
  src/gpttraj_visualizer_node.cpp
  src/gpttraj.cpp
  src/minimum_snap_trajectory.cpp
)
ament_target_dependencies(gpttraj_visualizer_node
  rclcpp
  geometry_msgs
  visualization_msgs
)
target_link_libraries(gpttraj_visualizer_node ${OSQP_LIBRARY})

# sun: 底层目标依赖 SuiteSparse/UMFPACK，为滑动窗口 TV 正则微分求解稀疏线性系统。
add_executable(px4ctrlrate_node 
  src/px4ctrlrate_node.cpp
  src/px4ctrlparam.cpp
  src/frame_transforms.cpp
)
ament_target_dependencies(px4ctrlrate_node 
  rclcpp px4_msgs 
  geometry_msgs
  sensor_msgs
  fms_utils 
  uav_utils 
  px4debug_msgs 
  joy_msgs 
  ratectrl_msgs
  std_srvs
)
target_link_libraries(px4ctrlrate_node umfpack)

install(TARGETS px4ctrl_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS px4ctrlrate_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS gpttraj_visualizer_node DESTINATION lib/${PROJECT_NAME})
# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
# Install launch and config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
