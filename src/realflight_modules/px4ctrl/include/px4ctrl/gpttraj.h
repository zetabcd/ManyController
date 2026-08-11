#ifndef PX4CTRL_GPTTRAJ_H_
#define PX4CTRL_GPTTRAJ_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <functional>
#include <string>
#include <vector>

// 无人机流形轨迹优化器（对应 wenzhang0710.docx / Lu.pdf 的无人机部分）。
//
// 坐标约定：位置、速度均在工程的 ENU 世界系表达；姿态四元数 q 对应的
// 旋转矩阵 R 将机体 FLU 向量变换到 ENU。动力学及优化器的实际控制输入为
//
//     u = [a_T, omega_x, omega_y, omega_z],
//
// 其中 a_T 是沿机体 +Z 方向的质量归一化总拉力，omega 是机体系角速度。
// 这与工程底层控制器所需的“拉力 + 角速度”接口直接对应。
struct GptTrajectoryState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double time{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
  double thrust_acceleration{9.805};  // a_T [m/s^2], positive along body +z
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
};

struct GptTrajectoryWaypoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  // 到点约束使用球区域 ||p-p_wp||_2 <= tolerance，而不是固定到某个离散时刻。
  double tolerance{0.05};
};

struct GptTrajectoryBoundary
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
};

struct GptTrajectoryIteration;

struct GptTrajectoryOptions
{
  // 时间离散为 N=intervals 段、N+1 个状态节点，dt=T/N。
  int intervals{60};
  int max_scp_iterations{12};
  int max_time_search_iterations{16};
  double gravity{9.805};
  double minimum_time{0.5};
  double maximum_time{20.0};
  double initial_speed{2.0};
  double time_search_tolerance{2.0e-2};
  double convergence_tolerance{2.0e-3};
  double dynamics_tolerance{5.0e-2};
  double position_trust_region{1.0};
  double velocity_trust_region{3.0};
  double attitude_trust_region{0.7};
  double thrust_trust_region{8.0};
  Eigen::Vector3d body_rate_trust_region{Eigen::Vector3d::Constant(4.0)};
  int scp_backtracking_steps{6};
  double thrust_acceleration_min{0.5};
  double thrust_acceleration_max{4.0 * 9.805};
  Eigen::Vector3d body_rate_max{Eigen::Vector3d::Constant(14.0)};
  double state_regularization{1.0};
  // Inner fixed-T tie-breakers. Because T is fixed and minimized only by the
  // outer bisection, these terms cannot trade smoothness against flight time.
  double input_regularization{0.05};
  double input_smoothness{0.2};
  double virtual_control_weight{1.0e5};
  // Strict minimum time is solved lexicographically: an outer bisection minimizes
  // T and each inner SCP/OSQP problem only tests feasibility at fixed T.
  bool optimize_total_time{true};
  // Optional actuator-bandwidth constraints. They are disabled by default because
  // the paper-level control input is u=[a_T,omega], whose box bounds are sufficient.
  bool enforce_input_rate_constraints{false};
  bool enforce_hover_boundary_input{false};
  double thrust_acceleration_rate_max{12.0};  // |dot(a_T)| [m/s^3]
  Eigen::Vector3d body_rate_acceleration_max{Eigen::Vector3d::Constant(20.0)};  // [rad/s^2]
  // CSTC 进度变量使优化器自行选择每个航点的通过节点/时刻，并保持给定顺序。
  bool enable_cstc{true};
  bool enforce_strict_waypoint_order{true};
  // Identify passage nodes with CSTC, then lock that complementarity active set
  // during outer time bisection to prevent false infeasibility from mode switching.
  bool lock_cstc_active_set_for_time_search{true};
  // If a locked passage schedule fails at a trial time, reopen CSTC once instead
  // of immediately declaring that time physically infeasible.
  bool retry_cstc_on_locked_failure{true};
  double progress_trust_region{0.35};
  double progress_regularization{1.0};
  int cstc_warm_start_iterations{3};
  // Spread the initial progress impulse over nearby nodes.  A nonzero support
  // avoids the zero-gradient deadlock of one-hot complementarity initialization.
  int cstc_initial_support_radius{2};
  // Homotopy sigma_i in mu*g<=sigma_i+s and lambda*mu_next<=sigma_i+s.
  // It is reduced geometrically and never used by the final feasibility test.
  double cstc_relaxation_initial{5.0e-2};
  double cstc_relaxation_decay{0.5};
  double cstc_slack_weight{2.0e5};
  double cstc_tolerance{1.0e-4};
  double cstc_slack_tolerance{1.0e-4};
  // Called synchronously after every accepted SCP step for live progress output.
  std::function<void(const GptTrajectoryIteration &)> progress_callback;
};

struct GptTrajectoryIteration
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int time_attempt{0};
  int scp_iteration{0};
  double total_time{0.0};
  double maximum_update{0.0};
  double maximum_dynamics_defect{0.0};
  double maximum_virtual_control{0.0};
  double maximum_waypoint_residual{0.0};
  double maximum_order_residual{0.0};
  double maximum_cstc_slack{0.0};
  double step_solve_time{0.0};
  double qp_solve_time{0.0};
  double elapsed_solve_time{0.0};
  std::string solver_status;
  std::vector<GptTrajectoryState, Eigen::aligned_allocator<GptTrajectoryState>> states;
};

struct GptTrajectoryResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool success{false};
  // success means constraint-feasible; converged additionally requires the SCP
  // update norm to satisfy convergence_tolerance before the iteration limit.
  bool converged{false};
  std::string status;
  int scp_iterations{0};
  double total_time{0.0};
  double maximum_update{0.0};
  double maximum_dynamics_defect{0.0};
  double maximum_virtual_control{0.0};
  double maximum_waypoint_residual{0.0};
  double maximum_order_residual{0.0};
  double maximum_cstc_slack{0.0};
  double total_solve_time{0.0};
  std::vector<double> waypoint_times;
  // Retained for exact CSTC warm starts during outer minimum-time bisection.
  Eigen::MatrixXd progress_lambda;
  Eigen::MatrixXd progress_mu;
  std::vector<GptTrajectoryState, Eigen::aligned_allocator<GptTrajectoryState>> states;
  // 保存每次 SCP 迭代，供残差诊断及 RViz 动画回放优化过程。
  std::vector<GptTrajectoryIteration, Eigen::aligned_allocator<GptTrajectoryIteration>> history;
};

// 在 R^3 x R^3 x SO(3) 上进行序列凸化（SCP）。每个凸子问题均组装为
//   min 0.5*z'P*z + q'z,  s.t. l <= A*z <= u,
// 使用 Eigen 稀疏矩阵构造 P/A，使用 OSQP 求解。
class GptTrajectoryOptimizer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit GptTrajectoryOptimizer(const GptTrajectoryOptions &options = {});

  void setOptions(const GptTrajectoryOptions &options);
  const GptTrajectoryOptions &options() const { return options_; }

  GptTrajectoryResult optimize(
    const GptTrajectoryBoundary &initial,
    const GptTrajectoryBoundary &terminal,
    const std::vector<GptTrajectoryWaypoint,
      Eigen::aligned_allocator<GptTrajectoryWaypoint>> &waypoints);

  // Interpolates position/velocity/input and performs a shortest-path SLERP.
  static GptTrajectoryState sample(const GptTrajectoryResult &trajectory, double time);

private:
  GptTrajectoryOptions options_;
};

#endif  // PX4CTRL_GPTTRAJ_H_
