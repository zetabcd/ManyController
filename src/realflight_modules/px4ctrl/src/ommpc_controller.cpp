#include <cmath>
#include <cstddef>
#include <iostream>
#include <px4ctrl/ommpc_controller.h>
#include <px4ctrl/px4ctrl_node.h>
#include <qpOASES.hpp>
#include <uav_utils/geometry_utils.h>
#include <uav_utils/other_utils.h>
#include <px4ctrl/px4ctrlparam.h>
#include <px4ctrl/motor_calculate.h>



// using namespace frame_transforms::utils::quaternion;
using namespace uav_utils;



//要定义新的控制器参数，并在这里初始化
OmmpcControl::OmmpcControl(PX4ControlNode& px4controlnode) : px4controlnode_(px4controlnode)
{

}

void OmmpcControl::resetControlParams()
{

}

px4debug_msgs::msg::Px4ctrlDebug OmmpcControl::calculateControl(
    const Ref_State_t &ref, 
    const LocalPose_Data_t &pose, 
    const Attitude_Data_t &att,
    const Sensor_Data_t &sens, 
    const double &dt,
    Control_Setpoint_t &control_sp,  
    const Parameter_t &param)
{
    px4debug_msgs::msg::Px4ctrlDebug PD;

    return PD;

}

void OmmpcControl::init_filters(const Parameter_t &param)
{
    // sun: 滤波器采样频率与顶层控制频率一致，三个轴可按传感器噪声分别设截止频率。
    double fs = param.ctrl_freq_max;
    lpf_acc_x_ = std::make_unique<SecondOrderButterworthLPF>(param.filter.lpf_acc_x_cutoff_hz, fs);
    lpf_acc_y_ = std::make_unique<SecondOrderButterworthLPF>(param.filter.lpf_acc_y_cutoff_hz, fs);
    lpf_acc_z_ = std::make_unique<SecondOrderButterworthLPF>(param.filter.lpf_acc_z_cutoff_hz, fs);
}

void OmmpcControl::reset_filters()
{
    // sun: reset 只清内部历史而不重新分配对象，适合在模式切换的实时路径调用。
    lpf_acc_x_->reset();
    lpf_acc_y_->reset();
    lpf_acc_z_->reset();
}

/*
  compute throttle percentage 
*/
double 
OmmpcControl::computeDesiredCollectiveThrustSignal(const double &thrust_des, const double &uav_mass)
{
    // sun: 利用在线辨识的 thr2acc_ 将期望加速度换算为飞控推力信号；映射失效时
    // sun: 回退到牛顿制推力，避免 NaN 或负比例继续传播。
    double acc_z_des = thrust_des / uav_mass; // 期望的垂直加速度（不包含重力）
    double thrust(0.0);
    
    /* compute throttle, thr2acc has been estimated before */
    if (thr2acc_ >= 0 && isfinite(thr2acc_))
    {
        thrust = acc_z_des / thr2acc_;
    }else{
        thrust = thrust_des;
    }
    return thrust;
}

bool 
OmmpcControl::estimateThrustModel(const Eigen::Vector3d &est_a)
{
   // sun: 取约 35~45 ms 前发送的推力与当前加速度配对，以补偿电机、飞控和通信延迟。
  rclcpp::Time t_now = px4controlnode_.get_clock()->now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<rclcpp::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).seconds();
    if (time_passed > 0.045) // 45ms
    {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      return false;
    }

    // sun: 标量递推最小二乘只估计竖直通道比例，遗忘因子 rho2_ 允许模型缓慢跟踪
    // sun: 电池电压、载荷和桨效率变化，clip 则限制异常样本造成的单次漂移。
    double thr = t_t.second;
    timed_thrust_.pop();

    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    thr2acc_ = clip(thr2acc_, 0.2, 2.0); // 1.368
    P_ = (1 - K * thr) * P_ / rho2_;
    // printf("%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_, est_a(2));
    return true;
  }
  return false;
}

void 
OmmpcControl::resetThrustMapping(const Parameter_t &param)
{
    // sun: 初值采用单位推力对应 1/mass 的比例，P_ 较大表示启动阶段估计不确定度高。
    thr2acc_ = 1.0 / param.uav.mass;
    P_ = 1e6;
}  
