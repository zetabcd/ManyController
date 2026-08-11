#ifndef __OMMPC_CONTROLLER_H
#define __OMMPC_CONTROLLER_H

// sun: 该头文件预留 OMMPC 外环控制器接口，目前数据结构与 QuadControl 保持兼容，
// sun: 便于在不修改状态机和底层角速度环的情况下替换顶层控制算法。

#include <rclcpp/time.hpp>
#include <Eigen/Dense>
#include <queue>
#include <cmath>
#include <osqp/osqp.h>
#include <cfloat>

#include <px4ctrl/controller.h>
#include <px4debug_msgs/msg/px4ctrl_debug.hpp>
#include <px4ctrl/px4ctrlparam.h>
#include <px4ctrl/input.h>
#include <px4ctrl/frame_transforms.h>
#include <fms_utils/openfsm.h>

// #define USE_OSQP     // 决定是否使用OSQP求解器
// #define USE_NORMAL      // 决定是否使用归一化推力（推荐）

class PX4ControlNode;

using namespace frame_transforms::utils::quaternion;



class OmmpcControl
{
public:
    // sun: calculateControl() 是状态机唯一调用入口，输入全部为只读快照，结果写入 control_sp。
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OmmpcControl(PX4ControlNode &);
    px4debug_msgs::msg::Px4ctrlDebug calculateControl(
        const Ref_State_t &des,
        const LocalPose_Data_t &pose,
        const Attitude_Data_t &att,
        const Sensor_Data_t &sens,
		const double &dt,
        Control_Setpoint_t &control_sp,
        const Parameter_t &param
    );
	void resetControlParams();
	double computeDesiredCollectiveThrustSignal(const double &thrust_des, const double &uav_mass);
	bool estimateThrustModel(const Eigen::Vector3d &est_a);
	void resetThrustMapping(const Parameter_t &param);
	void init_filters(const Parameter_t &param);
	void reset_filters();
	
private:
    enum procedure_id_ {
        FSM_STATE(manual_on),
        FSM_STATE(manual), 
        FSM_STATE(auto_hover),
        FSM_STATE(cmd),
        FSM_STATE(safe),
        FSM_STATE(err),
    };
    PX4ControlNode& px4controlnode_;
    px4debug_msgs::msg::Px4ctrlDebug debug_msg_;

    std::queue<std::pair<rclcpp::Time, double>> timed_thrust_;
	// sun: 低于该阈值的推力样本激励不足，不适合用于在线辨识推力—加速度比例。
	static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;



    // sun: RLS 在线估计参数；thr2acc_ 表示单位推力信号对应的竖直加速度。
	const double rho2_ = 0.998; // do not change
	double thr2acc_;
	double P_;

	// sun: 三轴加速度分别滤波，降低速度环微分反馈对 IMU 高频噪声的放大。
	std::unique_ptr<SecondOrderButterworthLPF> lpf_acc_x_;
	std::unique_ptr<SecondOrderButterworthLPF> lpf_acc_y_;
	std::unique_ptr<SecondOrderButterworthLPF> lpf_acc_z_;



};


#endif
