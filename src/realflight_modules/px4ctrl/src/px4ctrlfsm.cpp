#include "px4ctrl/input.h"
#include <Eigen/src/Core/Matrix.h>
#include <px4ctrl/px4ctrlfsm.h>
#include <px4ctrl/px4ctrl_node.h>
#include <uav_utils/geometry_utils.h>

#include <algorithm>
#include <utility>
#include <vector>

#define pi acos(-1)
#define deg2rad(x) x/180.0*pi

// sun: 状态机的安全层级为 manual_on（PX4 接管）-> manual/hover/cmd（外部控制）
// sun: -> safe（受控降落）-> err（框架故障）。每个状态函数在一次 process() 中只执行一步。

using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(PX4ControlNode& px4controlnode) : 
    rc_data(px4controlnode), sta_data(px4controlnode), bat_data(px4controlnode),
    sens_data(px4controlnode), att_data(px4controlnode), pose_data(px4controlnode),
    controller(px4controlnode), px4controlnode_(px4controlnode) 
{
    // sun: procedure_list_ 的顺序必须与头文件 procedure_id_ 枚举一致，否则状态号会调用错误处理函数。
    start_time_ = px4controlnode_.get_clock()->now();
    last_time_ = px4controlnode_.get_clock()->now();

    procedure_list_[0] = std::bind(&PX4CtrlFSM::FSM_FUNCT(manual_on), this, std::placeholders::_1);
    procedure_list_[1] = std::bind(&PX4CtrlFSM::FSM_FUNCT(manual), this, std::placeholders::_1);
    procedure_list_[2] = std::bind(&PX4CtrlFSM::FSM_FUNCT(auto_hover), this, std::placeholders::_1);
    procedure_list_[3] = std::bind(&PX4CtrlFSM::FSM_FUNCT(cmd), this, std::placeholders::_1);
    procedure_list_[4] = std::bind(&PX4CtrlFSM::FSM_FUNCT(safe), this, std::placeholders::_1);
    procedure_list_[5] = std::bind(&PX4CtrlFSM::FSM_FUNCT(err), this, std::placeholders::_1);

    // 设置 状态机
    set_procedures(&fsm_, procedure_list_);
    set_data_entry(&fsm_, &rc_data);
#ifdef USE_WITHOUT_RC
    set_default_state(&fsm_, FSM_STATE(manual));
#else
    set_default_state(&fsm_, FSM_STATE(manual_on));
#endif  
    set_err_var(&fsm_, &err_code_);
    clr_fsm_error_flag(&fsm_);

    service_result_ = 0;
    service_done = false;

    record_state_data.set_zero();
}


void PX4CtrlFSM::process()
{
    // sun: 每周期先更新时间和悬停油门点，再步进 FSM；dt_ 同时供积分器和参考生成使用。
    rc_data.hover_percentage = px4controlnode_.param.motor.hover_percentage;
    now_time = px4controlnode_.get_clock()->now();
    t_ = (now_time - start_time_).seconds();
    dt_ = (now_time - last_time_).seconds();
    last_time_ = now_time;

    // 执行状态机，使其步进一次
    cur_state_ = run_state_machine_once(&fsm_);
    // sun: 通用 FSM 错误标志表示状态索引或过程调用异常，读取后清除以免重复报告同一错误。
    if(is_fsm_error(&fsm_))
    {
        printf("Error when Stepping !\n");
        got_err_ = (int *)get_err_var(&fsm_);
        printf("Get Error Code :0x%x\n", *got_err_);
        clr_fsm_error_flag(&fsm_);
        // rclcpp::shutdown();
    }
    
    debug_msg.voltage = bat_data.voltage_filtered_v;
    debug_msg.state = get_curr_state(&fsm_);
    px4ctrldebug_publisher->publish(debug_msg);
#ifdef USE_WITHOUT_RC
    // sun: 无遥控编译模式用合成摇杆消息复用正常状态切换逻辑，避免维护另一套启动流程。
    auto joy_empty_msg = std::make_unique<joy_msgs::msg::JoyStick>();
    joy_empty_msg->aux1 = 1.0;
    joy_empty_msg->aux2 = -1.0;
    double enter_auto_hover_after_launch = 0.1; // 秒
    double enter_cmd_after_hover = 2.0; // 秒
    if (px4controlnode_.param.tuning.attitude_loop || px4controlnode_.param.tuning.angular_rate_loop)
    {
        enter_auto_hover_after_launch = 3.0;
    }
    if (t_ >= enter_auto_hover_after_launch)
    {
        // 切换到 auto_hover
        joy_empty_msg->aux2 = 0.0;

        if (!px4controlnode_.param.tuning.attitude_loop && 
            !px4controlnode_.param.tuning.angular_rate_loop){
                if (t_ > (enter_auto_hover_after_launch + enter_cmd_after_hover))
                {
                    // 切换到 cmd
                    joy_empty_msg->aux2 = 1.0;
                }
            }
    }
    rc_data.feed(std::move(joy_empty_msg));//这里定义没有RC的时候自己给rc回调函数赋值
#endif
}

void PX4CtrlFSM::reset_start_time()
{
    // sun: 重置任务相对时间，后续轨迹以新的状态进入时刻作为 t=0。
    start_time_ = px4controlnode_.get_clock()->now();
}
/* MANUAL(ONBOARD)模式 */ 
void* PX4CtrlFSM::FSM_FUNCT(manual_on)(void * this_fsm)
{   
    // sun: manual_on 表示 PX4 本机模式仍掌握执行器；控制节点仅监视输入并等待切入 OFFBOARD。
    /* 状态切换 */ 
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    if(pd->aux1_changed){
        switch (pd->aux1){
            case GEARS::DOWN:
                // 只要一切到down都落入这里
                set_default_state(&fsm_, FSM_STATE(manual_on));
                break;
            case GEARS::UP:
                // sun: 切入 OFFBOARD 前要求位置、姿态和 IMU 都新鲜，避免用默认状态闭环。
                if (!pose_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject MANUAL(OFF). No pose!\033[0m");
                    break;
                }
                if (!att_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject MANUAL(OFF). No attitude!\033[0m");
                    break;
                }
                if (!sens_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject MANUAL(OFF). No imu!\033[0m");
                    break;
                }

                // 如果上个状态不是CMD。不用判断，因为不可能进入到这种模式
                // if (get_last_state((FSM *)this_fsm) != FSM_STATE(cmd))
                // {
                    // sun: PX4 要求切换前已收到控制模式心跳，因此先发布一次再发模式请求。
                    publish_offboard_control_mode_(); // 切换前要发一次
                    if (switch_to_offboard_mode_())
                    {
                        set_last_state((FSM *)this_fsm);
                        set_next_state((FSM *)this_fsm, FSM_STATE(manual));
                        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(ON) --> MANUAL(OFF)\033[0m");
                        service_done = false;
#if !PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
                        // [LEGACY QuadControl] 在线推力映射复位。
                        controller.resetThrustMapping(px4controlnode_.param);
#endif
                    }           
                // }
                return NULL;
                break;
            default:
                break;
        }
    }

    //正常不进入if的时候执行，也就是要控制不动的时候执行下面这段代码
#ifndef USE_WITHOUT_RC
    /* 故障处理 */
    if (!rc_is_received(px4controlnode_.get_clock()->now()))
    {
        // set_last_state((FSM *)this_fsm);
        set_next_state((FSM *)this_fsm, FSM_STATE(err));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is disconnected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(ON) --> ERROR\033[0m");
    }
#endif
    /* 任务 */ 
    control_sp_ = Control_Setpoint_t();
    publish_rates_thrust_setpoint();
    return NULL;
    
}

/* MANUAL(OFFBOARD)模式 */ 
void* PX4CtrlFSM::FSM_FUNCT(manual)(void * this_fsm)
{
    // sun: manual 是 OFFBOARD 下的遥控姿态模式；首次进入时锁定现场状态并清控制器历史。
    if (get_last_state((FSM *)this_fsm) != FSM_STATE(manual))
    {
#if PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
        // [GPT MPC] 离开 CMD 后必须清除整条轨迹，否则轨迹源优先级高于 Ref_State_t。
        controller.clearTrajectory();
#endif
        set_init_ref();
        record_position();
        controller.resetControlParams();
#ifndef USE_WITHOUT_RC
        set_manual_ref(dt_, true);
#endif
        set_last_state((FSM *)this_fsm);
    }
    /* 状态切换 */ 
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    // std::cout << "pd->aux1:" << pd->aux1 << std::endl;
    // std::cout << "pd->aux2:" << pd->aux2 << std::endl;
    if(pd->aux1_changed){
        switch (pd->aux1){
            case GEARS::DOWN:
                if (switch_to_manual_mode_())
                {
                    // set_last_state((FSM *)this_fsm);
                    set_next_state((FSM *)this_fsm, FSM_STATE(manual_on));
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(OFF) --> MANUAL(ON)\033[0m");
                    service_done = false;
                }
                return NULL;
                break;
            
            default:
                break;
        }
    }
    if(pd->aux2_changed){
        switch (pd->aux2){
            case GEARS::MID:
                // sun: 悬停属于位置闭环，除数据新鲜度外还要求 PX4 的位置有效标志成立。
                if (!pose_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject AUTO_HOVER. No pose!\033[0m");
                    break;
                }
                if (!att_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject AUTO_HOVER. No attitude!\033[0m");
                    break;
                }
                if (!sens_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject AUTO_HOVER. No imu!\033[0m");
                    break;
                }
                if (!pose_is_valid(pose_data))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject AUTO_HOVER. Local position is invalid!\033[0m");
                    break;
                }

                if (pd->aux2_has_downed)
                {
                    // set_last_state((FSM *)this_fsm);
                    set_next_state((FSM *)this_fsm, FSM_STATE(auto_hover));
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(OFF) --> AUTO_HOVER\033[0m");
                }
                return NULL;
                break;
            default:
                break;
        }
    }
#ifndef USE_WITHOUT_RC
    /* 故障处理 */
    if (!rc_is_received(px4controlnode_.get_clock()->now()) || (pd->aux6 == GEARS::UP))
    {
        // set_last_state((FSM *)this_fsm);
        set_next_state((FSM *)this_fsm, FSM_STATE(safe));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is disconnected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] MANUAL(OFF) --> SAFE\033[0m");
    }
#endif
    /* 任务 */
    publish_offboard_control_mode_(); // 理论上每隔2Hz发一次
#ifdef USE_WITHOUT_RC
    control_sp_ = Control_Setpoint_t();
    control_sp_.thrust = px4controlnode_.param.uav.mass * px4controlnode_.param.gra;
    control_sp_.bodyrates = Eigen::Vector3d::Zero();
#else
    set_manual_ref(dt_,false);
    debug_msg = controller.calculateControl(
        ref_, 
        pose_data, 
        att_data, 
        sens_data, 
        dt_,
        control_sp_,  
        px4controlnode_.param);
#endif
    publish_rates_thrust_setpoint();
    return NULL;
}

/* AUTO_HOVER模式 */ 
void* PX4CtrlFSM::FSM_FUNCT(auto_hover)(void * this_fsm)
{
    // sun: auto_hover 锁定进入状态时的位置和航向，并在稳定工况下持续辨识推力映射。
    /*  刚进入状态 */
    if (get_last_state((FSM *)this_fsm) != FSM_STATE(auto_hover))
    {
#if PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
        // [GPT MPC] 非 CMD 模式使用 Ref_State_t 局部参考，不保留旧轨迹。
        controller.clearTrajectory();
#endif
        set_init_ref();
        record_position();
        controller.resetControlParams();
#if !PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
        // [LEGACY QuadControl] 悬停阶段复位推力映射。
        controller.resetThrustMapping(px4controlnode_.param);
#endif
        set_last_state((FSM *)this_fsm);
    }
    /* 状态切换 */ 
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    if(pd->aux1_changed){
        switch (pd->aux1){
            case GEARS::DOWN:
                if (switch_to_manual_mode_())
                {
                    set_next_state((FSM *)this_fsm, FSM_STATE(manual_on));
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] AUTO_HOVER --> MANUAL(ON)\033[0m");
                    service_done = false;
                }
                return NULL;
                break;
            default:
                break;
        }
    }
    if(pd->aux2_changed){
        switch (pd->aux2){
            case GEARS::UP:
                // sun: CMD 轨迹会用到 p/v/a 多阶反馈，因此再次完整检查估计数据。
                if (!pose_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject CMD_CTRL. No pose!\033[0m");
                    break;
                }
                if (!att_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject CMD_CTRL. No attitude!\033[0m");
                    break;
                }
                if (!sens_is_received(now_time))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject CMD_CTRL. No imu!\033[0m");
                    break;
                }
                if (!pose_is_valid(pose_data))
                {
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[31m[px4ctrl] Reject CMD_CTRL. Local position is invalid!\033[0m");
                    break;
                }
                if (get_last_state((FSM *)this_fsm) == FSM_STATE(cmd)) // 目标丢失切回来的情况
                {
                    break;
                }

                set_next_state((FSM *)this_fsm, FSM_STATE(cmd));
                RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] AUTO_HOVER --> CMD_CTRL\033[0m");
                return NULL;
                break;
            case GEARS::DOWN:
                if (switch_to_manual_mode_())
                {
                    set_next_state((FSM *)this_fsm, FSM_STATE(manual));
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] AUTO_HOVER --> MANUAL(OFF)\033[0m");
                    service_done = false;
                }
                return NULL;
                break;
            default:
                break;
        }
    }
#ifndef USE_WITHOUT_RC
    /* 故障处理 */
    if (!rc_is_received(px4controlnode_.get_clock()->now()) || (pd->aux6 == GEARS::UP))
    {
        set_next_state((FSM *)this_fsm, FSM_STATE(safe));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is disconnected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] AUTO_HOVER --> SAFE\033[0m");
    }
#endif
    /* 任务 */
    // [LEGACY QuadControl] 先用当前 IMU 更新推力模型；GPT MPC 不使用该映射。
    // set_hover_ref();
    // set_point_hover(0,0,0.5);
    set_manual_postion_ref(dt_,false);
#if !PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
    controller.estimateThrustModel(sens_data.a);
#endif
    debug_msg = controller.calculateControl(
        ref_, 
        pose_data, 
        att_data, 
        sens_data, 
        dt_,
        control_sp_,
        px4controlnode_.param);

    // RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31mJust for debug!!!\033[0m");

    publish_rates_thrust_setpoint();

    return NULL;
}

/* CMD模式 */ 
// 不进行在线推力系数辨识，使用悬停时的值
void* PX4CtrlFSM::FSM_FUNCT(cmd)(void * this_fsm)
{   
    // RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31mJust for debug11111111111111!!!\033[0m");
    // [ACTIVE GptMpcControl] CMD 首次进入时生成并一次性装载所选离散轨迹；
    // 后续控制周期只按 elapsed time 采样 H+1 个预瞄点，不会重复生成轨迹。
    if (get_last_state((FSM *)this_fsm) != FSM_STATE(cmd))
    {
        record_position();
#if PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
        // ref_ 在轨迹激活时只提供 FSM 状态等兼容字段，真正的 p/v/R/u 参考来自
        // GptMpcControl 内部缓存的 GptTrajectoryResult。
        set_hover_ref();
#endif
        controller.resetControlParams();
#if PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
#if PX4CTRL_CMD_TRAJECTORY == 2
        if (!load_figure_eight_cmd_trajectory_()) {
            RCLCPP_ERROR(
                px4controlnode_.get_logger(),
                "[px4ctrl] Cannot enter CMD: figure-eight trajectory generation failed");
            set_next_state((FSM *)this_fsm, FSM_STATE(auto_hover));
            return NULL;
        }
#elif PX4CTRL_CMD_TRAJECTORY == 1
        if (!load_barrel_roll_cmd_trajectory_()) {
            RCLCPP_ERROR(
                px4controlnode_.get_logger(),
                "[px4ctrl] Cannot enter CMD: barrel-roll trajectory generation failed");
            set_next_state((FSM *)this_fsm, FSM_STATE(auto_hover));
            return NULL;
        }

#elif PX4CTRL_CMD_TRAJECTORY == 0
        // [ALTERNATIVE] 原 minimum-snap CMD 轨迹完整保留；将轨迹宏改为 0 即启用。
        if (!load_minimum_snap_cmd_trajectory_()) {
            RCLCPP_ERROR(
                px4controlnode_.get_logger(),
                "[px4ctrl] Cannot enter CMD: minimum-snap trajectory generation failed");
            set_next_state((FSM *)this_fsm, FSM_STATE(auto_hover));
            return NULL;
        }
#else
#error "PX4CTRL_CMD_TRAJECTORY must be 0 (minimum-snap), 1 (barrel roll), or 2 (figure eight)"
#endif
#endif
        set_last_state((FSM *)this_fsm);
    }
    /* 状态切换 */ 
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    if(pd->aux1_changed){
        switch (pd->aux1){
            case GEARS::DOWN:
                if (switch_to_manual_mode_())
                {
                    // set_last_state((FSM *)this_fsm);
                    set_next_state((FSM *)this_fsm, FSM_STATE(manual_on));
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] CMD_CTRL --> MANUAL(ON)\033[0m");
                    service_done = false;
                }
                return NULL;
                break;
            
            default:
                break;
        }
    }
    if(pd->aux2_changed){
        switch (pd->aux2){
            case GEARS::MID:
                if (switch_to_manual_mode_())
                {
                    // set_last_state((FSM *)this_fsm);
                    set_next_state((FSM *)this_fsm, FSM_STATE(manual));
                    RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] CMD_CTRL --> MANUAL(OFF)\033[0m");
                    service_done = false;
                }
                return NULL;
                break;
            
            default:
                break;
        }
    }
#ifndef USE_WITHOUT_RC
    /* 故障处理 */
    if (!rc_is_received(px4controlnode_.get_clock()->now()) || (pd->aux6 == GEARS::UP))
    {
        // set_last_state((FSM *)this_fsm);
        set_next_state((FSM *)this_fsm, FSM_STATE(safe));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is disconnected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] CMD_CTRL --> SAFE\033[0m");
    }
#endif
    /* 任务 */
    publish_offboard_control_mode_(); // 理论上每隔2Hz发一次
#if !PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
    // [LEGACY QuadControl] 原解析 8 字轨迹入口完整保留；切换宏为 0 后自动启用。
    px4controlnode_.init_param();
    set_2D8_ref();
#endif
    debug_msg = controller.calculateControl(
        ref_, 
        pose_data, 
        att_data, 
        sens_data, 
        dt_,
        control_sp_, 
        px4controlnode_.param);
    publish_rates_thrust_setpoint();
    return NULL;
}

/* 安全模式 */
void* PX4CtrlFSM::FSM_FUNCT(safe)(void* this_fsm)
{
    // sun: safe 先保持进入点附近姿态；当速度估计无效或速度已降到阈值内，再转为恒速下降。
    RC_Data_t *pd = (RC_Data_t *)get_data_entry((FSM *)this_fsm);
    static bool executed = false;
    /*  刚进入状态 */
    if (get_last_state((FSM *)this_fsm) != FSM_STATE(safe))
    {
#if PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
        // [GPT MPC] 安全/降落参考必须覆盖 CMD 轨迹源。
        controller.clearTrajectory();
#endif
        record_position();
        controller.resetControlParams();
#if !PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
        // [LEGACY QuadControl] 安全模式恢复推力映射。
        controller.resetThrustMapping(px4controlnode_.param);
#endif
        set_last_state((FSM *)this_fsm);
        executed = false;
    }
    if (rc_is_received(px4controlnode_.get_clock()->now()) && (pd->aux6 == GEARS::DOWN))
    {
        set_last_state((FSM *)this_fsm);
        set_next_state((FSM *)this_fsm, FSM_STATE(manual));
        RCLCPP_INFO(px4controlnode_.get_logger(),"\033[31m[PX4CTRL] RC is connected!\033[0m");
        RCLCPP_INFO(px4controlnode_.get_logger(), "\033[32m[px4ctrl] CMD_CTRL --> MANUAL(OFF)\033[0m");
        px4controlnode_.init_param();
        return NULL;
    }

    set_hover_ref();
    if (!pose_data.v_xy_valid || pose_data.v.norm() < 3.0) // 速度小于3m/s或者速度估计无效时
    {
        px4controlnode_.init_param();
        if (!executed) {
            // sun: 下降起点只记录一次，否则每周期重置时间会使下降参考始终停在原处。
            record_position();
            executed = true;
        }
        set_land_ref();
    }
    debug_msg = controller.calculateControl(
        ref_, 
        pose_data, 
        att_data, 
        sens_data, 
        dt_,
        control_sp_, 
        px4controlnode_.param);
    publish_rates_thrust_setpoint();

    return NULL;
}

#if PX4CTRL_USE_GPT_MPC_PRIMARY_CONTROLLER
bool PX4CtrlFSM::load_figure_eight_cmd_trajectory_()
{
    FigureEightTrajectoryOptions options;
    // 八字长轴沿进入 CMD 时的机头水平投影，避免装载轨迹瞬间改变参考偏航。
    const double initial_yaw = get_yaw_from_quaternion(record_state_data.q);
    options.forward_axis =
        Eigen::Vector3d(std::cos(initial_yaw), std::sin(initial_yaw), 0.0);

    // 参数在节点启动时声明，进入 CMD 时读取当前值；因此也可以先用
    // `ros2 param set /px4ctrl_node trajectory.figure_eight.speed ...` 调整，
    // 再重新进入 CMD 生成新轨迹，无需重新编译。
    px4controlnode_.get_parameter(
        "trajectory.figure_eight.takeoff_height", options.takeoff_height);
    px4controlnode_.get_parameter(
        "trajectory.figure_eight.takeoff_duration", options.takeoff_duration);
    px4controlnode_.get_parameter(
        "trajectory.figure_eight.settle_duration", options.takeoff_settle_duration);
    px4controlnode_.get_parameter("trajectory.figure_eight.length", options.length);
    px4controlnode_.get_parameter("trajectory.figure_eight.width", options.width);
    px4controlnode_.get_parameter("trajectory.figure_eight.speed", options.speed);
    options.laps = static_cast<int>(
        px4controlnode_.get_parameter("trajectory.figure_eight.laps").as_int());
    options.sample_dt = 1.0 / std::max(1.0, px4controlnode_.param.ctrl_freq_max);
    options.gravity = px4controlnode_.param.gra;

    GptTrajectoryResult trajectory =
        generateFigureEightTrajectory(record_state_data.p, options);
    if (!trajectory.success) {
        RCLCPP_ERROR(
            px4controlnode_.get_logger(), "[px4ctrl] Figure-eight failed: %s",
            trajectory.status.c_str());
        return false;
    }

    const std::size_t state_count = trajectory.states.size();
    const double total_time = trajectory.total_time;
    // 和其他轨迹相同：完整缓存只装载一次，在线 MPC 每周期仅采样 H+1 个预瞄点。
    controller.setTrajectory(std::move(trajectory), px4controlnode_.get_clock()->now());
    RCLCPP_INFO(
        px4controlnode_.get_logger(),
        "[px4ctrl] Takeoff + figure-eight activated: height=%.2f m, size=%.2fx%.2f m, "
        "max_speed=%.2f m/s, laps=%d, %zu samples, %.3f s",
        options.takeoff_height, options.length, options.width, options.speed,
        options.laps, state_count, total_time);
    return true;
}

bool PX4CtrlFSM::load_barrel_roll_cmd_trajectory_()
{
    BarrelRollTrajectoryOptions options;
    // 在原 barrel_roll_trajectory_preview_node 参数前增加平滑起飞和短暂稳定段。
    // 起点使用进入 CMD 时的实际位置；滚转轴沿当前机头水平投影，避免偏航跳变。
    const double initial_yaw = get_yaw_from_quaternion(record_state_data.q);
    options.roll_axis = Eigen::Vector3d(std::cos(initial_yaw), std::sin(initial_yaw), 0.0);
    // 先用 2.5 s 竖直上升 1 m，再悬停 0.5 s；起飞段首末端 v/a/jerk
    // 均为零，飞机不会从地面直接进入水平加速或滚转。
    options.takeoff_height = 1.0;
    options.takeoff_duration = 2.5;
    options.takeoff_settle_duration = 0.5;
    options.radius = 1.0;
    options.axial_speed = 0.5;
    options.entry_duration = 1.5;
    options.roll_duration = 3.5;
    options.exit_duration = 1.5;
    options.turns = 1;
    options.polynomial_segments_per_turn = 32;
    options.sample_dt = 1.0 / std::max(1.0, px4controlnode_.param.ctrl_freq_max);
    options.gravity = px4controlnode_.param.gra;

    GptTrajectoryResult trajectory =
        generateBarrelRollTrajectory(record_state_data.p, options);
    if (!trajectory.success) {
        RCLCPP_ERROR(
            px4controlnode_.get_logger(), "[px4ctrl] Barrel-roll failed: %s",
            trajectory.status.c_str());
        return false;
    }

    const std::size_t state_count = trajectory.states.size();
    const double total_time = trajectory.total_time;
    // setTrajectory() 一次接收整条缓存；在线 MPC 只抽取当前时刻开始的预测窗。
    controller.setTrajectory(std::move(trajectory), px4controlnode_.get_clock()->now());
    RCLCPP_INFO(
        px4controlnode_.get_logger(),
        "[px4ctrl] Takeoff + barrel-roll activated: %.2f m takeoff, %d turn(s), "
        "%zu samples, %.3f s",
        options.takeoff_height, options.turns, state_count, total_time);
    return true;
}

bool PX4CtrlFSM::load_minimum_snap_cmd_trajectory_()
{
    // 航点采用“进入 CMD 时的位置 + ENU 相对偏移”，因此无论起飞原点和悬停
    // 高度如何，第一点都与实机当前位置连续。这里是唯一需要修改任务航点的位置。
    const Eigen::Vector3d origin = record_state_data.p;
    // 10 个相对航点构成一条小范围、连续转弯并缓慢爬升的测试路线，用于
    // 检查 MPC 对弯绕轨迹的跟踪；这里只控制轨迹尺度，不添加空间硬约束。
    const std::vector<Eigen::Vector3d> relative_points{
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.4, 0.2, 0.1),
        Eigen::Vector3d(0.9, 0.6, 0.2),
        Eigen::Vector3d(1.4, 1.1, 0.3),
        Eigen::Vector3d(1.15, 1.5, 0.4),
        Eigen::Vector3d(0.4, 1.3, 0.5),
        Eigen::Vector3d(-0.4, 0.9, 0.6),
        Eigen::Vector3d(-1.1, 0.4, 0.675),
        Eigen::Vector3d(-1.5, -0.3, 0.75),
        Eigen::Vector3d(-1.1, -1.2, 0.8)};
    std::vector<Eigen::Vector3d> points;
    points.reserve(relative_points.size());
    for (const auto &offset : relative_points) {
        points.push_back(origin + offset);
    }

    MinimumSnapOptions trajectory_options;
    // 保守速度用于降低倾角、推力变化率和角加速度。
    trajectory_options.nominal_speed = 0.65;
    trajectory_options.minimum_segment_time = 0.2;
    // 缓存采样频率取外环名义频率。在线 MPC 的预测步长可以略有不同，
    // GptTrajectoryOptimizer::sample() 会对缓存轨迹插值。
    trajectory_options.sample_dt = 1.0 / std::max(1.0, px4controlnode_.param.ctrl_freq_max);
    trajectory_options.gravity = px4controlnode_.param.gra;
    trajectory_options.yaw = get_yaw_from_quaternion(record_state_data.q);

    MinimumSnapTrajectory generator(points.size(), points, trajectory_options);
    GptTrajectoryResult trajectory = generator.generate();
    if (!trajectory.success) {
        RCLCPP_ERROR(
            px4controlnode_.get_logger(), "[px4ctrl] Minimum-snap failed: %s",
            trajectory.status.c_str());
        return false;
    }

    const std::size_t state_count = trajectory.states.size();
    const double total_time = trajectory.total_time;
    // 右值装载把离散缓存所有权移交给 MPC，不复制数千个轨迹节点。
    controller.setTrajectory(std::move(trajectory), px4controlnode_.get_clock()->now());
    RCLCPP_INFO(
        px4controlnode_.get_logger(),
        "[px4ctrl] Minimum-snap trajectory activated: %zu points, %zu samples, %.3f s",
        points.size(), state_count, total_time);
    return true;
}
#endif

/* 故障模式 */
void* PX4CtrlFSM::FSM_FUNCT(err)(void* this_fsm)// 错误状态
{
    // sun: err 是 FSM 框架级兜底状态，通过共享错误码把异常通知给 process()。
    int *err_var;


    // 通知 调用者 有错误发生
    set_fsm_error_flag((FSM *)this_fsm);

    // 把 错误值 设进 容器中（如果容器存在）
    err_var = (int *)get_err_var((FSM *)this_fsm);
    if(err_var) 
        *err_var = 0xff;
    return NULL;
}

//---------------------------------------------------------------------------------------------------------------
void PX4CtrlFSM::set_init_ref()
{
    // sun: 统一填充所有参考阶次，避免状态切换后沿用上一模式未覆盖的字段。
	Ref_State_t ref;
	ref.p = Eigen::Vector3d::Zero();
	ref.v = Eigen::Vector3d::Zero();
	ref.a = Eigen::Vector3d::Zero();
	ref.j = Eigen::Vector3d::Zero();
    ref.s = Eigen::Vector3d::Zero();
    ref.q = Eigen::Quaterniond::Identity();
	ref.yaw_rate = 0.0;
    ref.throttle = 0.0;
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;
}
void PX4CtrlFSM::set_hover_ref()
{
    // sun: 用进入状态时的速度做 0.3 s 前视补偿，使悬停点位于当前运动趋势前方，减小急停冲击。
	Ref_State_t ref;
	ref.p = record_state_data.p + record_state_data.v * 0.3;
	ref.v = Eigen::Vector3d::Zero();
	ref.a = Eigen::Vector3d::Zero();
    ref.q = yaw_to_quaternion(get_yaw_from_quaternion(record_state_data.q));
    ref.yaw_rate = 0.0;    
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;
}

void PX4CtrlFSM::set_point_hover(const double &x, const double &y, const double &z)
{
    // sun: 用进入状态时的速度做 0.3 s 前视补偿，使悬停点位于当前运动趋势前方，减小急停冲击。
    Eigen::Vector3d point(x,y,z);
	Ref_State_t ref;
	ref.p = point;
	ref.v = Eigen::Vector3d::Zero();
	ref.a = Eigen::Vector3d::Zero();
    ref.q = yaw_to_quaternion(get_yaw_from_quaternion(record_state_data.q));
    ref.yaw_rate = 0.0;    
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;
}

void PX4CtrlFSM::set_land_ref()
{
    // sun: 降落参考保持水平位置和进入时航向，只沿 ENU z 轴以 0.3 m/s 匀速下降。
	Ref_State_t ref;
    auto start_time = record_state_data.time;
    auto now_time = px4controlnode_.get_clock()->now();
    double t = (now_time-start_time).seconds();
    
    // 速度减到0
    double vmax = 0.3;
    ref.p << 0.0, 0.0, -vmax*t;
    ref.p += record_state_data.p + record_state_data.v * 0.3;
    // des.p << 0.0, 0.0, 0.0;
    ref.v << 0.0, 0.0, -vmax;
	ref.a << 0.0, 0.0, 0.0;
	ref.j << 0.0, 0.0, 0.0;
    ref.s << 0.0, 0.0, 0.0;
    ref.q = yaw_to_quaternion(get_yaw_from_quaternion(record_state_data.q));
	ref.yaw_rate = 0.0;
    ref.yaw_accel = 0.0;
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;
}

// 8轨迹（高度不变）
void PX4CtrlFSM::set_2D8_ref()
{
    // sun: 轨迹采用参数化“8”字曲线，并显式给出位置到 snap 的解析导数供前馈控制使用。
    Ref_State_t ref;
    auto start_time = record_state_data.time;
    auto now_time = px4controlnode_.get_clock()->now();
    double t = (now_time-start_time).seconds();
    
    double m = 0.5;
    double vmax = 2.5;
    double v = (2.0/(exp(-m*t)+1)-1)*vmax;
    // sun: Logistic 速度包络让轨迹从零速平滑加速到 vmax，降低刚进入 CMD 时的参考突变。
    double A = 5.0;
    double k = v / A;
    t = t + pi/2/k;
    if (t > pi*2/k)t -= pi*2/k;
	ref.p << A*sin(k*t)*cos(k*t),A*cos(k*t),0.0;
    ref.p += record_state_data.p;
	ref.v << A*k*cos(2*k*t),-A*k*sin(k*t),0.0;
	ref.a << -2*A*k*k*sin(2*k*t),-A*k*k*cos(k*t),0.0;
	ref.j << -4*A*k*k*k*cos(2*k*t),A*k*k*k*sin(k*t),0.0;
    ref.s << 8*A*k*k*k*k*sin(2*k*t),A*k*k*k*k*cos(k*t),0.0;
    ref.q = yaw_to_quaternion(get_yaw_from_quaternion(record_state_data.q));
	ref.yaw_rate = 0.0;
    ref.throttle = std::numeric_limits<double>::quiet_NaN();
    ref.fsm_state = get_curr_state(&fsm_);
    
    ref.flag_valid_p = true;
    ref.flag_valid_v = true;
    ref.flag_valid_a = true;
    ref_ = ref;
}

void PX4CtrlFSM::record_position()
{
    // sun: 记录位置、速度、加速度、姿态和时间，作为后续悬停/降落/轨迹的公共初始条件。
    record_state_data.time = px4controlnode_.get_clock()->now();
	record_state_data.p = pose_data.p;
    record_state_data.v = pose_data.v;
    record_state_data.a = sens_data.a;
    record_state_data.q = att_data.q;
    record_state_data.yaw = get_yaw_from_quaternion(att_data.q);
}

void PX4CtrlFSM::set_manual_ref(const double &dt, bool reset_yaw_des)
{
    // sun: 遥控滚转/俯仰映射到 ±90°，偏航通道映射到 ±90°/s，油门映射到 [0,1]。
    rclcpp::Time time_now = px4controlnode_.get_clock()->now();

    double rc_roll = rc_data.roll*deg2rad(90) * (px4controlnode_.param.rc_reverse.roll ? 1 : -1);
    double rc_pitch = rc_data.pitch*deg2rad(90) * (px4controlnode_.param.rc_reverse.pitch ? 1 : -1);
    double manual_throttle = (rc_data.throttle * (px4controlnode_.param.rc_reverse.throttle ? 1 : -1) + 1)/2.0 ; // [0,1]
    // std::cout<<"manual_throttle:  "<<manual_throttle<<std::endl;
    double manual_yawrate = rc_data.yaw * deg2rad(90) * (px4controlnode_.param.rc_reverse.yaw ? 1 : -1);
    
    // sun: 大油门时把航向参考重新贴合当前记录航向，避免积分航向误差造成突转。
    if (manual_throttle > 0.9) {
		reset_yaw_des = true;
	}
    // 确保绝对航向误差不积累
    if (reset_yaw_des){
        manual_yaw_ = record_state_data.yaw;
    }else{
        manual_yaw_ = normalize_angle(manual_yaw_ + dt*manual_yawrate);
    }

    Eigen::Vector3d axis(rc_roll, rc_pitch, 0.0); // 旋转轴（单位向量）
    // sun: 将滚转/俯仰摇杆组成倾转轴角，再左乘航向四元数，实现倾转与偏航解耦。
    double angle = axis.norm(); // 旋转角度（弧度）
    axis.normalize(); // 确保轴是单位向量
    Eigen::AngleAxisd angleAxis(angle, axis);
    Eigen::Quaterniond q_rp(angleAxis);
    Eigen::Quaterniond q_yaw(cos(manual_yaw_ / 2.f), 0.f, 0.f, sin(manual_yaw_ / 2.f));
    
    Eigen::Quaterniond manual_q = q_yaw*q_rp;

    Ref_State_t ref;
	ref.p << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
	ref.v << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
	ref.a << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
    ref.j << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
    ref.s << std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN();
	ref.q = manual_q;
	ref.yaw_rate = manual_yawrate;
    ref.throttle = manual_throttle;
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;

}


//遥控器位置控制逻辑实现
//有摇杆量的时候就失效位置控制而转向速度控制，然后在不控制的时候立马采集此时的传感器数据作为位置控制的期望值
void PX4CtrlFSM::set_manual_postion_ref(const double &dt, bool reset_yaw_des)
{
    // sun: 遥控滚转/俯仰映射到 ±90°，偏航通道映射到 ±90°/s，油门映射到 [0,1]。
    rclcpp::Time time_now = px4controlnode_.get_clock()->now();
    Ref_State_t ref;

    double rc_roll = rc_data.roll;//右到左-1到1 左边
    double rc_pitch = rc_data.pitch;// 上到下-1到1 右边边
    double manual_throttle = rc_data.throttle; // 上到下-1到1 左边
    // std::cout<<"manual_throttle:  "<<manual_throttle<<std::endl;
    double manual_yawrate = rc_data.yaw * deg2rad(90) * (px4controlnode_.param.rc_reverse.yaw ? 1 : -1);//右到左-1到1 右边边


    // std::cout<<"rc_data.pitch:   "<<rc_data.pitch<<std::endl;
    // std::cout<<"rc_data.roll:   "<<rc_data.roll<<std::endl;
    // std::cout<<"rc_data.throttle:   "<<rc_data.throttle<<std::endl;
    // std::cout<<"rc_data.yaw:   "<<rc_data.yaw<<std::endl;
    // std::cout<<"rc_pitch:   "<<rc_pitch<<std::endl;
    // std::cout<<"rc_roll:   "<<rc_roll<<std::endl;
    // std::cout<<"manual_throttle:   "<<manual_throttle<<std::endl;
    // std::cout<<"manual_yawrate:   "<<manual_yawrate<<std::endl;

    Eigen::Vector3d point(0,0,manual_throttle);

    if((manual_throttle>0.1)||(manual_throttle<-0.1)){
        ref.v = point;
        ref.flag_valid_p = false;
    }
    else{
        ref.p = pose_data.p;
        ref.v = Eigen::Vector3d::Zero();
        ref.flag_valid_p = true;
    }
    
      
	
	ref.a = Eigen::Vector3d::Zero();
    ref.q = yaw_to_quaternion(get_yaw_from_quaternion(record_state_data.q));
    // ref.yaw_rate = manual_yawrate;    
    ref.fsm_state = get_curr_state(&fsm_);
	ref_ = ref;


    

}


//------------------------------------------------------------------------------------------------------------
bool PX4CtrlFSM::rc_is_received(const rclcpp::Time &now_time)
{
    // sun: 遥控允许较宽的 1.5 s 超时，位置/姿态等闭环反馈使用更严格的 0.5 s。
	return (now_time - rc_data.rcv_stamp).seconds() < 1.5; //param.msg_timeout.rc = 0.5s

}

/* 判断初始状态切换摇杆是否在最下面 */
bool PX4CtrlFSM::rc_is_downinit()
{
    return (rc_data.aux2 == GEARS::DOWN && rc_data.aux1 == GEARS::DOWN);
}

/* 判断即停开关 */
bool PX4CtrlFSM::rc_is_kill()
{
    return rc_data.aux4 == GEARS::UP;
}

/*  判断解锁开关 */
bool PX4CtrlFSM::rc_is_armed()
{
    static GEARS last_aux3 = rc_data.aux3;
    if (rc_data.aux3 == GEARS::UP)
    {
        if (last_aux3 == GEARS::DOWN || last_aux3 == GEARS::MID){
            last_aux3 = rc_data.aux3;
            return true;
        }
        else{
            return false;
        }
    }else{
        last_aux3 = rc_data.aux3;
        return false;
    }
}
/* 判断解锁 */
bool PX4CtrlFSM::vs_is_armed()
{
    return sta_data.arming_state == 2;
}

bool PX4CtrlFSM::pose_is_valid(const LocalPose_Data_t &pose)
{   
	return (pose.xy_valid && pose.z_valid && pose.v_xy_valid && pose.v_z_valid);
}

bool PX4CtrlFSM::pose_is_received(const rclcpp::Time &now_time)
{   
	return (now_time - pose_data.rcv_stamp).seconds() < 0.5;
}

bool PX4CtrlFSM::att_is_received(const rclcpp::Time &now_time)
{
	return (now_time - att_data.rcv_stamp).seconds() < 0.5;
}

bool PX4CtrlFSM::sens_is_received(const rclcpp::Time &now_time)
{
	return (now_time - sens_data.rcv_stamp).seconds() < 0.5;
}

bool PX4CtrlFSM::bat_is_received(const rclcpp::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).seconds() < 0.5;
}

bool PX4CtrlFSM::recv_new_pose()
{
    // sun: 读取后立即清零，提供“每条位置消息最多触发一次处理”的消费语义。
	if (pose_data.recv_new_msg)
	{
		pose_data.recv_new_msg = false;
		return true;
	}
	return false;
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void PX4CtrlFSM::publish_vehicle_command_(uint16_t command, double param1, double param2)
{
	// sun: PX4 时间戳单位为微秒；source/target 均设为系统 1、组件 1，并标记命令来自外部。
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->px4controlnode_.get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher->publish(msg);
}

bool PX4CtrlFSM::switch_to_offboard_mode_(){
#ifdef SIMULATION
    // sun: 仿真器不实现 PX4 模式服务，直接视为切换成功。
    return true;
#endif
    if(service_done){
        if (service_result_==0){
            return true;			
        }
        else{
            RCLCPP_ERROR(this->px4controlnode_.get_logger(), "\033[31mFailed to enter offboard mode, retrying\033[0m");
            service_done = false;
            return false;
        }
    }
    else{
        request_vehicle_command_(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        return false;
    }
	
}
bool PX4CtrlFSM::switch_to_manual_mode_(){
#ifdef SIMULATION
    return true;
#endif
    if(service_done){
        if (service_result_==0){
            return true;			
        }
        else{
            RCLCPP_ERROR(this->px4controlnode_.get_logger(), "\033[31mFailed to enter manual mode, retrying\033[0m");
            service_done = false;
            return false;
        }
    }
    else{
        request_vehicle_command_(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
        return false;
    }
	
}
bool PX4CtrlFSM::arm()
{
    if(service_done){
        if (service_result_==0){
            // RCLCPP_INFO(this->px4controlnode_.get_logger(), "Entered manual mode");
            return true;			
        }
        else{
            RCLCPP_ERROR(this->px4controlnode_.get_logger(), "\033[31mFailed to enter manual mode, retrying\033[0m");
            service_done = false;
            return false;
        }
    }
    else{
        request_vehicle_command_(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        return false;
    }

	RCLCPP_INFO(this->px4controlnode_.get_logger(), "\033[33m[PX4CTRL] Arm command send\033[0m");
}
/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void PX4CtrlFSM::request_vehicle_command_(uint16_t command, double param1, double param2)
{
	// sun: 异步服务避免控制线程阻塞；完成标志由 response_callback_ 统一更新。
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->px4controlnode_.get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	vehicle_command_client->async_send_request(request, 
                                               std::bind(&PX4CtrlFSM::response_callback_, 
                                               this,
                                               std::placeholders::_1));
	// RCLCPP_INFO(this->px4controlnode_.get_logger(), "Command send");
}

void PX4CtrlFSM::response_callback_(
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) 
{
    // sun: 回调最多等待一秒获取共享 future，并保存 PX4 命令结果供下一次状态机周期判定。
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
	  auto reply = future.get()->reply;
	  service_result_ = reply.result;
      switch (service_result_)
		{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			// RCLCPP_INFO(this->px4controlnode_.get_logger(), "command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command cancelled");
			break;
		default:
			RCLCPP_WARN(this->px4controlnode_.get_logger(), "command reply unknown");
			break;
		}
      service_done = true;
    } else {
      RCLCPP_INFO(this->px4controlnode_.get_logger(), "Service In-Progress...");
    }
  }
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void PX4CtrlFSM::publish_offboard_control_mode_()
{
	// sun: 本工程自行完成控制和分配，因此声明 direct_actuator，其余 PX4 控制层全部关闭。
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = true;
	msg.timestamp = this->px4controlnode_.get_clock()->now().nanoseconds() / 1000; // 微秒
	offboard_control_mode_publisher->publish(msg);
}

void PX4CtrlFSM::publish_rates_thrust_setpoint()
{
    // sun: 自定义消息保留 ENU/FLU 角速度符号，底层节点负责最终转换和电机混控。
    ratectrl_msgs::msg::RatesThrustSetpoint msg{};
    msg.bodyrates[0] = control_sp_.bodyrates[0];
    msg.bodyrates[1] = control_sp_.bodyrates[1];
    msg.bodyrates[2] = control_sp_.bodyrates[2];
    if(std::isnan(msg.bodyrates[0]))
    {
        std::cout << "msg.bodyrates contains NAN!" << std::endl;
    }
    msg.thrust = control_sp_.thrust;
    msg.rate_dot_ref[0] = control_sp_.rate_dot_ref[0];
    msg.rate_dot_ref[1] = control_sp_.rate_dot_ref[1];
    msg.rate_dot_ref[2] = control_sp_.rate_dot_ref[2];
	msg.timestamp = this->px4controlnode_.get_clock()->now().nanoseconds() / 1000;
	rates_thrust_setpoint_publisher->publish(msg);
}
