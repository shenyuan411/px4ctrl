#include <ros/ros.h>
#include "PX4Ctrlfsm.h"
#include <signal.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh_);

    // Controller controller(param);
    LinearControl controller(param);
    PX4CtrlFSM fsm(param, controller);

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    // ros::Subscriber bat_sub =
    //     nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
    //                                             100,
    //                                             boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
    //                                             ros::VoidConstPtr(),
    //                                             ros::TransportHints().tcpNoDelay());

	// 订阅起飞命令话题，takeoff_land
	// 回调函数	->fsm.takeoff_land_data.triggered = 1，接收起飞命令
    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());

	// /mavros/setpoint_raw/attitude 对应有两种方式控制：1.姿态+油门；2.机体角速度+油门
    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    //首先飞机要处在定点模式下，此时把6通道拨杆从控制指令拒绝状态拨到使能状态，此刻px4ctrl会发一个`/traj_start_trigger`出来，
    // 当下还尚未切换到指令控制模式，px4ctrl会等待外部指令，一旦接收到指令，模式就会切换，屏幕也会打印绿色的"[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)"
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    ros::Duration(0.5).sleep();

    dynamic_reconfigure::Server<px4ctrl::fake_rcConfig> server;
    dynamic_reconfigure::Server<px4ctrl::fake_rcConfig>::CallbackType f;

	//判断从参数服务器读过来的参数设置是否需要遥控：1不需要，0需要
    if (param.takeoff_land.no_RC)
    {
        f = boost::bind(&Dynamic_Data_t::feed, &fsm.dy_data ,_1); //绑定回调函数
        server.setCallback(f); //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    //进入死循环，检查px4的连接，连接正常跳出循环
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

	//主循环，以固定帧率进入fsm.process()进程
    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        ROS_INFO_ONCE("PX4CTRL] Is OK!");
        r.sleep();
        ros::spinOnce();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}