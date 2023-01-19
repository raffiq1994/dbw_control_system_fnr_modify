/*******************************************************
 *
 *    _    ___  ___   ___  ___ __   __ ___  ___  ___
 *   /_\  |_ _||   \ | _ \|_ _|\ \ / /| __|| _ \/ __|
 *  / _ \  | | | |) ||   / | |  \ V / | _| |   /\__ \
 * /_/ \_\|___||___/ |_|_\|___|  \_/  |___||_|_\|___/
 *
 *
 * Copyright (C) 2022 AIDE @ AIDRIVERS Ltd - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * author = 'Robert Watkins'
 * email  = 'robert@aidrivers.ai'
 * Version = 0.9
 *******************************************************/



// ROS standard msg Headers
#include "dbw_control_system/dbw_control_system.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <iostream>  
#include <string>
#include <sstream>
#include <math.h>
#include <stdio.h>

// TF Headers


// aide_msg Headers
#include <aide_apm_msgs/ApmStatus.h>


  //dbw_control_system_class(ros::NodeHandle nh, ros::NodeHandle private_nh)
  
  airs::dbw_control_system_class::dbw_control_system_class(ros::NodeHandle nh, ros::NodeHandle private_nh)
  {
    /*
      dbw_control_system_node/at1/PropB_ATCC2_01: /airs/plc/at1/PropB_ATCC2_01
      dbw_control_system_node/at1/PropB_ATCC2_02: /airs/plc/at1/PropB_ATCC2_02
      dbw_control_system_node/at1/PropB_ATCC2_03: /airs/plc/at1/PropB_ATCC2_03
      dbw_control_system_node/at1/PropB_ATCC2_05: /airs/plc/at1/PropB_ATCC2_05
      dbw_control_system_node/at1/PropB_ATCC2_06: /airs/plc/at1/PropB_ATCC2_06
      dbw_control_system_node/at1/PropB_YTCC2_01: /airs/plc/at1/PropB_YTCC2_01
      dbw_control_system_node/at1/PropB_YTCC2_02: /airs/plc/at1/PropB_YTCC2_02
      dbw_control_system_node/at1/PropB_YTCC2_03: /airs/plc/at1/PropB_YTCC2_04
      */


    // Topics for subscribe

    private_nh.param<std::string>("/dbw_control_system_node/aidc/crtl_cmd",
                                  param_aidc_crtl_intopic_, "/crtl_cmd");
    private_nh.param<std::string>("/dbw_control_system_node/aidc/gear_req",
                                  param_aidc_transmission_intopic_, "/gear_req");

    private_nh.param<std::string>("/dbw_control_system_node/aios/navigation_mode",
                                  param_aios_auto_intopic_, "/navigation_mode");

    private_nh.param<std::string>("/dbw_control_system_node/aidc/e_stop_req",
                                  param_aidc_e_stop_intopic_, "/e_stop_req");
    private_nh.param<std::string>("/dbw_control_system_node/aidc/indicator",
                                  param_aidc_indicator_intopic_, "/indicator");
    private_nh.param<std::string>("/dbw_control_system_node/aios/error_reset",
                                  param_aios_error_reset_intopic_, "/error_reset");
    private_nh.param<std::string>("/dbw_control_system_node/aidc/aidc_dashboard",
                                  param_aidc_dashboard_cmd_intopic_, "/aidc_dashboard");
    private_nh.param<std::string>("/dbw_control_system_node/aios/aios_dashboard",
                                  param_aios_dashboard_cmd_intopic_, "/aios_dashboard");
    private_nh.param<std::string>("/dbw_control_system_node/aidc/apm_status",
                                  param_aidc_apm_status_intopic_, "/aidc_apm_status");


    private_nh.param<std::string>("/dbw_control_system_node/airs_dbw_fss",
                                  param_airs_fss_intopic_, "/airs_dbw_fss");
    private_nh.param<std::string>("/dbw_control_system_node/airs_dbw_fs_vcu",
                                  param_airs_fs_vcu_intopic_, "/airs_dbw_fs_vcu");

    // ### auto tug subscribers ####
    private_nh.param<std::string>("/dbw_control_system_node/at1/CCVS1",
                                  param_dbw_user_CCVS1_intopic_, "/airsCCVS1");
    private_nh.param<std::string>("/dbw_control_system_node/at1/ETC2",
                                  param_dbw_user_gear_fb_intopic_, "/airsETC2");
    private_nh.param<std::string>("/dbw_control_system_node/at1/PropB_ATCC2_01",
                                  param_airs_at_dashboard_fb_intopic_);
    private_nh.param<std::string>("/dbw_control_system_node/at1/PropB_ATCC2_02",
                                  param_airs_speed_fb_intopic_, "/airs3");
    private_nh.param<std::string>("/dbw_control_system_node/at1/PropB_ATCC2_03",
                                  param_airs_fluid_levels_fb_intopic_, "/airs4");
    private_nh.param<std::string>("/dbw_control_system_node/at1/PropB_ATCC2_05",
                                  param_airs_auto_manual_fb_intopic_, "/airs5");
    private_nh.param<std::string>("/dbw_control_system_node/at1/PropB_ATCC2_06",
                                  param_airs_brake_fb_intopic_, "/airs6");

    private_nh.param<std::string>("/dbw_control_system_node/at1/PropB_YTCC2_01",
                                  param_airs_accpedal_Lights_fb_intopic_, "/airsy1");
    private_nh.param<std::string>("/dbw_control_system_node/at1/PropB_YTCC2_02",
                                  param_airs_fith_wheel_fb_intopic_, "/airsy2");
    private_nh.param<std::string>("/dbw_control_system_node/at1/PropB_YTCC2_03",
                                  param_airs_engine_fb_intopic_, "/airsy3");



    // ### auto tug steering subscribers ####
    private_nh.param<std::string>("/dbw_control_system_node/at2/dbw_user_steering_fb",
                              param_dbw_user_steering_fb_intopic_, "/airs11");
    private_nh.param<std::string>("/dbw_control_system_node/at2/dbw_user_steering_fb_s",
                              param_dbw_user_steering_fb_s_intopic_, "/airs22");
    private_nh.param<std::string>("/dbw_control_system_node/at2/dbw_user_steering_status",
                              param_dbw_user_steering_status_intopic_, "/airs33");

    private_nh.param<std::string>("/dbw_control_system_node/at2/dbw_user_steering_status_s",
                              param_dbw_user_steering_status_s_intopic_, "/airs44");
    private_nh.param<std::string>("/dbw_control_system_node/at2/dbw_user_steering_sensor_fb",
                              param_dbw_user_steering_sensor_fb_intopic_, "/airs55");
    private_nh.param<std::string>("/dbw_control_system_node/at2/dbw_user_steering_sensor_fb_r",
                              param_dbw_user_steering_sensor_fb_r_intopic_, "/airs66");

    aidc_ctrl_cmd_sub_ =
        nh.subscribe(param_aidc_crtl_intopic_, 1,
                     &dbw_control_system_class::aidcCrtlCmdCallback, this,
                     ros::TransportHints().tcpNoDelay(true));

    aidc_gear_req_sub_ = nh.subscribe(
        param_aidc_transmission_intopic_, 1, &dbw_control_system_class::aidcGearRequestCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    aios_auto_req_sub_ = nh.subscribe(
        param_aios_auto_intopic_, 1, &dbw_control_system_class::aiosAutoRequestCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    aidc_E_brake_req_sub_ = nh.subscribe(
        param_aidc_e_stop_intopic_, 1, &dbw_control_system_class::aidcEStopRequestCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    aidc_indicator_sub_ = nh.subscribe(
        param_aidc_indicator_intopic_, 1, &dbw_control_system_class::aidcIndicatorRequestCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    aios_error_reset_sub_ = nh.subscribe(
        param_aios_error_reset_intopic_, 1, &dbw_control_system_class::aidcEStopResetCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    aidc_dashboard_sub_ = nh.subscribe(
        param_aidc_dashboard_cmd_intopic_, 1, &dbw_control_system_class::aidcDashboardCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    aios_dashboard_sub_ = nh.subscribe(
        param_aios_dashboard_cmd_intopic_, 1, &dbw_control_system_class::aiosDashboardCallback,
        this, ros::TransportHints().tcpNoDelay(true));      
    aidc_apm_status_sub_ = nh.subscribe(
        param_aidc_apm_status_intopic_, 1, &dbw_control_system_class::aidcApmStatusCallback,
        this, ros::TransportHints().tcpNoDelay(true));


    // ##################### AutoTug AT1 signals subscribers #####################
    airs_VehSpeed_fb_sub_ = nh.subscribe(
        param_dbw_user_CCVS1_intopic_, 1, &dbw_control_system_class::airsCCVS1FBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    airs_transmission_fb_sub_ = nh.subscribe(
        param_dbw_user_gear_fb_intopic_, 1, &dbw_control_system_class::airstransmissionFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    airs_dashbord_fb_sub_ = nh.subscribe(
        param_airs_at_dashboard_fb_intopic_, 1, &dbw_control_system_class::airsDashboardFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    airs_speed_fb_sub_ = nh.subscribe(
        param_airs_speed_fb_intopic_, 1, &dbw_control_system_class::airsSpeedFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    airs_fluid_levels_fb_sub_ = nh.subscribe(
        param_airs_fluid_levels_fb_intopic_, 1, &dbw_control_system_class::airsFluidlevelsFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

    airs_dbw_control_fb_sub_ = nh.subscribe(
        param_airs_auto_manual_fb_intopic_, 1, &dbw_control_system_class::airsAutoManualFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

   airs_brake_fb_sub_ = nh.subscribe(
        param_airs_brake_fb_intopic_, 1, &dbw_control_system_class::airsbrakeFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

   airs_acc_pedal_Lights_sub_ = nh.subscribe(
        param_airs_accpedal_Lights_fb_intopic_, 1, &dbw_control_system_class::airsAccpedalLightsFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

   airs_fith_wheel_sub_ = nh.subscribe(
        param_airs_fith_wheel_fb_intopic_, 1, &dbw_control_system_class::airsFithWheelFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

   airs_engine_fb_sub_ = nh.subscribe(
        param_airs_engine_fb_intopic_, 1, &dbw_control_system_class::airsEngineFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

   airs_steering_status_sub_ = nh.subscribe(
        param_dbw_user_steering_status_intopic_, 1, &dbw_control_system_class::airsSteeringStatusFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));
   airs_steering_fb_sub_ = nh.subscribe(
        param_dbw_user_steering_fb_intopic_, 1, &dbw_control_system_class::airsSteeringFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));

   airs_fss_fb_sub_ = nh.subscribe(
        param_airs_fss_intopic_, 1, &dbw_control_system_class::airsFSSFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));
   airs_fs_vcu_fb_sub_ = nh.subscribe(
        param_airs_fs_vcu_intopic_, 1, &dbw_control_system_class::airsFSVCUFBCallback,
        this, ros::TransportHints().tcpNoDelay(true));



    // ##################### Steering Subscribers: #####################

    /* Steering subscribers:
    dbw_control_system_node/at1/dbw_user_dashboard: /airs/plc/at1/PropB_DbwUser_01
    dbw_control_system_node/at1/dbw_user_E_brake: /airs/plc/at1/PropB_DbwUser_02
    dbw_control_system_node/at1/dbw_user_fith_wheel: /airs/plc/at1/PropB_DbwUser_03
    dbw_control_system_node/at1/dbw_user_Auto: /airs/plc/at1/PropB_DbwUser_04
    dbw_control_system_node/at1/dbw_user_Engine: /airs/plc/at1/PropB_DbwUser_05
    dbw_control_system_node/at1/dbw_user_brake: /airs/plc/at1/PropB_DbwUser_07
        */

    ///#### Publishers

    private_nh.param<std::string>("/dbw_control_system_node/airs_dbw_status",
                              param_airs_dbw_status_intopic_,"/airs_dbw_status"); //00
    private_nh.param<std::string>("/dbw_control_system_node/airs_dbw_dashboard",
                              param_airs_dashboard_fb_intopic_,"/airs_dbw_status"); //00
    private_nh.param<std::string>("/dbw_control_system_node/at1/dbw_user_dashboard",
                              param_dbw_user_dashboard_intopic_,"/airsdbw01"); //01
    private_nh.param<std::string>("/dbw_control_system_node/at1/dbw_user_E_brake",
                              param_dbw_user_E_brake_intopic_,"/airsdbw02"); //02
    private_nh.param<std::string>("/dbw_control_system_node/at1/dbw_user_fith_wheel",
                              param_dbw_user_fith_wheel_intopic_,"/airsdbw03"); //03
    private_nh.param<std::string>("/dbw_control_system_node/at1/dbw_user_Auto",
                              param_dbw_user_Auto_intopic_,"/airsdbw04"); //04
    private_nh.param<std::string>("/dbw_control_system_node/at1/dbw_user_Engine",
                              param_dbw_user_Engine_intopic_,"/airsdbw05"); //05
    private_nh.param<std::string>("/dbw_control_system_node/at1/dbw_user_brake",
                              param_dbw_user_brake_intopic_,"/airsdbw07"); //07
    private_nh.param<std::string>("/dbw_control_system_node/at2/dbw_user_park_brake",
                              param_dbw_user_park_brake_intopic_,"/dbw_user_park_brake"); //at2 01

    private_nh.param<std::string>("/dbw_control_system_node/at2/dbw_user_steering_req",
                              param_dbw_user_steering_req_intopic_,"/airsdbwsr");
    private_nh.param<std::string>("/dbw_control_system_node/at2/dbw_user_steering_req_r",
                              param_dbw_user_steering_req_r_intopic_,"/airsdbwsrr");

    private_nh.param<std::string>("/dbw_control_system_node/airs_dbw_speed_control",
                              param_airs_dbw_speed_control_intopic_,"/airs_dbw_speed_control");

    private_nh.param<std::string>("/dbw_control_system_node/airs_dbw_steering_control",
                              param_dbw_user_steering_control_fb_intopic_,"/airs_dbw_steering_control");

    private_nh.param<std::string>("/dbw_control_system_node/airs_vehicle_state",
                              param_airs_vehicle_state_intopic_,"/airs_vehicle_state");
    
    airs_dbw_status_pub = nh.advertise<airs_msgs::airs_dbw_status>(
      param_airs_dbw_status_intopic_, 1);

    airs_dashboard_fb_pub = nh.advertise<airs_msgs::airs_dashboard_fb>(
      param_airs_dashboard_fb_intopic_, 1);

    airs_speed_control_pub = nh.advertise<airs_msgs::airs_speed_controller_fb>(
      param_airs_dbw_speed_control_intopic_, 1);

    dbw_user_dashboard_pub = nh.advertise<airs_msgs::PropB_DbwUser_01>(
      param_dbw_user_dashboard_intopic_, 1);

    dbw_user_E_brake_pub = nh.advertise<airs_msgs::PropB_DbwUser_02>(
      param_dbw_user_E_brake_intopic_, 1);

    dbw_user_fith_wheel_pub = nh.advertise<airs_msgs::PropB_DbwUser_03>(
      param_dbw_user_fith_wheel_intopic_, 1);

    dbw_user_Auto_pub = nh.advertise<airs_msgs::PropB_DbwUser_04>(
      param_dbw_user_Auto_intopic_, 1);

    dbw_user_Engine_pub = nh.advertise<airs_msgs::PropB_DbwUser_05>(
      param_dbw_user_Engine_intopic_, 1);

    dbw_user_brake_pub = nh.advertise<airs_msgs::PropB_DbwUser_07>(
      param_dbw_user_brake_intopic_, 1);


    dbw_user_P_brake_pub = nh.advertise<airs_msgs::AT2_PropB_DbwUser_01>(
      param_dbw_user_park_brake_intopic_, 1);

    // Steering advertise
    dbw_user_steering_pub = nh.advertise<airs_msgs::AUX_JOY_P>(
      param_dbw_user_steering_req_intopic_, 1);

    dbw_user_steering_r_pub = nh.advertise<airs_msgs::AUX_JOY_R>(
      param_dbw_user_steering_req_r_intopic_, 1);

    airs_steering_control_pub = nh.advertise<airs_msgs::airs_steering_controller_fb>(
      param_dbw_user_steering_control_fb_intopic_, 1);

    airs_vehicle_state_pub = nh.advertise<airs_msgs::reg37>(
          param_airs_vehicle_state_intopic_, 1);

    // parameters from config file for engine pid
    private_nh.param<double>("/dbw_control_system_node/airs_speed_control_param/Engine_Gain_Param",
                             Engine_Gain_Param_, 1);
    private_nh.param<double>("/dbw_control_system_node/airs_speed_control_param/Engine_Integral_Param",
                             Engine_Integral_Param_, 0.5);
    private_nh.param<double>("/dbw_control_system_node/airs_speed_control_param/Engine_Integral_Rate_Param",
                             Engine_Integral_Rate_Param_, 0.5);
    private_nh.param<double>("/dbw_control_system_node/airs_speed_control_param/Engine_Derivative_Param",
                             Engine_Derivative_Param_, 0.5);

    // parameters from config file for brake pid
    private_nh.param<double>("/dbw_control_system_node/airs_speed_control_param/Brake_Gain_Param",
                             Brake_Gain_Param_, 0.5);
    private_nh.param<double>("/dbw_control_system_node/airs_speed_control_param/Brake_Integral_Param",
                             Brake_Integral_Param_, 0.5);
    private_nh.param<double>("/dbw_control_system_node/airs_speed_control_param/Brake_Integral_Rate_Param",
                             Brake_Integral_Rate_Param_, 0.5);
    private_nh.param<double>("/dbw_control_system_node/airs_speed_control_param/Brake_Derivative_Param",
                             Brake_Derivative_Param_, 0.5);
    private_nh.param<double>("/dbw_control_system_node/airs_speed_control_param/Vehicle_Dynamic_Param",
                             Vehicle_Dynamic_Param_, 0.5);

    
      initialise();
  }

  void airs::dbw_control_system_class::initialise(){
    //tl_wrapper_status_msg_ptr->header = std_msgs::Header();
    ROS_INFO("initialise");
    aidc_speed_cmd = 0.0;
    aidc_steering_cmd = 0.0;
    aios_auto_request = 0;
    aidc_gear_request = 0;
    aios_auto_request_prev = 0;

    checksum_counter_max =16;
    //checksum_counter_arr[9];
    for (int k=1;k<=11;k++){
      checksum_counter_arr[k]=0;
    }

    // emergency stop
    aidc_e_stop_request = false;
    aidc_e_stop_feedback = 0;
    airs_dbw_Emergency_Trigger = false;
    aidc_e_stop_latch = 0;
    aidc_e_stop_reset = 0;
    aios_error_reset_request = false;
    airs_e_stop_request_time_check = ros::Time::now();
    airs_e_stop_function_time_check = ros::Time::now();
    airs_e_stop_request_time_diff = ros::Duration(2);

    //aidc crtl_cmd timers
    airs_aidc_motion_function_time_check = ros::Time::now();
    airs_aidc_motion_request_time_check = ros::Time::now();
    airs_aidc_motion_request_time_diff = ros::Duration(2);
    airs_aidc_gear_request_time_check = ros::Time::now();
    airs_aidc_gear_request_time_diff = ros::Duration(2);
    airs_aidc_indicator_request_time_check = ros::Time::now();
    airs_aidc_indicator_request_time_diff = ros::Duration(2);

    //gear change
    airs_aidc_gear_check = ros::Time::now();
    airs_aidc_gear_timer = ros::Time::now();
    airs_aidc_gear_duration = ros::Duration(2);
    airs_aidc_dashboard_cmd_.GearInt = 0;

    airs_park_brake_release_check = ros::Time::now();
    airs_park_brake_release_real = ros::Time::now();
    airs_park_brake_release_duration = ros::Duration(2);
    airs_park_brake_release_diff= ros::Duration(1);

    //e_stop


    airs_dbw_status_.header = std_msgs::Header();
    airs_dbw_status_.airs_manual_override = false;
    airs_dbw_status_.airs_auto_ready = false;
    airs_dbw_status_.airs_auto_state_indx = 0;
    airs_dbw_status_.aios_auto_request_Latch=0;
    airs_dbw_status_.airs_at_SwitchToManualCause =0;
    airs_dbw_status_.airs_at_DBWControlState = 0;
    airs_dbw_status_.airs_at_DbwUserMode = 0;
    airs_dbw_status_.airs_apm_in_auto = false;
    airs_dbw_status_.airs_e_stop_latch = false;
    airs_dbw_status_.airs_error_reset_state = " All Clear";
    airs_dbw_status_.airs_version = 0.0;
    airs_dbw_status_.airs_version_release_date = "00/00/0000";

    // speed controller
    airs_speed_control_param_.Engine_Gain_Param = Engine_Gain_Param_;
    airs_speed_control_param_.Engine_Integral_Param = Engine_Integral_Param_;
    airs_speed_control_param_.Engine_Integral_Rate_Param = Engine_Integral_Rate_Param_;
    airs_speed_control_param_.Engine_Derivative_Param = Engine_Derivative_Param_;
    airs_speed_control_param_.Brake_Gain_Param = Brake_Gain_Param_;
    airs_speed_control_param_.Brake_Integral_Param = Brake_Integral_Param_;
    airs_speed_control_param_.Brake_Integral_Rate_Param = Brake_Integral_Rate_Param_;
    airs_speed_control_param_.Brake_Derivative_Param = Brake_Derivative_Param_;
    airs_speed_control_param_.Vehicle_Dynamic_Param = Vehicle_Dynamic_Param_;
    airs_previous_velocity_time = ros::Time::now();
    airs_previous_velocity_time_qa = ros::Time::now();
    airs_previous_velocity_time_qavi = ros::Time::now();
    Engine_Integral_max = 180;
    Brake_Integral_max = 100;
    aidc_steering_cmd_prev = 0.0;

    /*
      dbw_control_system_node/airs_speed_control_param/Engine_Gain_Param: -0.05

      double Engine_Gain_Param_;
        double Engine_Integral_Param_;
        double Engine_Integral_Rate_Param_;
        double Engine_Derivative_Param_;
        double Brake_Gain_Param_;
        double Brake_Integral_Param_;
        double Brake_Integral_Rate_Param_;
        double Brake_Derivative_Param_;
        double Vehicle_Dynamic_Param_;

    */

    airs_previous_velocity_time_diff = ros::Duration(0.1);
    airs_previous_time_diff_twoh_sec = ros::Duration(0.2);
    airs_previous_time_diff_half_sec = ros::Duration(0.5);
    airs_previous_time_diff_one_sec = ros::Duration(1.0);

    airs_engine_reset_ = false;
    airs_brake_reset_ = false;
    airs_gear_brake_pressure = false;
    param_Engine_de_acc_multiplier = 1.0;
    airs_engine_ref_cnt = 5;
    v_i =0;
    Acc_prev =0;
    for (int i=0; i<5; i++)
    {
      V_prev_array[i]=0;
    }
    Brake_Live_Calib_timeout = false;
    Brake_Live_Calib_trigger = false;
    Brake_live_Calib_Value = 1;
    param_Brake_de_acc_multiplier = 1.0;
    Brake_low_speed = false;

    
    airs_AT1_CCVS1_.WheelBasedVehicleSpeed = 0;
    airs_AT1_CCVS1_.ParkingBrakeSwitch = 0;
    airs_AT1_CCVS1_.ParkBrakeReleaseInhibitRq = 0;
    airs_steering_control_.airs_steering_Operation_state = 0;

    // Functional safety systems
    airs_dbw_status_.airs_safe_stop = false;
    airs_dbw_status_.FSS_Ready = true;
    Functional_Safety_System_Fail = false;
    FSS_Front_OBJ_Detected = false;
    FSS_Left_OBJ_Detected = false;
    FSS_Right_OBJ_Detected = false;
    FSS_Working_Lane_OBJ_Detected = false;
    airs_aios_dashboard_cmd_.FSSSpeedThreshold = 8;
    airs_fss_fb_.Physical_Safety_Estop_Triggered = false;

    airs_speed_control_.header = std_msgs::Header();
    airs_speed_control_.header.frame_id = "speed_control";
    airs_speed_control_.Acc_actual = 0;
    airs_speed_control_.Acc_req = 0;
    airs_speed_control_.Acc_error = 0;
    airs_speed_control_.V_error = 0;
    airs_speed_control_.V_req = 0;
    airs_speed_control_.V_actual = 0;
    airs_speed_control_.delta_t = 1;
    airs_speed_control_.delta_t_act =0.5;


    airs_engine_ref_prev = 0;
    airs_speed_control_.Engine_ref_fx = 0;
    airs_speed_control_.Engine_Gain = 0;
    airs_speed_control_.Engine_Integral = 0;
    airs_speed_control_.Engine_Integral_Rate = 0.1;
    airs_speed_control_.Engine_Derivative = 0;

    airs_speed_control_.Brake_Front_ref_fx = 0;    
    airs_speed_control_.Brake_Rear_ref_fx = 0;
    airs_speed_control_.Brake_Gain = 0;
    airs_speed_control_.Brake_Integral = 0;
    airs_speed_control_.Brake_Integral_Rate = 1;
    airs_speed_control_.Brake_Derivative = 0;
    airs_speed_control_.Engine_Controller_beh = 0;
    airs_speed_control_.Brake_Controller_beh = 0;
    airs_speed_control_.Brake_Live_Calib_Pressure = 0;
    Acc_req_max_ = 0.6; //0.6m/s2 is the max safe acceleration of the APM's from initial tests in PSA APM

    // legacy vehicle state initilisation
    airs_vehicle_state_.header = std_msgs::Header();
    airs_vehicle_state_.Steering_Angle_Feedback = '0';
    airs_vehicle_state_.Vehicle_Speed_Feedback = '0';
    airs_vehicle_state_.PLC_Ready = '0';
    airs_vehicle_state_.Manual_Mode_Flag = '0';
    airs_vehicle_state_.Brake_Hard_Flag = '0';
    airs_vehicle_state_.Brake_Released_Flag = '0';
    airs_vehicle_state_.Safety_E_Stop_Triggered_Flag = '0';
    airs_vehicle_state_.Gear_Current_Status_Feedback = '0';


    airs_ServiceBrakeAirPressFrontRq_max = 33;
    airs_ServiceBrakeAirPressRearRq_max = 33; 
    airs_ServiceBrakeAirPressFrontRq_min = 0;
    airs_ServiceBrakeAirPressRearRq_min = 0;

    airs_ServiceBrakeAirPressFrontRq_gear = 33;
    airs_ServiceBrakeAirPressRearRq_gear = 33;
    airs_ServiceBrakeAirPressRq_Park = 20; 



    airs_DbwUser_01_.header = std_msgs::Header();
    airs_DbwUser_01_.IndicatorLightRequest =0;
    airs_DbwUser_01_.DippedBeamRequest =0;
    airs_DbwUser_01_.MainBeamRequest =0;
    airs_DbwUser_01_.AirHornRequest =0;
    airs_DbwUser_01_.FogLightRequest =0;
    airs_DbwUser_01_.RotatingBeaconLightRequest =0;
    airs_DbwUser_01_.WorkLightFrontRequest =0;
    airs_DbwUser_01_.WorkLightRearRequest =0;
    airs_DbwUser_01_.ModeLightRequest =0;
    airs_DbwUser_01_.MessageCounter =0;
    airs_DbwUser_01_.MessageChecksum =0;

    airs_DbwUser_02_.header = std_msgs::Header();
    airs_DbwUser_02_.EmergencyBrakeRequest = 0;
    airs_DbwUser_02_.MessageCounter =0;
    airs_DbwUser_02_.MessageChecksum =0;

    airs_DbwUser_03_.header = std_msgs::Header();
    airs_DbwUser_03_.FifthWheelHeightRequest = 0;
    airs_DbwUser_03_.FifthWheelUnlockRequest = 0;
    airs_DbwUser_03_.MessageCounter = 0;
    airs_DbwUser_03_.MessageChecksum = 0;

    airs_DbwUser_04_.header = std_msgs::Header();
    airs_DbwUser_04_.DBWControlRequest = 0;
    airs_DbwUser_04_.MessageCounter = 0;
    airs_DbwUser_04_.MessageChecksum = 0;

    airs_DbwUser_05_.header = std_msgs::Header();
    airs_DbwUser_05_.EngineOnOffRequest = 0;
    airs_DbwUser_05_.TransRequestedGear = 125;
    airs_DbwUser_05_.EngineTorqueRequest = 0;
    airs_DbwUser_05_.MessageCounter = 0;
    airs_DbwUser_05_.MessageChecksum = 0;

    airs_DbwUser_07_.header = std_msgs::Header();
    airs_DbwUser_07_.ServiceBrakeAirPressFrontRq =0;
    airs_DbwUser_07_.ServiceBrakeAirPressRearRq =0;
    airs_DbwUser_07_.MessageCounter =0;
    airs_DbwUser_07_.MessageChecksum =0;
    //wrapped_classification_msg_ptr->header =  std_msgs::Header();

    airs_AT2_DbwUser_01_.header = std_msgs::Header();
    airs_AT2_DbwUser_01_.ParkBrakeApplyRequest = 0;
    airs_AT2_DbwUser_01_.ParkBrakeReleaseRequest = 0;
    

    airs_AUX_JOY_P_.header = std_msgs::Header();
    airs_AUX_JOY_P_.AUX_JOY_POS_P = 1000;
    airs_AUX_JOY_P_.AUX_JOY_CL_trim_P = 200;
    airs_AUX_JOY_P_.AUX_JOY_CL_enable_P = 0;
    airs_AUX_JOY_P_.AUX_JOY_Seq_P = 0;
    airs_AUX_JOY_P_.AUX_JOY_CL_error_code_P = 15;
    airs_AUX_JOY_P_.AUX_JOY_CRC_P = 0;

    airs_AUX_JOY_R_.header = std_msgs::Header();
    airs_AUX_JOY_R_.AUX_JOY_POS_R = 1000;
    airs_AUX_JOY_R_.AUX_JOY_CL_trim_R = 200;
    airs_AUX_JOY_R_.AUX_JOY_CL_enable_R = 0;
    airs_AUX_JOY_R_.AUX_JOY_Seq_R = 0;
    airs_AUX_JOY_R_.AUX_JOY_CL_error_code_R = 15;
    airs_AUX_JOY_R_.AUX_JOY_CRC_R = 0;


    //steering control system initialisation
    airs_steering_control_.aidc_steering_deg_request = 0;
    airs_aidc_dashboard_cmd_.SteeringTrimRequest = 200;
    airs_steering_angle_max = 28;
    airs_steering_angle_min = -28;
  }


  void airs::dbw_control_system_class::aidcCrtlCmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    aidc_speed_cmd = msg->linear.x;
    aidc_steering_cmd = -1*msg->angular.z;
    airs_aidc_motion_request_time_check = ros::Time::now();
  }
  void airs::dbw_control_system_class::aidcGearRequestCallback(const std_msgs::Int64::ConstPtr &msg)
  {
    airs_aidc_dashboard_cmd_.GearInt = msg->data;
    
  }
  void airs::dbw_control_system_class::aidcIndicatorRequestCallback(const std_msgs::Int64::ConstPtr &msg)
  {
    airs_aidc_dashboard_cmd_.IndicatorLightRequest = msg->data;
    //aidc_indicator_light = msg->data;
    airs_aidc_indicator_request_time_check = ros::Time::now();
  }
  double NormalizeAngle(double x) {
    if (x < -M_PI) x += 2 * M_PI;
    if (x > M_PI) x -= 2 * M_PI;
    return x;
  }

  void airs::dbw_control_system_class::aiosAutoRequestCallback(const std_msgs::Int64::ConstPtr &msg)
  {

    // aios_auto_request =  msg->data;

    // airs_dbw_status_.airs_auto_state_indx = 9;
    // // && aios_auto_request_prev = 0
    // if (airs_dbw_status_.aios_auto_request_Latch == 0 && (aios_auto_request == 1) && (aios_auto_request_prev == 0))
    // {
    //   airs_dbw_status_.aios_auto_request_Latch = 1;
    // } else if (airs_dbw_status_.aios_auto_request_Latch == 1 && aios_auto_request == 1 && (aios_auto_request_prev == 0))
    // {
    //   airs_dbw_status_.aios_auto_request_Latch = 0;
    // }
    // aios_auto_request_prev = aios_auto_request;
    //ROS_INFO("aidc airs_dbw_status_.aios_auto_request_Latch %d", airs_dbw_status_.aios_auto_request_Latch);
  }

  void airs::dbw_control_system_class::aidcEStopRequestCallback (const std_msgs::Bool::ConstPtr &msg)
  {
        //    int aidc_e_stop_request;
        //int aidc_e_stop_feedback;
        //int aidc_e_stop_latch;
        aidc_e_stop_request = msg->data;
        airs_e_stop_request_time_check = ros::Time::now();

  }
  void airs::dbw_control_system_class::aidcEStopResetCallback (const std_msgs::Bool::ConstPtr &msg)
  {
    aios_error_reset_request = msg->data;
  }

  void airs::dbw_control_system_class::aidcDashboardCallback(const airs_msgs::airs_aidc_dashboard_cmd::ConstPtr &msg)
  {
    if ((ros::Time::now() - airs_aidc_gear_request_time_check) > airs_aidc_gear_request_time_diff)
      {
        //aidc_e_stop_request = false;  to be readded once aidc online
        airs_aidc_dashboard_cmd_.GearInt = msg->GearInt;
        airs_aidc_dashboard_cmd_.IndicatorLightRequest = msg->IndicatorLightRequest;
        airs_aidc_dashboard_cmd_.AirHornRequest = msg->AirHornRequest;
        //ROS_INFO(" Inside aidc status check");
      }
    
    
    airs_aidc_dashboard_cmd_.hazardLight = msg->hazardLight;
    airs_aidc_dashboard_cmd_.Estop = msg->Estop;
    aidc_e_stop_request = airs_aidc_dashboard_cmd_.Estop;

    if(msg->SteeringTrimRequest ==0)
    {
      airs_aidc_dashboard_cmd_.SteeringTrimRequest = 200;
    }else{
      airs_aidc_dashboard_cmd_.SteeringTrimRequest = msg->SteeringTrimRequest;
    }

  }
  void airs::dbw_control_system_class::aidcApmStatusCallback(const std_msgs::Int64::ConstPtr &msg)
  {
    aidc_apm_status= msg->data;
    //ROS_INFO(" aidc_apm_status %d", aidc_apm_status);
    airs_aidc_gear_request_time_check = ros::Time::now();
  }
  void airs::dbw_control_system_class::aiosDashboardCallback(const airs_msgs::airs_aios_dashboard_cmd::ConstPtr &msg)
  {
    /*// auto latch creation start  ////
      this latches the auto button of the gui.
    */
    aios_auto_request =  msg->AutoManualRequest;
    airs_dbw_status_.airs_auto_state_indx = 9;
    // && aios_auto_request_prev = 0
    if (airs_dbw_status_.aios_auto_request_Latch == 0 && (aios_auto_request == 1) && (aios_auto_request_prev == 0))
    {
      airs_dbw_status_.aios_auto_request_Latch = 1;
    } else if (airs_dbw_status_.aios_auto_request_Latch == 1 && aios_auto_request == 1 && (aios_auto_request_prev == 0))
    {
      airs_dbw_status_.aios_auto_request_Latch = 0;
    }
    aios_auto_request_prev = aios_auto_request;

    // the below line cancels the latch logic above as it is not needed due to the terberg's auto tug does not switch to manual from this request.
    airs_dbw_status_.aios_auto_request_Latch = aios_auto_request;

    /// auto latch creation end  /////

    airs_aios_dashboard_cmd_.DippedBeamRequest = msg->DippedBeamRequest;
    airs_aios_dashboard_cmd_.MainBeamRequest = msg->MainBeamRequest;
    airs_aios_dashboard_cmd_.FogLightRequest = msg->FogLightRequest;
    airs_aios_dashboard_cmd_.WorkLightFrontRequest = msg->WorkLightFrontRequest;
    airs_aios_dashboard_cmd_.WorkLightRearRequest = msg->WorkLightRearRequest;
    airs_aios_dashboard_cmd_.ModeLightRequest = msg->ModeLightRequest;
    airs_aios_dashboard_cmd_.RotatingBeaconLightRequest = msg->RotatingBeaconLightRequest;
    airs_aios_dashboard_cmd_.EngineStartRequest = msg->EngineStartRequest;
    //ROS_INFO(" airs_aios_d msg->DippedBeamRequest %d", msg->DippedBeamRequest);
  }

  /* ############################ ####################################################################
        Auto Tug callbacks Start:

      ###########################################
  */
  
  void airs::dbw_control_system_class::airsCCVS1FBCallback(const airs_msgs::CCVS1::ConstPtr &msg)
  {
    airs_AT1_CCVS1_.WheelBasedVehicleSpeed = msg->WheelBasedVehicleSpeed;
    airs_AT1_CCVS1_.ParkingBrakeSwitch = msg->ParkingBrakeSwitch;
    airs_AT1_CCVS1_.ParkBrakeReleaseInhibitRq = msg->ParkBrakeReleaseInhibitRq;
    airs_dbw_status_.airs_at_ParkBrake = airs_AT1_CCVS1_.ParkingBrakeSwitch;
    airs_dbw_status_.airs_at_ParkBrakeReleaseInhibitRq = airs_AT1_CCVS1_.ParkBrakeReleaseInhibitRq;
    //ROS_INFO("airs_AT1_VehSpeed_.Vehicle_Speed_feedback %f",airs_AT1_VehSpeed_.Vehicle_Speed_feedback);
  }

  void airs::dbw_control_system_class::airstransmissionFBCallback(const airs_msgs::ETC2::ConstPtr &msg)
  {
    airs_AT1_ETC2_.TransSelectedGear = msg->TransSelectedGear;
    airs_AT1_ETC2_.TransCurrentGear = msg->TransCurrentGear;
    //ROS_INFO("airs_AT1_ETC2_.TransSelectedGear = %d",airs_AT1_ETC2_.TransSelectedGear);
  }
  void airs::dbw_control_system_class::airsDashboardFBCallback(const airs_msgs::PropB_ATCC2_01::ConstPtr &msg)
  {
    airs_AT1_ATCC2_01_.AirHornState = msg->AirHornState;
    airs_AT1_ATCC2_01_.DippedBeamState = msg->DippedBeamState;
    airs_AT1_ATCC2_01_.MainBeamState = msg->MainBeamState;
    airs_AT1_ATCC2_01_.IndicatorLightState = msg->IndicatorLightState;

  }
  void airs::dbw_control_system_class::airsSpeedFBCallback(const airs_msgs::PropB_ATCC2_02::ConstPtr &msg)
  {
    airs_AT1_ATCC2_02_.BrakesLiftedState = msg->BrakesLiftedState;
    airs_AT1_ATCC2_02_.CORA_Velocity = msg->CORA_Velocity;
  }
  void airs::dbw_control_system_class::airsFluidlevelsFBCallback(const airs_msgs::PropB_ATCC2_03::ConstPtr &msg)
  {
    airs_AT1_ATCC2_03_.StoredESLvl = msg->StoredESLvl;
    airs_AT1_ATCC2_03_.AlternatorTelltale_Red = msg->AlternatorTelltale_Red;
    airs_AT1_ATCC2_03_.DieselExhaustFluidLevel = msg->DieselExhaustFluidLevel;
    airs_AT1_ATCC2_03_.RearAxDiffEngagedTelltale_Green = msg->RearAxDiffEngagedTelltale_Green;
    airs_AT1_ATCC2_03_.OilPressureLowTelltale_Red = msg->OilPressureLowTelltale_Red;
  }
  void airs::dbw_control_system_class::airsAutoManualFBCallback(const airs_msgs::PropB_ATCC2_05::ConstPtr &msg)
  {
    airs_AT1_ATCC2_05_.DBWControlState = msg->DBWControlState;
    airs_AT1_ATCC2_05_.SwitchToManualCause = msg->SwitchToManualCause;
    airs_AT1_ATCC2_05_.EmergencyBrakeState = msg->EmergencyBrakeState;
    airs_dbw_status_.airs_at_SwitchToManualCause = airs_AT1_ATCC2_05_.SwitchToManualCause;
    airs_dbw_status_.airs_at_DBWControlState = airs_AT1_ATCC2_05_.DBWControlState;
  }
  void airs::dbw_control_system_class::airsbrakeFBCallback(const airs_msgs::PropB_ATCC2_06::ConstPtr &msg)
  {
    
    airs_AT1_ATCC2_06_.DbwUserMode = msg->DbwUserMode;
    airs_AT1_ATCC2_06_.BrakeControlErrorState = msg->BrakeControlErrorState;
    airs_AT1_ATCC2_06_.BrakeSwitch = msg->BrakeSwitch;
    airs_AT1_ATCC2_06_.ActualServiceBrakeAirPressFront = (msg->ActualServiceBrakeAirPressFront)/3.75;
    airs_AT1_ATCC2_06_.ActualServiceBrakeAirPressRear = (msg->ActualServiceBrakeAirPressRear)/3.75;
    airs_AT1_ATCC2_06_.AvailableServiceBrakePressFront = msg->AvailableServiceBrakePressFront;
    airs_AT1_ATCC2_06_.AvailableServiceBrakePressRear = msg->AvailableServiceBrakePressRear;
    airs_dbw_status_.airs_at_DbwUserMode = airs_AT1_ATCC2_06_.DbwUserMode;
  }
  void airs::dbw_control_system_class::airsAccpedalLightsFBCallback(const airs_msgs::PropB_YTCC2_01::ConstPtr &msg)
  {
    airs_AT1_YTCC2_01_.FogLightState = msg->FogLightState;
    airs_AT1_YTCC2_01_.WorkLightFrontState = msg->WorkLightFrontState;
    airs_AT1_YTCC2_01_.WorkLightRearState = msg->WorkLightRearState;
    airs_AT1_YTCC2_01_.RotatingBeaconLightState = msg->RotatingBeaconLightState;
    airs_AT1_YTCC2_01_.AccelPedalPos1 = msg->AccelPedalPos1;
  }
  void airs::dbw_control_system_class::airsFithWheelFBCallback(const airs_msgs::PropB_YTCC2_02::ConstPtr &msg)
  {
    airs_AT1_YTCC2_02_.FifthWheelHeight = msg->FifthWheelHeight;
    airs_AT1_YTCC2_02_.FifthWheelLoad = msg->FifthWheelLoad;
    airs_AT1_YTCC2_02_.FifthWheeLockState = msg->FifthWheeLockState;
    airs_AT1_YTCC2_02_.KingPinPresence = msg->KingPinPresence;
    airs_AT1_YTCC2_02_.DriverSeated = msg->DriverSeated;
    airs_AT1_YTCC2_02_.SeatBeltFastened = msg->SeatBeltFastened;
    airs_AT1_YTCC2_02_.FifthWhlHeightControlErrorState = msg->FifthWhlHeightControlErrorState;
    airs_AT1_YTCC2_02_.FifthWhlControlInhibitState = msg->FifthWhlControlInhibitState;
  }
  void airs::dbw_control_system_class::airsEngineFBCallback(const airs_msgs::PropB_YTCC2_04::ConstPtr &msg)
  {
    airs_AT1_YTCC2_04_.EngineOnOffState = msg->EngineOnOffState;
    airs_AT1_YTCC2_04_.DriveDirCtrlErrState = msg->DriveDirCtrlErrState;
    airs_AT1_YTCC2_04_.DriveDirCtrlInhibState = msg->DriveDirCtrlInhibState;
    airs_AT1_YTCC2_04_.TorqueControlInhibitState = msg->TorqueControlInhibitState;
    airs_AT1_YTCC2_04_.TorqueControlErrorState = msg->TorqueControlErrorState;
    airs_AT1_YTCC2_04_.EngReferenceTorque = msg->EngReferenceTorque;


  }
  void airs::dbw_control_system_class::airsSteeringStatusFBCallback(const airs_msgs::STAT_MSG_OP_M::ConstPtr &msg)
  {
    airs_STAT_MSG_OP_M_.OperationState_M = msg->OperationState_M;
    airs_steering_control_.airs_steering_Operation_state = airs_STAT_MSG_OP_M_.OperationState_M;

    //ROS_INFO("airs_STAT_MSG_OP_M_.OperationState_M %d", airs_STAT_MSG_OP_M_.OperationState_M);

  }
  void airs::dbw_control_system_class::airsSteeringFBCallback(const airs_msgs::STR_FB_MSG_M::ConstPtr &msg)
  {
    airs_STR_FB_MSG_M_.STR_FB_Est_flow_M = msg->STR_FB_Est_flow_M;
    airs_STR_FB_MSG_M_.STR_FB_Est_WA_M = msg->STR_FB_Est_WA_M;
    airs_STR_FB_MSG_M_.IMD_STW_Status_M = msg->IMD_STW_Status_M;
    airs_STR_FB_MSG_M_.STR_FB_STW_L2L_M = msg->STR_FB_STW_L2L_M;

  }

  void airs::dbw_control_system_class::airsFSSFBCallback(const airs_msgs::Functional_safety_sensor::ConstPtr &msg)
  {
    airs_fss_fb_.Safety_SICK_Input_1 = msg->Safety_SICK_Input_1;
    airs_fss_fb_.Safety_SICK_Input_2 = msg->Safety_SICK_Input_2;
    airs_fss_fb_.Safety_SICK_Input_3 = msg->Safety_SICK_Input_3;
    airs_fss_fb_.Safety_SICK_Input_4 = msg->Safety_SICK_Input_4;
    airs_fss_fb_.Safety_SICK_Input_5 = msg->Safety_SICK_Input_5;
    airs_fss_fb_.Safety_SICK_Input_6 = msg->Safety_SICK_Input_6;
    airs_fss_fb_.Safety_SICK_Input_7 = msg->Safety_SICK_Input_7;
    airs_fss_fb_.Safety_SICK_Input_8 = msg->Safety_SICK_Input_8;
    
  }

  void airs::dbw_control_system_class::airsFSVCUFBCallback(const airs_msgs::Functional_safety_vcu::ConstPtr &msg)
  {
    airs_fss_fb_.Physical_Safety_Estop_Triggered = msg->Physical_Safety_Estop_Triggered;
    
  }

  /* ############################

      Auto Tug callbacks End:

    ###########################################
  */  
    ////#################################### publish_dbw_user_msg ####################################    
  void airs::dbw_control_system_class::publish_dbw_user_msg()
  {
    // ## Version Control
    airs_dbw_status_.airs_version = 0.14;
    airs_dbw_status_.airs_version_release_date = "06/12/2022";

    // ## Auto manual Setup and Process ##
    airs_auto_manual_function();
    airs_emergency_function();
    airs_dashboard_function();
    airs_speed_controller();
    airs_steering_controller();
    airs_vehicle_fith_wheel();
    airs_vehicle_state_msg();
    airs_functional_safety_system();
    


    //ROS_INFO("checksum_counter_arr[1] %d", checksum_counter_arr[1]);
    airs_dbw_status_.header.stamp = ros::Time::now ();
    airs_dbw_dashboard_fb_.header.stamp = ros::Time::now ();
    airs_speed_control_.header.stamp = ros::Time::now ();
    airs_steering_control_.header.stamp = ros::Time::now ();
    airs_DbwUser_01_.header.stamp = ros::Time::now ();
    airs_DbwUser_02_.header.stamp = ros::Time::now ();
    airs_DbwUser_03_.header.stamp = ros::Time::now ();
    airs_DbwUser_04_.header.stamp = ros::Time::now ();
    airs_DbwUser_05_.header.stamp = ros::Time::now ();
    airs_DbwUser_07_.header.stamp = ros::Time::now ();
    airs_vehicle_state_.header.stamp = ros::Time::now ();
    airs_AT2_DbwUser_01_.header.stamp = ros::Time::now ();
    airs_vehicle_state_.header.stamp = ros::Time::now ();
    airs_AUX_JOY_P_.header.stamp = ros::Time::now ();
    airs_AUX_JOY_R_.header.stamp = ros::Time::now ();
    airs_dbw_status_pub.publish(airs_dbw_status_);
    airs_dashboard_fb_pub.publish(airs_dbw_dashboard_fb_);

    airs_speed_control_pub.publish(airs_speed_control_);
    airs_vehicle_state_pub.publish(airs_vehicle_state_);


    dbw_user_dashboard_pub.publish(airs_DbwUser_01_);
    dbw_user_E_brake_pub.publish(airs_DbwUser_02_);
    dbw_user_fith_wheel_pub.publish(airs_DbwUser_03_);
    dbw_user_Auto_pub.publish(airs_DbwUser_04_);
    dbw_user_Engine_pub.publish(airs_DbwUser_05_);
    dbw_user_brake_pub.publish(airs_DbwUser_07_);


    dbw_user_P_brake_pub.publish(airs_AT2_DbwUser_01_);


    airs_steering_control_pub.publish(airs_steering_control_);
    dbw_user_steering_pub.publish(airs_AUX_JOY_P_);
    dbw_user_steering_r_pub.publish(airs_AUX_JOY_R_);
  }
  ////#################################### airs_auto_manual_function ####################################
  void airs::dbw_control_system_class::airs_auto_manual_function(){
    /*
      The auto manual button is designed to be a push button as depending on the circumstances it may be controlled from
      multiple different systems. but all logic controlling it is done in one system
    */
    if (airs_AT1_ATCC2_06_.DbwUserMode == 2 && airs_AT1_ATCC2_05_.DBWControlState == 2)
    {
      airs_dbw_status_.airs_apm_in_auto = true;
    }else{
      airs_dbw_status_.airs_apm_in_auto = false;
    }

    if (airs_AT1_ATCC2_05_.DBWControlState >= 1 && aidc_speed_cmd == 0)
    {
      airs_dbw_status_.airs_auto_ready = true;
      airs_dbw_status_.airs_auto_state_indx = 1;
    }else
    {
      airs_dbw_status_.airs_auto_ready = false;
      airs_dbw_status_.airs_auto_state_indx = 2;
    }

    if (airs_AT1_ATCC2_05_.SwitchToManualCause >1)
    {
      airs_dbw_status_.airs_manual_override = true;
      airs_dbw_status_.airs_auto_state_indx = 3;
    }else{
      airs_dbw_status_.airs_manual_override = false;
      airs_dbw_status_.airs_auto_state_indx = 4;
    }


    if (airs_dbw_status_.airs_manual_override == true)
    {
      airs_DbwUser_04_.DBWControlRequest = 0;
    }else if (airs_dbw_status_.aios_auto_request_Latch >=1){
      airs_DbwUser_04_.DBWControlRequest = 1;
    } else{
      airs_DbwUser_04_.DBWControlRequest = 0;
    }
  }
  ////#################################### airs_emergency_function ####################################
  void airs::dbw_control_system_class::airs_emergency_function(){
    /*
      The emergency stop request will if the apm is in auto immediately send a stop request
      The system uses a error reset button and a latch based system, it will only unlatch if the aidc has stopped sending estop
      and if 
    */


    //airs_AT1_ATCC2_05_.EmergencyBrakeState
    
    // Emergency critical failure from drive by wire system detected Start:
    
    if (airs_STAT_MSG_OP_M_.OperationState_M == 255 || airs_AT1_ATCC2_06_.BrakeControlErrorState > 0 || airs_AT1_YTCC2_04_.DriveDirCtrlErrState > 0 || airs_AT1_YTCC2_04_.TorqueControlErrorState > 0){
      airs_dbw_Emergency_Trigger = true;
    }
    else{
      airs_dbw_Emergency_Trigger = false;
    } 
      


    // Emergency critical failure from drive by wire system detected End:


    int aidc_estop =0;

    
    airs_e_stop_function_time_check = ros::Time::now();
    // this function below determines if aidc has stopped publishing.
    if ((airs_e_stop_function_time_check.toSec() - airs_e_stop_request_time_check.toSec()) > airs_e_stop_request_time_diff.toSec())
    {
      //aidc_e_stop_request = false;  to be readded once aidc online
      
    }
    //printf("e_stop_request %d\n", aidc_e_stop_request); 
    if ((aidc_e_stop_request == true || Functional_Safety_System_Fail == true || airs_fss_fb_.Physical_Safety_Estop_Triggered == 1) && airs_dbw_status_.airs_apm_in_auto ==true)
    {

      airs_dbw_status_.airs_e_stop_latch = true;
    }else if (aios_error_reset_request == true && airs_speed_control_.V_actual == 0.0 && aidc_e_stop_request == false)
    {
      airs_dbw_status_.airs_e_stop_latch = false;
    }

    if (airs_dbw_status_.airs_e_stop_latch == true || airs_dbw_Emergency_Trigger == true)
    {
      airs_DbwUser_02_.EmergencyBrakeRequest = 1;
    } else {
      airs_DbwUser_02_.EmergencyBrakeRequest = 0;
    }
    if (aios_error_reset_request == true && airs_DbwUser_02_.EmergencyBrakeRequest == 1 && airs_speed_control_.V_actual > 0.0)
    {
      airs_dbw_status_.airs_error_reset_state = " current vehicle speed is above 0.0";
    }else if (aios_error_reset_request == true && airs_DbwUser_02_.EmergencyBrakeRequest == 1 && aidc_e_stop_request == true)
    {
      airs_dbw_status_.airs_error_reset_state = "system diagnostics is reporting an error";
    }
  
  }
  ////#################################### airs_auto_manual_function ####################################
  void airs::dbw_control_system_class::airs_dashboard_function(){
    if (airs_dbw_status_.airs_apm_in_auto == true)
    {




      //ROS_INFO(" airs_aios_dashboard_cmd_.DippedBeamRequest %d", airs_aios_dashboard_cmd_.DippedBeamRequest);
      if (airs_aidc_dashboard_cmd_.hazardLight == true || airs_DbwUser_02_.EmergencyBrakeRequest == 1 || airs_dbw_status_.airs_safe_stop == true){
        airs_DbwUser_01_.IndicatorLightRequest = 3; // hazard light
      }else if (airs_aidc_dashboard_cmd_.IndicatorLightRequest ==1){
        airs_DbwUser_01_.IndicatorLightRequest = 2; // Right
      }else if (airs_aidc_dashboard_cmd_.IndicatorLightRequest ==2){
        airs_DbwUser_01_.IndicatorLightRequest = 1; // left

      }else if (airs_aidc_dashboard_cmd_.IndicatorLightRequest ==0){
        airs_DbwUser_01_.IndicatorLightRequest = 0;
      }

      airs_DbwUser_01_.DippedBeamRequest = airs_aios_dashboard_cmd_.DippedBeamRequest;
      airs_DbwUser_01_.MainBeamRequest = airs_aios_dashboard_cmd_.MainBeamRequest;
      airs_DbwUser_01_.AirHornRequest = airs_aidc_dashboard_cmd_.AirHornRequest; // sim-passed
      airs_DbwUser_01_.FogLightRequest = airs_aios_dashboard_cmd_.FogLightRequest;
      airs_DbwUser_01_.RotatingBeaconLightRequest = airs_aios_dashboard_cmd_.RotatingBeaconLightRequest;
      airs_DbwUser_01_.WorkLightFrontRequest = airs_aios_dashboard_cmd_.WorkLightFrontRequest;
      airs_DbwUser_01_.WorkLightRearRequest = airs_aios_dashboard_cmd_.WorkLightRearRequest;

      if (airs_STAT_MSG_OP_M_.OperationState_M == 255)
      {
        airs_DbwUser_01_.ModeLightRequest = 1;
        
      } else if(airs_DbwUser_02_.EmergencyBrakeRequest == 1)
      {
        airs_DbwUser_01_.ModeLightRequest = 2;
      }
      else if (airs_DbwUser_04_.DBWControlRequest >0)
      {
        airs_DbwUser_01_.ModeLightRequest = 6;
      }
      
      
      //airs_DbwUser_01_.ModeLightRequest = airs_aios_dashboard_cmd_.ModeLightRequest;

      airs_DbwUser_05_.EngineOnOffRequest = airs_aios_dashboard_cmd_.EngineStartRequest;
      //ROS_INFO(@@)
      /* Gear change
        Hex: - Dec: - Action:

        0x7D - 125  - Neutral
        0xDF - 223  - Reverse
        0xFC - 252  - Forward
        0xFE - 254  - Error
      
      */
      if (airs_AT1_CCVS1_.ParkingBrakeSwitch == 1)
      {
        airs_park_brake_release_check = ros::Time::now();
      }


      if (airs_AT1_ATCC2_06_.ActualServiceBrakeAirPressRear >= 29 && airs_AT1_ATCC2_06_.ActualServiceBrakeAirPressFront >= 32){
        airs_gear_brake_pressure = true;
      }
      else
      {
        airs_gear_brake_pressure = false;
      }
      airs_aidc_gear_check = ros::Time::now(); 

      if (airs_speed_control_.V_actual <= 0.3 && airs_aidc_dashboard_cmd_.GearInt == 1 &&  airs_AT1_ETC2_.TransSelectedGear < 0 ){ //airs_DbwUser_05_.TransRequestedGear == 223 &&
        airs_DbwUser_05_.TransRequestedGear = 125;
        airs_aidc_gear_timer = ros::Time::now();
      }else if (airs_speed_control_.V_actual <= 0.3 && airs_aidc_dashboard_cmd_.GearInt == 1 && airs_AT1_CCVS1_.ParkingBrakeSwitch ==0 && airs_AT1_ETC2_.TransSelectedGear == 0 && airs_gear_brake_pressure ==true &&  ((airs_aidc_gear_check-airs_aidc_gear_timer) > airs_aidc_gear_duration)){ //airs_DbwUser_05_.TransRequestedGear == 125 && 
        
        //ROS_INFO(" airs_park_brake_release_diff: %lf ", airs_park_brake_release_diff.toSec() );
        
        if ((ros::Time::now() - airs_park_brake_release_check) > airs_park_brake_release_duration)
        {
          airs_DbwUser_05_.TransRequestedGear = 252;
        }
      } else if(airs_speed_control_.V_actual <= 0.3 && airs_aidc_dashboard_cmd_.GearInt == 2 &&  airs_AT1_ETC2_.TransSelectedGear >0){ //airs_DbwUser_05_.TransRequestedGear == 252 &&
        airs_DbwUser_05_.TransRequestedGear = 125;
        airs_aidc_gear_timer = ros::Time::now();
      }else if(airs_speed_control_.V_actual <= 0.3 && airs_aidc_dashboard_cmd_.GearInt == 2 && airs_AT1_CCVS1_.ParkingBrakeSwitch ==0 && airs_AT1_ETC2_.TransSelectedGear == 0 && airs_gear_brake_pressure == true && ((airs_aidc_gear_check-airs_aidc_gear_timer) > airs_aidc_gear_duration)){ //airs_DbwUser_05_.TransRequestedGear == 125 &&
        if ((ros::Time::now()- airs_park_brake_release_check) > airs_park_brake_release_duration)
        {
          
          airs_DbwUser_05_.TransRequestedGear = 223;
        }
        
      }else if (airs_speed_control_.V_actual <= 0.3 && airs_aidc_dashboard_cmd_.GearInt == 0){

        airs_DbwUser_05_.TransRequestedGear = 125;
      }
      
    }else {
      airs_DbwUser_01_.IndicatorLightRequest =0;
      airs_DbwUser_01_.DippedBeamRequest =0;
      airs_DbwUser_01_.MainBeamRequest =0;
      airs_DbwUser_01_.AirHornRequest =0;
      airs_DbwUser_01_.FogLightRequest =0;
      airs_DbwUser_01_.RotatingBeaconLightRequest =0;
      airs_DbwUser_01_.WorkLightFrontRequest =0;
      airs_DbwUser_01_.WorkLightRearRequest =0;
      airs_DbwUser_01_.ModeLightRequest =0;

    }
    /// ####   visualisation status messages  ### ///
    if(airs_AT1_ETC2_.TransSelectedGear == 0)
    {
      airs_dbw_status_.airs_Gear_State = "N";
    } else if (airs_AT1_ETC2_.TransSelectedGear > 0)
    {
      airs_dbw_status_.airs_Gear_State = "D";
    }else if (airs_AT1_ETC2_.TransSelectedGear < 0)
    {
      airs_dbw_status_.airs_Gear_State = "R";
    }

    if(airs_AT1_YTCC2_04_.EngineOnOffState ==1)
    {
      airs_dbw_status_.airs_at_Engine_State = "Engine On";
    } else
    {
      airs_dbw_status_.airs_at_Engine_State = "Engine OFF";
    }
  }
  ////////// ###########  AIRS SPEED CONTROLLER #########################   /////////////////////
  //
  //
  //
  //////////###########  AIRS SPEED CONTROLLER #########################/////////////////////////////

  void airs::dbw_control_system_class::airs_speed_controller(){

    airs_speed_control_.V_actual = airs_AT1_CCVS1_.WheelBasedVehicleSpeed;
    airs_speed_control_.V_req = 3.6*aidc_speed_cmd;
    airs_speed_control_.V_error = airs_speed_control_.V_req - airs_speed_control_.V_actual;

    // ############# Determining Acceleration Errors
    airs_speed_control_.Acc_actual = (airs_speed_control_.V_actual - V_prev)/airs_speed_control_.delta_t_act;
    airs_speed_control_.Acc_req = (airs_speed_control_.V_req - airs_speed_control_.V_actual)/airs_speed_control_.delta_t;
    airs_speed_control_.Acc_error = airs_speed_control_.Acc_req - airs_speed_control_.Acc_actual;

    if (airs_speed_control_.V_req ==0.0 && airs_speed_control_.Acc_error > 1.0)
    {
      airs_speed_control_.Acc_error = 1.0;
    }



    if (ros::Time::now()- airs_previous_velocity_time > airs_previous_velocity_time_diff)
    { 
      Brake_Live_Calib_timeout = true;
      airs_previous_velocity_time = ros::Time::now();
      
      V_prev_array[4] = airs_speed_control_.V_actual;
      Brake_Pressure_Feedback_array[4] =  airs_AT1_ATCC2_06_.ActualServiceBrakeAirPressFront;

      for (int iv=0; iv<4; iv++)
      {
        V_prev_array[iv]=V_prev_array[iv+1];
        Brake_Pressure_Feedback_array[iv]=Brake_Pressure_Feedback_array[iv+1];
      }
    }else{
      Brake_Live_Calib_timeout = false;
    }
    Brake_Pressure_Feedback_avg = (Brake_Pressure_Feedback_array[4] + Brake_Pressure_Feedback_array[3] + Brake_Pressure_Feedback_array[42])/3;
    V_prev = V_prev_array[0];

     //####### brake pressureaverage

    Brake_Pressure_Feedback_array [5];
    Brake_Pressure_Feedback_avg;



    if (airs_speed_control_.V_error <= 0.0)
    {

      param_Brake_de_acc_multiplier = 9.0;
      if (airs_speed_control_.V_error <= -5.0)
      {
        param_Engine_de_acc_multiplier = 50.0;
      }
      else{
        param_Engine_de_acc_multiplier = 16.0;
      }
      
    }else{
      param_Engine_de_acc_multiplier = 1.0;
      
      if (airs_speed_control_.V_actual == 0 && airs_speed_control_.V_req > 0 && airs_DbwUser_07_.ServiceBrakeAirPressFrontRq >= airs_ServiceBrakeAirPressRq_Park)
      {
        param_Brake_de_acc_multiplier= 10.0;
      }
      else if (airs_speed_control_.V_error >= 5.0)
      {
        param_Brake_de_acc_multiplier= 10.0;
      }
      else if(airs_speed_control_.V_error >= 3){
        param_Brake_de_acc_multiplier = 3;
      }
      else{
        param_Brake_de_acc_multiplier = 1;
      }

    }
    //########################## Engine Controller start #############################################//
    if (airs_engine_reset_ ==true)
    {
      airs_speed_control_.Engine_Integral=0;
      //airs_speed_control_.Engine_ref_fx = 0;
    }
    
    airs_speed_control_.Engine_Gain = airs_speed_control_.Acc_error* airs_speed_control_param_.Engine_Gain_Param;
    airs_speed_control_.Engine_Integral = airs_speed_control_.Engine_Integral + param_Engine_de_acc_multiplier*airs_speed_control_.Acc_error*airs_speed_control_.Engine_Integral_Rate*airs_speed_control_param_.Engine_Integral_Param; 
    airs_speed_control_.Engine_Derivative = (airs_speed_control_.Acc_error -Acc_prev)*airs_speed_control_param_.Engine_Derivative_Param; // +airs_speed_control_.V_req*airs_speed_control_param_.Vehicle_Dynamic_Param;  

    airs_speed_control_.Engine_ref_fx = airs_speed_control_.Engine_Gain + airs_speed_control_.Engine_Integral + airs_speed_control_.Engine_Derivative;
    airs_speed_control_.Engine_Controller_beh = 1;
    if (airs_speed_control_.Engine_Integral >= Engine_Integral_max){
        airs_speed_control_.Engine_Integral = Engine_Integral_max;
      } else if(airs_speed_control_.Engine_Integral <= 0){
        airs_speed_control_.Engine_Integral = 0;
      }
    if (airs_speed_control_.Engine_ref_fx >= Engine_Integral_max){
        airs_speed_control_.Engine_ref_fx = Engine_Integral_max;
        airs_speed_control_.Engine_Controller_beh = 4;
      } else if(airs_speed_control_.Engine_ref_fx <= 0)
      {
        airs_speed_control_.Engine_ref_fx = 0;
        airs_speed_control_.Engine_Controller_beh = 5;
    }

    if(airs_AT1_ATCC2_05_.EmergencyBrakeState >= 1){
      airs_DbwUser_05_.EngineTorqueRequest = 0;
      airs_engine_reset_ = true;
      airs_speed_control_.Engine_Controller_beh = 2;
    }

    else if (airs_dbw_status_.airs_apm_in_auto == true && airs_AT1_ETC2_.TransSelectedGear != 0 && airs_AT1_ATCC2_02_.BrakesLiftedState >= 1 && airs_AT1_YTCC2_04_.TorqueControlInhibitState <7 && airs_dbw_status_.airs_safe_stop == false)

    {
      airs_speed_control_.Engine_Controller_beh = 3;
      airs_engine_reset_ = false;  
      
      if (airs_engine_ref_prev < (airs_speed_control_.Engine_ref_fx-5))
      {
        airs_engine_ref_cnt = airs_engine_ref_cnt+2;
        if (airs_engine_ref_cnt>= 90){
          airs_engine_ref_cnt =90;
        }
        //ROS_INFO(" airs_engine_ref_cnt %d ",airs_engine_ref_cnt);
        //ROS_INFO(" sin %f ",sin(airs_engine_ref_cnt*PI/180));
        airs_DbwUser_05_.EngineTorqueRequest = airs_engine_ref_prev +(airs_speed_control_.Engine_ref_fx-airs_engine_ref_prev)*sin(airs_engine_ref_cnt*PI/180);
      }
      else{
        airs_engine_ref_cnt =5;
        airs_DbwUser_05_.EngineTorqueRequest = airs_speed_control_.Engine_ref_fx;
      }
      

      airs_engine_ref_prev = airs_DbwUser_05_.EngineTorqueRequest;
    } else{
      airs_engine_reset_ = true;
      airs_DbwUser_05_.EngineTorqueRequest = 0;
      airs_speed_control_.Engine_Controller_beh = 6;
    }
    //########################## Engine Controller end #############################################//

    //########################## brake  Controller start #############################################//
    if (airs_brake_reset_== true)
    {
      airs_speed_control_.Brake_Integral =0;
      //airs_speed_control_.Brake_Front_ref_fx=0;
    }
    /*
      // ###   Brake Controller Gain, Integral & Derivative Calculations ###
      // ###   Start  ###
    */
    airs_speed_control_.Brake_Gain = (-1*airs_speed_control_.Acc_error)*airs_speed_control_param_.Brake_Gain_Param;
    airs_speed_control_.Brake_Integral = airs_speed_control_.Brake_Integral + param_Brake_de_acc_multiplier*(-1*airs_speed_control_.Acc_error)*airs_speed_control_.Brake_Integral_Rate*airs_speed_control_param_.Brake_Integral_Param; 
    airs_speed_control_.Brake_Derivative = ((-1*airs_speed_control_.Acc_error)* -(-1*Acc_prev))*airs_speed_control_param_.Brake_Derivative_Param; 
    /*
      // <<<   Brake Controller Gain, Integral & Derivative Calculations 
      // <<<   End  
    */
    
    
      // >>>   Vehicle Dynamic Brake coefficient algorithm for low speed brake control 
      // >>>>   Start  >>>>>
    
    if (airs_speed_control_.V_req > 0 && airs_speed_control_.V_req <5.0 && airs_speed_control_.V_actual <5.0)
    {
      
      if(Brake_Live_Calib_timeout == true && airs_speed_control_.V_actual <= 0.1 && airs_speed_control_.Brake_Integral <= 0.1) // slowely reduced the holding brake pressure
      {
        Brake_live_Calib_Value = Brake_live_Calib_Value*0.955;
      }else if(Brake_live_Calib_Value <= 0.5 &&  airs_speed_control_.V_error <=0)
      {
        Brake_live_Calib_Value = Brake_live_Calib_Value*1.1;
        if (Brake_live_Calib_Value >=1)
        {
          Brake_live_Calib_Value = 1;
        }

      }
      airs_speed_control_.Brake_Live_Calib_Pressure = Brake_live_Calib_Value*airs_speed_control_param_.Vehicle_Dynamic_Param/(airs_speed_control_.V_req+0.1);
      if (airs_speed_control_.Brake_Live_Calib_Pressure >= airs_ServiceBrakeAirPressRq_Park/2)
      {
        airs_speed_control_.Brake_Live_Calib_Pressure = airs_ServiceBrakeAirPressRq_Park/2;
      }

      if (aidc_steering_cmd_prev >= airs_speed_control_.V_req && V_prev >= 5.0 && Brake_low_speed == false)
      {
        airs_speed_control_.Brake_Integral = airs_speed_control_.Brake_Integral - airs_speed_control_.Brake_Live_Calib_Pressure;
      }
      Brake_low_speed = true;
    }else{
      airs_speed_control_.Brake_Live_Calib_Pressure = 0;
      Brake_live_Calib_Value = 1;
      Brake_low_speed = false;
    }
    aidc_steering_cmd_prev = airs_speed_control_.V_req;
    /*
      // <<<<   Vehicle Dynamic Brake coefficient algorithm for low speed brake control 
      // <<<<   End  <<<<<
    */
    if (airs_aidc_dashboard_cmd_.GearInt > 0 && airs_speed_control_.V_req == 0.0 && airs_speed_control_.V_actual <= 0.0)
    {
      airs_speed_control_.Brake_Integral = airs_ServiceBrakeAirPressFrontRq_gear;
      airs_speed_control_.Brake_Controller_beh = 1;
      //airs_engine_reset_ = true;
    }
    if (airs_speed_control_.Brake_Integral >= airs_ServiceBrakeAirPressFrontRq_max)
    {

      airs_speed_control_.Brake_Controller_beh = 2;

      airs_speed_control_.Brake_Integral = airs_ServiceBrakeAirPressFrontRq_max;
    } else if (airs_speed_control_.Brake_Integral <= airs_ServiceBrakeAirPressFrontRq_min)
    {
      airs_speed_control_.Brake_Integral = airs_ServiceBrakeAirPressFrontRq_min;

      airs_speed_control_.Brake_Controller_beh = 3;

    }



    airs_speed_control_.Brake_Front_ref_fx = airs_speed_control_.Brake_Gain + airs_speed_control_.Brake_Integral + airs_speed_control_.Brake_Derivative; // + airs_speed_control_.Brake_Live_Calib_Pressure;

    airs_speed_control_.Brake_Rear_ref_fx = airs_speed_control_.Brake_Gain + airs_speed_control_.Brake_Integral + airs_speed_control_.Brake_Derivative; // + airs_speed_control_.Brake_Live_Calib_Pressure;
    

    if (airs_speed_control_.Brake_Front_ref_fx >= airs_ServiceBrakeAirPressFrontRq_max){
        airs_speed_control_.Brake_Front_ref_fx = airs_ServiceBrakeAirPressFrontRq_max;
        airs_speed_control_.Brake_Rear_ref_fx = airs_ServiceBrakeAirPressRearRq_max;
        airs_speed_control_.Brake_Integral = airs_ServiceBrakeAirPressFrontRq_max;

        airs_speed_control_.Brake_Controller_beh = 9;


      } else if(airs_speed_control_.Brake_Front_ref_fx <= airs_ServiceBrakeAirPressFrontRq_min){

        airs_speed_control_.Brake_Front_ref_fx = airs_ServiceBrakeAirPressFrontRq_min;
        airs_speed_control_.Brake_Rear_ref_fx = airs_ServiceBrakeAirPressFrontRq_min;
        airs_speed_control_.Brake_Integral = airs_ServiceBrakeAirPressFrontRq_min;

        airs_speed_control_.Brake_Controller_beh = 10;

      }

    /* 
      ###  Park brake request && pressure brake request when APM in Neutral 
      --------------------------------------------------------------------------------
    */
    if (airs_aidc_dashboard_cmd_.GearInt == 0 && airs_AT1_ETC2_.TransSelectedGear == 0 && airs_speed_control_.V_actual < 3) // if vehicle in neutral - apply park brake
    {
      

      if (airs_AT1_CCVS1_.ParkingBrakeSwitch ==0)
      {
        airs_speed_control_.Brake_Front_ref_fx = airs_ServiceBrakeAirPressRq_Park;
        airs_speed_control_.Brake_Rear_ref_fx = airs_ServiceBrakeAirPressRq_Park;
        airs_speed_control_.Brake_Integral = airs_ServiceBrakeAirPressRq_Park;
        airs_AT2_DbwUser_01_.ParkBrakeApplyRequest = 1;
        airs_AT2_DbwUser_01_.ParkBrakeReleaseRequest = 0;
      }
      else{
        airs_speed_control_.Brake_Front_ref_fx = airs_ServiceBrakeAirPressFrontRq_min;
        airs_speed_control_.Brake_Rear_ref_fx = airs_ServiceBrakeAirPressFrontRq_min;
        airs_speed_control_.Brake_Integral = airs_ServiceBrakeAirPressFrontRq_min;

      }
      
      airs_speed_control_.Brake_Controller_beh = 4;

    }
    else if(airs_aidc_dashboard_cmd_.GearInt >= 0 && airs_AT1_ETC2_.TransSelectedGear == 0)
    {
      if (airs_gear_brake_pressure == true)
      {
        airs_AT2_DbwUser_01_.ParkBrakeApplyRequest = 0;
        airs_AT2_DbwUser_01_.ParkBrakeReleaseRequest = 1;
        
      }
      

      airs_speed_control_.Brake_Controller_beh = 5;

    }
    /* 
        ## Primary brake controller if statement Start  
        -------------------------------------------------------------------------------
    */
    if (airs_aidc_dashboard_cmd_.GearInt > 0 && airs_AT1_CCVS1_.ParkingBrakeSwitch ==1)
    {
      airs_DbwUser_07_.ServiceBrakeAirPressFrontRq = airs_ServiceBrakeAirPressFrontRq_gear;
      airs_DbwUser_07_.ServiceBrakeAirPressRearRq = airs_ServiceBrakeAirPressRearRq_gear;
      airs_speed_control_.Brake_Controller_beh = 12;
    }
    else if ((airs_AT1_ETC2_.TransSelectedGear == 0 && airs_AT1_CCVS1_.ParkingBrakeSwitch ==0)) // applies brake for gear changing
    {
      airs_DbwUser_07_.ServiceBrakeAirPressFrontRq = airs_ServiceBrakeAirPressFrontRq_gear;
      airs_DbwUser_07_.ServiceBrakeAirPressRearRq = airs_ServiceBrakeAirPressRearRq_gear;

      airs_speed_control_.Brake_Controller_beh = 6;

    }  
    else if(airs_dbw_status_.airs_apm_in_auto == true && (airs_AT1_ATCC2_05_.EmergencyBrakeState >= 1 || airs_dbw_status_.airs_safe_stop == true)){
      airs_DbwUser_07_.ServiceBrakeAirPressFrontRq = airs_ServiceBrakeAirPressFrontRq_max;
      airs_DbwUser_07_.ServiceBrakeAirPressRearRq = airs_ServiceBrakeAirPressRearRq_max;
      airs_brake_reset_ = true;

      airs_speed_control_.Brake_Controller_beh = 7;

    }
    else if (airs_dbw_status_.airs_apm_in_auto == true && airs_DbwUser_05_.EngineTorqueRequest <= 6)
    {
      airs_brake_reset_ = false;  

      airs_speed_control_.Brake_Controller_beh = 8;

      airs_DbwUser_07_.ServiceBrakeAirPressFrontRq = airs_speed_control_.Brake_Front_ref_fx + airs_speed_control_.Brake_Live_Calib_Pressure;
      airs_DbwUser_07_.ServiceBrakeAirPressRearRq = 0.8*airs_speed_control_.Brake_Rear_ref_fx + airs_speed_control_.Brake_Live_Calib_Pressure;

    } else{
      airs_DbwUser_07_.ServiceBrakeAirPressFrontRq = airs_ServiceBrakeAirPressFrontRq_min;
      airs_DbwUser_07_.ServiceBrakeAirPressRearRq = airs_ServiceBrakeAirPressRearRq_min;
      airs_brake_reset_ = true;

      airs_speed_control_.Brake_Controller_beh = 11;

    }
    //// ## Primary brake controller if statement End  ///////////
        //########################## brake  Controller end #############################################//


  }
  void airs::dbw_control_system_class::airs_steering_controller(){
    /*
      The steering controller receives commands from the aidc angular.z 
      and translate it to the steering angle required for the terberg 
      system which uses danfoss.
      the terberg steering system is maxxed at 32 degrees left and 34 degrees right.
    */
    airs_steering_control_.airs_steering_trim_request = airs_aidc_dashboard_cmd_.SteeringTrimRequest;

    airs_steering_control_.airs_steering_deg_fb = ((float)airs_STR_FB_MSG_M_.STR_FB_Est_WA_M/1000.0)*32.0; //32 degrees left and 34 degrees right
    //ROS_INFO("airs_steering_control_.airs_steering_deg_fb %f", airs_steering_control_.airs_steering_deg_fb);

    
    airs_steering_control_.aidc_steering_deg_request = 57.29578*aidc_steering_cmd;
    if (airs_aidc_dashboard_cmd_.GearInt >= 0 && airs_dbw_status_.airs_apm_in_auto == true)
    {
      if (airs_steering_angle_max <= airs_steering_control_.aidc_steering_deg_request )
      {
        airs_steering_control_.aidc_steering_deg_request = airs_steering_angle_max;
      } 
      else if (airs_steering_control_.aidc_steering_deg_request <= airs_steering_angle_min)
      {
        airs_steering_control_.aidc_steering_deg_request = airs_steering_angle_min;
      }
      airs_steering_control_.airs_steering_POS_JOY = 1000 + (1000/32)*airs_steering_control_.aidc_steering_deg_request;

     
      
      airs_AUX_JOY_P_.AUX_JOY_CL_enable_P = 1;
      airs_AUX_JOY_R_.AUX_JOY_CL_enable_R = 1;
      airs_AUX_JOY_P_.AUX_JOY_POS_P = airs_steering_control_.airs_steering_POS_JOY;
      airs_AUX_JOY_R_.AUX_JOY_POS_R = airs_steering_control_.airs_steering_POS_JOY; 
      airs_AUX_JOY_P_.AUX_JOY_CL_trim_P = airs_steering_control_.airs_steering_trim_request;
      airs_AUX_JOY_R_.AUX_JOY_CL_trim_R = airs_steering_control_.airs_steering_trim_request;
    }else
    {
      airs_AUX_JOY_P_.AUX_JOY_CL_trim_P = 200;
      airs_AUX_JOY_P_.AUX_JOY_CL_error_code_P = 15;
      airs_AUX_JOY_R_.AUX_JOY_CL_error_code_R = 15;
      airs_AUX_JOY_R_.AUX_JOY_CL_trim_R = 200;
      airs_AUX_JOY_P_.AUX_JOY_CL_enable_P = 1;
      airs_AUX_JOY_R_.AUX_JOY_CL_enable_R = 1;
      airs_AUX_JOY_P_.AUX_JOY_POS_P = 1000;
      airs_AUX_JOY_R_.AUX_JOY_POS_R = 1000;
    }
    /*
          airs_AUX_JOY_P_.AUX_JOY_POS_P = 0;
    airs_AUX_JOY_P_.AUX_JOY_CL_trim_P = 0;
    airs_AUX_JOY_P_.AUX_JOY_CL_enable_P = 0;
    airs_AUX_JOY_P_.AUX_JOY_Seq_P = 0;
    airs_AUX_JOY_P_.AUX_JOY_CL_error_code_P = 0;
    airs_AUX_JOY_P_.AUX_JOY_CRC_P = 0;
    */

  }
  void airs::dbw_control_system_class::airs_vehicle_fith_wheel(){
    airs_DbwUser_03_.FifthWheelHeightRequest = airs_AT1_YTCC2_02_.FifthWheelHeight;
  }


  void airs::dbw_control_system_class::airs_vehicle_state_msg(){
    airs_vehicle_state_.Steering_Angle_Feedback = std::to_string(airs_steering_control_.airs_steering_deg_fb);
    airs_vehicle_state_.Vehicle_Speed_Feedback = std::to_string(airs_speed_control_.V_actual);
    airs_vehicle_state_.PLC_Ready = '1';
    airs_vehicle_state_.Manual_Mode_Flag = '0';
    airs_vehicle_state_.Brake_Hard_Flag = '0';
    airs_vehicle_state_.Brake_Released_Flag = '0';
    airs_vehicle_state_.Safety_E_Stop_Triggered_Flag = '0';
    
    if(airs_AT1_ETC2_.TransSelectedGear == 0)
    {
      airs_vehicle_state_.Gear_Current_Status_Feedback = '0';
    } else if (airs_AT1_ETC2_.TransSelectedGear > 0)
    {
      airs_vehicle_state_.Gear_Current_Status_Feedback = '1';
    }else if (airs_AT1_ETC2_.TransSelectedGear < 0)
    {
      airs_vehicle_state_.Gear_Current_Status_Feedback = '2';
    }
  }
  void airs::dbw_control_system_class::airs_functional_safety_system(){
    
    FSS_Front_Field = airs_fss_fb_.Safety_SICK_Input_1;
    FSS_Left_Field = airs_fss_fb_.Safety_SICK_Input_2;
    FSS_Right_Field = airs_fss_fb_.Safety_SICK_Input_3;
    FSS_Working_Lane_Field = airs_fss_fb_.Safety_SICK_Input_8;

    if (airs_aios_dashboard_cmd_.FSSSpeedThreshold > (int)airs_speed_control_.V_actual)
    {
      airs_dbw_status_.FSS_Ready = true;
    }else{
      airs_dbw_status_.FSS_Ready = false;
    }

    if (airs_dbw_status_.FSS_Ready == true && airs_fss_fb_.Safety_SICK_Input_4 == 0)
    {
      Functional_Safety_System_Fail = true;
    }else
    {
      Functional_Safety_System_Fail = false;
    }

    if (airs_dbw_status_.FSS_Ready == true && Functional_Safety_System_Fail == false)
    {
      //ROS_INFO("inside FSS_Ready");
      if (FSS_Working_Lane_Field == 0 && airs_speed_control_.V_req <2.0) // low speed or working lane
      {
        airs_dbw_status_.airs_safe_stop = true;

      }
      else if(FSS_Right_Field == 0 && airs_DbwUser_01_.IndicatorLightRequest == 2) // Right
      {
        airs_dbw_status_.airs_safe_stop = true;
      }else if(FSS_Left_Field== 0 && airs_DbwUser_01_.IndicatorLightRequest == 1)  // Left
      {
        airs_dbw_status_.airs_safe_stop = true;
      }else if(FSS_Front_Field == 0 && (airs_DbwUser_01_.IndicatorLightRequest != 1 && airs_DbwUser_01_.IndicatorLightRequest != 2)  && airs_speed_control_.V_req >=2.0) // straight
      {
        airs_dbw_status_.airs_safe_stop = true;
      }else {
        airs_dbw_status_.airs_safe_stop = false;
      }
    }else
    {
      airs_dbw_status_.airs_safe_stop = false;
    }
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dbw_control_system_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate r(20);


  airs::dbw_control_system_class dbw_control_system_class_object(nh, private_nh);

  while (ros::ok())
  {
    dbw_control_system_class_object.publish_dbw_user_msg();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
template < typename Type > std::string to_str (const Type & t)
{
  std::ostringstream os;
  os << t;
  return os.str ();
}