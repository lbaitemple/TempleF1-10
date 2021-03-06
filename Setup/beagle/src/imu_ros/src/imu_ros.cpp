extern "C"
{
#include "rc_usefulincludes.h"
}
extern "C"
{  
#include "roboticscape.h"
}
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>



#include "race/drive_param.h"
#include <unistd.h>

#define SAMPLE_RATE_HZ 100  // main filter and control loop speed
#define MOTOR_ANGLE   3
#define MOTOR_VEL 2

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21


float   time_last_good_ros_command_sec = 0.0;

ros::Publisher              state_publisher;
void monitor_imu();
geometry_msgs::TransformStamped tfs_msg;

rc_mpu_data_t imu_data;


/*******************************************************************************
 * shutdown_signal_handler(int signo)
 *
 * catch Ctrl-C signal and change system state to EXITING
 * all threads should watch for get_state()==EXITING and shut down cleanly
 *******************************************************************************/
void ros_compatible_shutdown_signal_handler(int signo)
{
  if (signo == SIGINT)
    {
      rc_set_state(EXITING);
      ROS_INFO("\nReceived SIGINT Ctrl-C.");
      ros::shutdown();
    }
  else if (signo == SIGTERM)
    {
      rc_set_state(EXITING);
      ROS_INFO("Received SIGTERM.");
      ros::shutdown();
    }
}


void CmdCallback(const race::drive_param::ConstPtr& cmd)
{

//  cmd->velocity
//  cmd->angle
  rc_motor_set(MOTOR_ANGLE, cmd->angle);
  rc_motor_set(MOTOR_VEL, cmd->velocity);
  time_last_good_ros_command_sec = ros::Time::now().toSec();

  return;
}



/*******************************************************************************
 * main()
 * rostopic pub -r 10 /cmd race/drive_param '{velocity: 0.2, angle: 0.2}' 
 *
 * Initialize the filters, IMU, threads, & wait untill shut down
 *******************************************************************************/
int main(int argc, char** argv)  
{

  // Announce this program to the ROS master as a "node" called "edumip_balance_ros_node"
  ros::init(argc, argv, "edumip_balance_ros_node");

  // Start the node resource managers (communication, time, etc)
  ros::start();

  // Broadcast a simple log message
  ROS_INFO("File %s compiled on %s %s.\r\n",__FILE__, __DATE__, __TIME__);

  // Create nodehandle
  ros::NodeHandle edumip_node;

  // Advertise the topics this node will publish
  state_publisher = edumip_node.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Subscriber sub_cmd = edumip_node.subscribe("edumip/cmd", 10, CmdCallback);

  
  rc_set_state(UNINITIALIZED);

  //while(rc_get_state()!=EXITING) rc_usleep(1000);
  // start printf_thread if running from a terminal
  // if it was started as a background process then don't bother
  // 2017-11-19 LLW comment out isatty() call that prevents this program from
  //                being run from a launch file
  // set up IMU configuration
  rc_mpu_config_t imu_config = rc_mpu_default_config();
  imu_config.i2c_bus = I2C_BUS;
  imu_config.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
  imu_config.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
  imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
  imu_config.orient = ORIENTATION_Z_UP;
  imu_config.dmp_fetch_accel_gyro=1;
  imu_config.enable_magnetometer = 1;
  imu_config.dmp_fetch_accel_gyro=1;
  // 2017-01-10 overide the robotics cape default signal handleers with
  // one that is ros compatible
  signal(SIGINT,  ros_compatible_shutdown_signal_handler);	
  signal(SIGTERM, ros_compatible_shutdown_signal_handler);	
	

// start imu
  if(rc_mpu_initialize_dmp(&imu_data, imu_config)){
    ROS_INFO("ERROR: can't talk to IMU, all hope is lost\n");
    rc_led_blink(RC_LED_RED, 5, 5);
    return -1;
  }

  // this should be the last step in initialization 
  // to make sure other setup functions don't interfere
  //rc_set_mpu_interrupt_func(&monitor_imu);
  rc_mpu_set_dmp_callback(&monitor_imu);
  while(rc_get_state()!=EXITING)  rc_usleep(100000);
  // ROS_INFO("Hello compiled \r\n");

  // monitor_imu();
  //  fflush(stdout);
  // rc_usleep(100000);
 // }
	
  // start in the RUNNING state, pressing the puase button will swap to 
  // the PUASED state then back again.
  ROS_INFO("\nHold your MIP upright to begin balancing\n");
  rc_set_state(RUNNING);
	
  // chill until something exits the program
  // while(rc_get_state()!=EXITING){
  //   rc_usleep(10000);
  // }

  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();

  // news
  ROS_INFO("Exiting!");

  // shut down the pthreads
  rc_set_state(EXITING);

  // Stop the ROS node's resources
  ros::shutdown();
	
  // cleanup
  rc_mpu_power_off();
  fflush(stdout);
  return 0;
}

/*******************************************************************************
 * void balance_controller()
 *
 * discrete-time balance controller operated off IMU interrupt
 * Called at SAMPLE_RATE_HZ
 *******************************************************************************/
void monitor_imu()
{
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "imu_link";  // no frame

  // acc data
     rc_usleep(100000);
     ROS_INFO("acce %f,  %f . \r\n",imu_data.accel[0], imu_data.accel[1]);
  
   msg.orientation.x  = imu_data.fused_quat[QUAT_X]; 
   msg.orientation.y = imu_data.fused_quat[QUAT_Y];
   msg.orientation.z = imu_data.fused_quat[QUAT_Z];
   msg.orientation.w = imu_data.fused_quat[QUAT_W];
   msg.linear_acceleration.x = imu_data.accel[0];
   msg.linear_acceleration.y = imu_data.accel[1];
   msg.linear_acceleration.z = imu_data.accel[2];
   msg.angular_velocity.x = imu_data.gyro[0];
   msg.angular_velocity.y = imu_data.gyro[1];
   msg.angular_velocity.z = imu_data.gyro[2];

    msg.angular_velocity_covariance[0] = 0.02;
    msg.angular_velocity_covariance[1] = 0;
    msg.angular_velocity_covariance[2] = 0;
    msg.angular_velocity_covariance[3] = 0;
    msg.angular_velocity_covariance[4] = 0.02;
    msg.angular_velocity_covariance[5] = 0;
    msg.angular_velocity_covariance[6] = 0;
    msg.angular_velocity_covariance[7] = 0;
    msg.angular_velocity_covariance[8] = 0.02;

   msg.linear_acceleration_covariance[0] = 0.04;
    msg.linear_acceleration_covariance[1] = 0;
    msg.linear_acceleration_covariance[2] = 0;
    msg.linear_acceleration_covariance[3] = 0;
    msg.linear_acceleration_covariance[4] = 0.04;
    msg.linear_acceleration_covariance[5] = 0;
    msg.linear_acceleration_covariance[6] = 0;
    msg.linear_acceleration_covariance[7] = 0;
    msg.linear_acceleration_covariance[8] = 0.04;

   msg.orientation_covariance[0] = 0.0025;
    msg.orientation_covariance[1] = 0;
    msg.orientation_covariance[2] = 0;
    msg.orientation_covariance[3] = 0;
    msg.orientation_covariance[4] = 0.0025;
    msg.orientation_covariance[5] = 0;
    msg.orientation_covariance[6] = 0;
    msg.orientation_covariance[7] = 0;
    msg.orientation_covariance[8] = 0.0025;


   state_publisher.publish(msg);
   tf::TransformBroadcaster tfbroadcaster;

    tfs_msg.header.stamp    = ros::Time::now();
    tfs_msg.header.frame_id = "base_link";
    tfs_msg.child_frame_id  = "imu_link";
    tfs_msg.transform.rotation.w = imu_data.fused_quat[QUAT_W];
    tfs_msg.transform.rotation.x = imu_data.fused_quat[QUAT_X];
    tfs_msg.transform.rotation.y = imu_data.fused_quat[QUAT_Y];
    tfs_msg.transform.rotation.z = imu_data.fused_quat[QUAT_Z];

    tfs_msg.transform.translation.x = 0.0;
    tfs_msg.transform.translation.y = 0.0;
    tfs_msg.transform.translation.z = 0.0;

    tfbroadcaster.sendTransform(tfs_msg);


   return;
}

