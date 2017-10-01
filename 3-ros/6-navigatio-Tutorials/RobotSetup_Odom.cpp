#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  //默认机器人的起始位置是odom参考系下的0点
  
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
// 设置机器人的默认前进速度，让机器人的base_link参考系在odom参考系下以x轴方向0.1m/s，
// Y轴速度-0.1m/s，角速度0.1rad/s的状态移动，这种状态下，可以让机器人保持圆周运动。

  double vx = 0.1;   //x方向速度
  double vy = -0.1;  //y方向速度
  double vth = 0.1;  //theta角速度

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;  //x方向位移
    y += delta_y;  //y方向位移
    th += delta_th;//方向角

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans; //带时间戳的变换。tf
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;				  //带时间戳的里程计数据。odem
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // 填充机器人的位置、速度，然后发布消息。注意，我们发布的是机器人本体的信息，
    //所以参考系需要填"base_link"。
    
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}

/*
rostopic echo /odom:

header: 
  seq: 62
  stamp: 
    secs: 1506742934
    nsecs: 258602691
  frame_id: odom
child_frame_id: base_link
pose: 
  pose: 
    position: 
      x: -0.0752243181017
      y: 0.0904377101602
      z: 0.0
    orientation: 
      x: -0.0
      y: 0.0
      z: 0.0415725552495
      w: -0.999135487634
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.1
      y: -0.1
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.1
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---


*/

/*
ref:
http://www.guyuehome.com/332
http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
*/