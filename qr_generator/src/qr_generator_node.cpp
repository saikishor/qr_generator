#include <ros/ros.h>
#include <qr_generator/qr_encoder.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "qr_generator_ros");
  ros::NodeHandle nh("~");
  try
  {
    QrEncoder qr_encoder(nh);
    ros::spin();
  }
  catch (const ros::Exception &e)
  {
    ROS_ERROR(e.what());
    return -1;
  }
  return 0;
}
