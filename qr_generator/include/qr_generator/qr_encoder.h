#ifndef QR_ENCODER_H
#define QR_ENCODER_H

/** \author Sai Kishor Kothakota **/
#include <ros/ros.h>
#include <qrencode.h>
#include <cv_bridge/cv_bridge.h>
#include <qr_generator_msgs/GetQR.h>

class QrEncoder
{
public:
  QrEncoder(ros::NodeHandle &nh);

  ~QrEncoder();

private:
  QRencodeMode getQREncodeMode(const std::string &data) const;
  bool encodeQR(qr_generator_msgs::GetQR::Request &req, qr_generator_msgs::GetQR::Response &res);
  void encodeToCvImage(const QRcode *qrcode, cv::Mat &image) const;

  ros::NodeHandle nh_;
  ros::ServiceServer qr_encode_srv_;
  QRcode *qrcode_;
  QRecLevel error_type_;
  int border_size_, qr_size_, qr_version_;
  bool casesensitive_;
  std::string save_to_;
};


#endif  // QR_ENCODER_H
