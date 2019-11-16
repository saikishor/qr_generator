#include <ros/ros.h>
#include <qr_generator/qr_encoder.h>

//void getCvImage(const QRcode *code, cv::Mat &image)
//{
//  image.release();
//  image = cv::Mat(code->width, code->width, CV_8UC1, cv::Scalar(255));
//  for (int i = 0; i < code->width; ++i)
//  {
//    for (int j = 0; j < code->width; ++j)
//    {
//      if (code->data[(i * code->width) + j] & 1)
//      {
//        image.at<uchar>(i, j) = 0;
//      }
//      else
//      {
//        image.at<uchar>(i, j) = 255;
//      }
//    }
//  }
//  cv::resize(image, image, cv::Size(5 * code->width, 5 * code->width), 0, 0, CV_INTER_AREA);
//  cv::copyMakeBorder(image, image, 10, 10, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(255));
//  cv::imwrite("/tmp/boo.png", image);
//}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "qr_generator_ros");
  ros::NodeHandle nh("~");
  QrEncoder qr_encoder(nh);
  ros::spin();
  return 0;
}
