#include <qr_generator/qr_encoder.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <boost/filesystem.hpp>
#include <cerrno>

QrEncoder::QrEncoder(ros::NodeHandle &nh) : nh_(nh)
{
  qr_encode_srv_ = nh_.advertiseService("generate_qr", &QrEncoder::encodeQR, this);
  ros::param::param<int>("~border_size", border_size_, 10);
  ros::param::param<int>("~qr_size", qr_size_, 200);
  ros::param::param<std::string>("~save_to", save_to_, "/tmp/foo/foo.png");
  ROS_INFO("QR generation service is active!");
}

QrEncoder::~QrEncoder()
{
}

bool QrEncoder::encodeQR(qr_generator_msgs::GetQR::Request &req,
                         qr_generator_msgs::GetQR::Response &res)
{
  qrcode_ = QRcode_encodeString(req.data.c_str(), 4, QR_ECLEVEL_H, QR_MODE_8, 1);
  res.status = true;
  if (qrcode_ == NULL)
  {
    res.status = false;
    if (errno == EINVAL)
    {
      res.message = "Parsed invalid input object!";
    }
    else if (errno == ENOMEM)
    {
      res.message = "Unable to allocate memory!";
    }
    else if (errno == ERANGE)
    {
      res.message = "Parsed input data is too large!";
    }
    else
    {
      res.message = std::strerror(errno);
    }
    ROS_ERROR_STREAM(res.message);
    return true;
  }
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  try
  {
    this->encodeToCvImage(qrcode_, cv_image.image);
    QRcode_free(qrcode_);
  }
  catch (...)
  {
    res.status = false;
    res.message = "Error while encoding data into the CV image!";
    return true;
  }
  res.image = *cv_image.toImageMsg();
  return true;
}

void QrEncoder::encodeToCvImage(const QRcode *qrcode, cv::Mat &image) const
{
  image.release();
  image = cv::Mat(qrcode->width, qrcode->width, CV_8UC1, cv::Scalar(255));
  for (int i = 0; i < qrcode->width; ++i)
  {
    for (int j = 0; j < qrcode->width; ++j)
    {
      if (qrcode->data[(i * qrcode->width) + j] & 1)
      {
        image.at<uchar>(i, j) = 0;
      }
      else
      {
        image.at<uchar>(i, j) = 255;
      }
    }
  }
  cv::resize(image, image, cv::Size(qr_size_, qr_size_), 0, 0, CV_INTER_NN);
  cv::copyMakeBorder(image, image, border_size_, border_size_, border_size_, border_size_,
                     cv::BORDER_CONSTANT, cv::Scalar(255));
  if (!save_to_.empty())
  {
    boost::filesystem::path directory =
        boost::filesystem::path(save_to_).parent_path().string();
    if (!boost::filesystem::exists(directory))
    {
      boost::system::error_code err;
      if (!boost::filesystem::create_directories(directory, err))
      {
        ROS_ERROR_STREAM("Error creating folders to save the image : " << err.message());
      }
    }
    cv::imwrite(save_to_, image);
  }
}
