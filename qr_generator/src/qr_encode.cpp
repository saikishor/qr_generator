#include <qr_generator/qr_encoder.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <cerrno>

QrEncoder::QrEncoder(ros::NodeHandle &nh) : nh_(nh), casesensitive_(true)
{
  qr_encode_srv_ = nh_.advertiseService("generate_qr", &QrEncoder::encodeQR, this);
  ros::param::param<int>("~border_size", border_size_, 10);
  ros::param::param<int>("~qr_size", qr_size_, 220);
  ros::param::param<int>("~qr_version", qr_version_, 6);
  if (qr_version_ <= 0)
  {
    throw ros::Exception("QR version cannot be less than or equal to zero!. Parsed : " +
                         std::to_string(qr_version_));
  }
  if (qr_version_ > 40)
  {
    throw ros::Exception("QR version cannot be greater than 40!. Parsed : " +
                         std::to_string(qr_version_));
  }
  ros::param::param<bool>("~casesensitive", casesensitive_, true);
  ros::param::param<std::string>("~save_to", save_to_, "");
  std::string error_type;
  ros::param::param<std::string>("~error_type", error_type, "MEDIUM");
  if (boost::iequals(error_type, "LOW"))
  {
    error_type_ = QR_ECLEVEL_L;
  }
  else if (boost::iequals(error_type, "MEDIUM"))
  {
    error_type_ = QR_ECLEVEL_M;
  }
  else if (boost::iequals(error_type, "QUARTILE"))
  {
    error_type_ = QR_ECLEVEL_Q;
  }
  else if (boost::iequals(error_type, "HIGH"))
  {
    error_type_ = QR_ECLEVEL_H;
  }
  else
  {
    throw ros::Exception("Parsed Unkown error type : " + error_type +
                         "!. Known error types are : LOW, MEDIUM, QUARTILE, HIGH");
  }
  ROS_INFO("QR generation service is active!");
}

QrEncoder::~QrEncoder()
{
}

QRencodeMode QrEncoder::getQREncodeMode(const std::string &data) const
{
  for (const QRencodeMode &qr_mode : { QR_MODE_NUM, QR_MODE_AN, QR_MODE_8 })
  {
    int data_info =
        QRinput_check(qr_mode, sizeof(data.c_str()), (unsigned char *)data.c_str());
    if (data_info == 0)
    {
      return qr_mode;
    }
  }
}

bool QrEncoder::encodeQR(qr_generator_msgs::GetQR::Request &req,
                         qr_generator_msgs::GetQR::Response &res)
{
  int case_sensitive = casesensitive_ ? 1 : 0;
  int qr_version = qr_version_;
  if (req.qr_version > 0)
  {
    if (req.qr_version > 40)
    {
      res.status = false;
      res.message = "qr_version cannot be greater than 40!";
      return false;
    }
    qr_version = req.qr_version;
  }
  QRecLevel type;
  if (req.error_correction_type == qr_generator_msgs::GetQR::Request::ECC_DEFAULT)
  {
    type = error_type_;
  }
  else
  {
    if (req.error_correction_type > qr_generator_msgs::GetQR::Request::ECC_HIGH)
    {
      res.status = false;
      res.message = "Invalid error correction type info!";
      return true;
    }
    type = static_cast<QRecLevel>(req.error_correction_type - 1);
  }
  qrcode_ = QRcode_encodeString(req.data.c_str(), qr_version, type, QR_MODE_8, case_sensitive);
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
    }
  }
  int qr_size = qr_size_ - (2 * border_size_);
  if (qr_size > image.rows)
  {
    cv::resize(image, image, cv::Size(qr_size, qr_size), 0, 0, CV_INTER_NN);
  }
  else if (qr_size < image.rows)
  {
    ROS_WARN_STREAM("Cannot resize the QR, as the parsed size : "
                    << qr_size_ << " is less than the standard QR size : " << image.rows);
  }
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
