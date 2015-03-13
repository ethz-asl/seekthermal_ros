#include <iostream>

#include <seekthermal_ros/seekthermal_ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <ros/package.h>

#define DEG2RAD 0.01745329

namespace seekthermal_ros {

SeekthermalRos::SeekthermalRos(ros::NodeHandle nh): nh_(nh), it_(nh)
{
  //Get parameters from configuration file
  nh.getParam("thermal_image_topic_name", thermal_image_topic_name_);
  nh.getParam("device_adress", device_adress_);
  nh.getParam("color_mode", color_mode_);
  nh.getParam("interpolate_dead_pixels", use_inpaint_);
  nh.getParam("calibrate_dead_pixels", calibrate_dead_pixels_);
  nh.getParam("mean_compensation", mean_compensation_);
  nh.getParam("calibrate_mean_compensation", calibrate_mean_compensation_);
  nh.getParam("denoise", denoise_);
  nh.getParam("show_debug_images", show_debug_images_);

  //Create a publisher for the raw thermal image
  thermal_image_publisher_ = it_.advertiseCamera(thermal_image_topic_name_, 1);

  // load the camera info
  nh_.getParam("camera_frame_id", camera_frame_id_);
  nh_.getParam("camera_name", camera_name_);
  nh_.getParam("camera_info_url", camera_info_url_);
  //nh_.getParam("image_width", image_width_);
  //nh_.getParam("image_height", image_height_);
  cinfo_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));
  // check for default camera info
  if (camera_info_url_.size() == 0)
  {
    cinfo_->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = camera_frame_id_;
    //camera_info.width = image_width_;
    //camera_info.height = image_height_;
    cinfo_->setCameraInfo(camera_info);
  }

  //Find all connected devices
  std::list<Pointer<Device>> devices = discoverDevices();

  SeekThermal::Usb::Context context;
  Pointer<Interface> interface;

  //choose first device in list if no device was specified in the configuration file
  if (device_adress_.empty())
  {
    if (!devices.empty())
    {
      for (std::list<Pointer<Device> >::const_iterator it = devices.begin(); it != devices.end(); ++it)
      {
        ROS_INFO_STREAM("Found " << (*it)->getInterface()->getName() << " " << (*it)->getInterface()->getAddress());
      }

      ROS_INFO_STREAM("Trying to open first device: " << devices.front()->getInterface()->getAddress());
      interface = context.getInterface(devices.front()->getInterface()->getAddress());
    }
    else
    {
      ROS_ERROR_STREAM("No devices found and no device specified in config file");
      ros::shutdown();
    }
  }
  else
  {
    ROS_INFO_STREAM("Trying to open device at " << device_adress_);
    interface = context.getInterface(device_adress_);
  }

  device_ = interface->discoverDevice();

  if (!device_.isNull()) {
    interface->setTimeout(1);
    device_->setInterface(interface);
    device_->connect();
    device_->initialize();
    ROS_INFO_STREAM("Device initialized!");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to open device");
    ros::shutdown();
  }

  if (calibrate_mean_compensation_)
  {
    state_ = CALIBRATE_MEAN;
  }
  else
  {
    state_ = LOAD_MEAN;
  }

  package_path_ = ros::package::getPath("seekthermal_ros");

  boost::thread* publishing_thread = new boost::thread(boost::bind(&SeekthermalRos::publishingThermalImages, this));

  captureThermalImages(device_);
}

SeekthermalRos::~SeekthermalRos()
{
  device_->disconnect();
}

void SeekthermalRos::captureThermalImages(const Pointer<Device>& device)
{
  while(ros::ok())
  {
    if (thermal_image_publisher_.getNumSubscribers()>0)
    {
      Pointer<Frame> frame = new Frame();
      device->capture(*frame);

      boost::mutex::scoped_lock lock(mutex_);
      frame_queue_.push(frame);

      if (frame_queue_.size()>10)
      {
        frame_queue_.pop();
      }
      lock.unlock();
      condition_variable_.notify_one();
    }
    else
      ros::Duration(0.1).sleep();
  }
}

void SeekthermalRos::publishingThermalImages()
{
  static int seq_counter = 0;
  static int dead_pixel_counter = 0;
  static size_t frameId = 0;

  std::vector<Frame> frame_vector;

  static Frame meanFrame;
  static Frame varianceFrame;

  while(ros::ok())
  {
    boost::mutex::scoped_lock lock(mutex_);
    while(frame_queue_.empty())
    {
      condition_variable_.wait(lock);
    }

    Pointer<Frame> frame = frame_queue_.front();
    frame_queue_.pop();

    if (frame->getType() == Frame::typeNormal)
    {
      //ROS_INFO_STREAM("Normal Frame");
      size_t width = frame->getWidth();
      size_t height = frame->getHeight();

      Frame frame_t = *frame;

      //subtract most recent calibration frame
      if (!last_calibration_frame_.isEmpty())
        *frame -= last_calibration_frame_;

      static Frame mean_comp;

      frame->normalize(-1600,200);

      if (show_debug_images_)
      {
        Mat cvImage_normalized = Mat(frame->getHeight(), frame->getWidth(), CV_8UC3);
        for (size_t x = 0; x < frame->getWidth(); ++x)
          for (size_t y = 0; y < frame->getHeight(); ++y) {
            float value = (*frame)(x, y)*255.0;
            cvImage_normalized.at<Vec3b>(y,x)[0] = value;
            cvImage_normalized.at<Vec3b>(y,x)[1] = value;
            cvImage_normalized.at<Vec3b>(y,x)[2] = value;
          }

        cv::imshow("normalized", cvImage_normalized);
        cv::waitKey(10);
      }


      switch (state_)
      {
        case CALIBRATE_MEAN:
          //collect frames for calibrating the mean
          if (frame_vector.size() < 20)
          {
            ROS_INFO_STREAM("mean");
            frame_vector.push_back(*frame);
          }
          else
          {
            mean_comp = Frame(frame->getWidth(), frame->getHeight(), Frame::typeNormal);
            for(std::vector<int>::size_type i = 0; i != frame_vector.size(); i++)
            {
              mean_comp += frame_vector[i];
            }
            mean_comp *= 1.0/(float)frame_vector.size();
            float overall_mean = 0;
            for (size_t x = 0; x < width; ++x)
              for (size_t y = 0; y < height; ++y) {
                float value = (mean_comp)(x, y);
                overall_mean += value;
              }
            overall_mean /= width*height;

            mean_comp -= overall_mean;

            mean_compensation_image_ = Mat(height, width, CV_8UC1);
            for (size_t x = 0; x < width; ++x)
              for (size_t y = 0; y < height; ++y) {
                float value = (mean_comp)(x, y)*255;
                mean_compensation_image_.at<uchar>(y,x) = value;
              }

            std::vector<int> image_pref;
            image_pref.push_back(CV_IMWRITE_PNG_COMPRESSION);
            image_pref.push_back(0);

            cv::imwrite(package_path_ + "/config/mean_compensation.png", mean_compensation_image_, image_pref);

            frame_vector.clear();
            if (calibrate_dead_pixels_)
            {
              state_ = CALIBRATE_DEAD_PIXEL;
            }
            else
            {
              state_ = LOAD_DEAD_PIXEL;
            }
          }
          break;


        //Load an already existing mean compensation image
        case LOAD_MEAN:
          mean_compensation_image_ = imread(package_path_ + "/config/mean_compensation.png", CV_LOAD_IMAGE_GRAYSCALE);

          if(! mean_compensation_image_.data )                              // Check for invalid input
          {
            ROS_ERROR_STREAM("No mean calibration file found. Forced calibration.");
            state_ = CALIBRATE_MEAN;
          }
          else if (calibrate_dead_pixels_)
          {
            state_ = CALIBRATE_DEAD_PIXEL;
          }
          else
          {
            state_ = LOAD_DEAD_PIXEL;
          }
          break;

        // find pixels that are dead by calculating the variance of each pixel
        case CALIBRATE_DEAD_PIXEL:
          if (frame_vector.size() < 20)
          {
            ROS_INFO_STREAM("dead pixel");
            frame_vector.push_back(*frame);
          }
          else
          {
            Frame mean = Frame(frame->getWidth(), frame->getHeight(), Frame::typeNormal);
            for(std::vector<int>::size_type i = 0; i != frame_vector.size(); i++)
            {
              mean += frame_vector[i];
            }
            mean *= 1.0/(float)frame_vector.size();

            Frame temp = Frame(frame->getWidth(), frame->getHeight(), Frame::typeNormal);
            for(std::vector<int>::size_type i = 0; i != frame_vector.size(); i++)
            {
              temp += (mean-frame_vector[i])*(mean-frame_vector[i]);
            }
            Frame variance = temp*(1.0/(float)frame_vector.size());

            dead_pixel_counter++;

            Mat variance_mat = Mat(height, width, CV_8UC1);
            for (size_t x = 0; x < width; ++x)
              for (size_t y = 0; y < height; ++y) {
                float value = (variance)(x, y)*255.0;
                variance_mat.at<uchar>(y,x) = value;
              }

            inpaint_mask_ = variance_mat<0.00001;

            std::vector<int> image_pref;
            image_pref.push_back(CV_IMWRITE_PNG_COMPRESSION);
            image_pref.push_back(0);

            cv::imwrite(package_path_ + "/config/dead_pixel.png", inpaint_mask_, image_pref);

            frame_vector.clear();

            state_ = RUN;
          }

          break;

        //Load calibration image for dead pixels
        case LOAD_DEAD_PIXEL:
          inpaint_mask_ = imread(package_path_ + "/config/dead_pixel.png", CV_LOAD_IMAGE_GRAYSCALE);

          if(! inpaint_mask_.data )                              // Check for invalid input
          {
            ROS_ERROR_STREAM("No dead pixel calibration file found. Forced calibration.");
            state_ = CALIBRATE_DEAD_PIXEL;
          }
          else
          {
            state_ = RUN;
          }
          break;

        case RUN:
          Mat cvImage = Mat(frame->getHeight(), frame->getWidth(), CV_8UC1);
          for (size_t x = 0; x < frame->getWidth(); ++x)
            for (size_t y = 0; y < frame->getHeight(); ++y) {
              float value = (*frame)(x, y)*255.0;
              cvImage.at<uchar>(y,x) = value;
            }

          if (mean_compensation_)
          {
            for (size_t x = 0; x < cvImage.cols; ++x)
              for (size_t y = 0; y < cvImage.rows; ++y)
              {
                float value = cvImage.at<uchar>(y,x) - mean_compensation_image_.at<uchar>(y,x);
                //if (value < 255)
                  cvImage.at<uchar>(y,x) = value;
                //else
                //  cvImage.at<uchar>(y,x) = 255;
              }

            cv::imshow("mean_comp", cvImage);
            cv::waitKey(10);
          }

          cv::Mat cvImage_inpainted = Mat(height, width, CV_8UC1);

          if (use_inpaint_)
          {
            cv::inpaint(cvImage, inpaint_mask_, cvImage_inpainted, 1, cv::INPAINT_NS);
            cvImage_inpainted.copyTo(cvImage);
            cv::imshow("inpainted frame", cvImage_inpainted);
            cv::waitKey(10);
          }

          cv::Mat cvImage_denoised = Mat(height, width, CV_8UC1);

          if (denoise_)
          {
            cv::fastNlMeansDenoising(cvImage, cvImage_denoised, 12, 5, 25);

            cvImage_denoised.copyTo(cvImage);

            cv::imshow("denoised frame", cvImage_denoised);
            cv::waitKey(10);
          }

          cv::Mat cvImage_out = Mat(height, width, CV_8UC3);
          if (color_mode_ == "color")
          {
            cvImage_out = convertFromGrayToColor(cvImage);
          }
          else
          {
            cvImage.convertTo(cvImage_out, CV_8UC3);
          }

          std_msgs::Header header;
          header.seq = seq_counter;
          header.frame_id = camera_frame_id_;
          //header.stamp = ros::Time(frame.getTimestamp());
          cv_bridge::CvImage *cv_ptr = new cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cvImage_out);

          // grab the camera info
          sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
          ci->header.seq = seq_counter++;
          ci->header.frame_id = camera_frame_id_;
          //ci->header.stamp = frame.getTimestamp();
          ci->height = frame->getHeight();
          ci->width = frame->getWidth();

          // publish the image
          thermal_image_publisher_.publish(*cv_ptr->toImageMsg(), *ci);
      }


    }
    else if (frame->getType() == Frame::typeCalibration)
    {
      //ROS_INFO_STREAM("Calibration Frame");
      last_calibration_frame_ = *frame;
    }
  }
}

Mat SeekthermalRos::convertFromGrayToColor(Mat &image)
{
  Mat cvImage_colored = Mat(image.rows, image.cols, CV_8UC3);
  for (size_t x = 0; x < image.cols; ++x)
    for (size_t y = 0; y < image.rows; ++y) {
      float value = image.at<uchar>(y,x);
      float r = (sin((value / 255.0 * 360.0 - 120.0 > 0 ? value / 255.0 * 360.0 - 120.0 : 0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
      float g = (sin((value / 255.0 * 360.0 + 60.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
      float b = (sin((value / 255.0 * 360.0 + 140.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
      cvImage_colored.at<Vec3b>(y,x)[0] = r;
      cvImage_colored.at<Vec3b>(y,x)[1] = g;
      cvImage_colored.at<Vec3b>(y,x)[2] = b;
    }
  return cvImage_colored;
}

} /* namespace */
