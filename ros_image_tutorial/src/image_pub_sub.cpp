#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image Window";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	public:

		ImageConverter():it_(nh_) {
			image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCallback, this);
			image_pub_ = it_.advertise("/webcam_image", 10);
			cv::namedWindow(OPENCV_WINDOW);
		}

		~ImageConverter() {
			cv::destroyWindow(OPENCV_WINDOW);
		}

		void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

			cv_bridge::CvImagePtr cv_ptr;

			try {
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			} catch(cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

			if (cv_ptr->image.rows > 100 && cv_ptr->image.cols > 100)
			  cv::circle(cv_ptr->image, cv::Point(100, 100), 30, CV_RGB(255,0,0));

			cv::imshow(OPENCV_WINDOW, cv_ptr->image);
			cv::waitKey(3);

			image_pub_.publish(cv_ptr->toImageMsg());

		}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_image_tutorial");
  ImageConverter ic; 
  ros::spin();
  return 0;
}







