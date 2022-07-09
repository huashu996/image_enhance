#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>//用于ROS图像和OpenCV图像的转换
#include <sensor_msgs/CameraInfo.h>//传感器信息
#include<cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#define _NODE_NAME_ "image_enhancement" //定义节点的名称
//OpenCV的函数都位于cv这一命名空间下，为了调用OpenCV的函数，需要在每个函数前加上cv::，向编译器说明你所调用的函数处于cv命名空间。为了摆脱这种繁琐的工作，可以使用using namespace cv;指令，告诉编译器假设所有函数都位于cv命名空间下。
using namespace cv;
using namespace std;

class ImageEnhancement  //节点参数类
{
private://基本参数
	ros::Publisher enhance_image_pub_;//发布者
	ros::Subscriber image_sub_;//接收者
	std::string image_topic_;//信息类型
	bool is_show_result_;
	bool image_ok_, image_enhanced_;
	int frame_rate_;
	int mode_;
	//0 for 基于直方图均衡化的图像增强 
	//1 for 基于对数Log变换的图像增强
	//2 for 基于伽马变换的图像增强
        //3 for 混合增强
	cv_bridge::CvImagePtr cv_ptr_;
        cv_bridge::CvImagePtr cv_ptr2;
	ros::Timer timer_;
	
public://基本函数
	bool init();
	void loadimage(const sensor_msgs::ImageConstPtr& msg);
	void enhancepub0(const ros::TimerEvent&);
	void enhancepub1(const ros::TimerEvent&);
	void enhancepub2(const ros::TimerEvent&);
        void enhancepub3(const ros::TimerEvent&);
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	ImageEnhancement enhance;//创建一个类的对象
	enhance.init();//调用类的成员函数
	ros::spin();//循环
	return 0;
}



bool ImageEnhancement::init()//定义ImageEnhancement类的成员函数
{
	ros::NodeHandle nh, nh_private("~");//开启节点对象nh
        //节点的参数服务器，写在launch文件中的可以随时修改的参数
	nh_private.param<std::string>("image_topic", image_topic_, "");
	nh_private.param<int>("frame_rate",frame_rate_,30);
	nh_private.param<int>("mode",mode_,0);

	image_ok_ = false;//一个关闭标志
	image_enhanced_ = false;
	enhance_image_pub_ = nh.advertise<sensor_msgs::Image>("/image_enhancement", 1);//定义发布者

	image_sub_ = nh.subscribe(image_topic_, 1, &ImageEnhancement::loadimage, this);//定义订阅image_topic_话题的消息，回调函数，以指针形式存储数据  回调函数在一个类中，后面加上this
	
	if(mode_ == 0)
		timer_ = nh.createTimer(ros::Duration(0.01), &ImageEnhancement::enhancepub0, this);
	else if(mode_ == 1)
		timer_ = nh.createTimer(ros::Duration(0.01), &ImageEnhancement::enhancepub1, this);
	else if(mode_ == 2)
		timer_ = nh.createTimer(ros::Duration(0.01), &ImageEnhancement::enhancepub2, this);
	else
		ROS_ERROR("none mode is starting!");//报错打印指令
	ROS_INFO("image_enhancement initial ok.");//提示打印指令
}

void ImageEnhancement::loadimage(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("[%s]: getting image!",_NODE_NAME_);
	cv_bridge::CvImagePtr cv;//在CVbridge类中创建一个对象cv
	cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//转化储存图像
	cv_ptr_ = cv;
	
	image_ok_ = true;
	image_enhanced_ = false;
}

void ImageEnhancement::enhancepub0(const ros::TimerEvent&)
{
	if (image_ok_ == false)
	{
		ROS_ERROR("[%s]: no image input!",_NODE_NAME_);
		return;
	}
	else if (image_enhanced_ == true)
	{
		ROS_INFO("[%s]: waiting for new image!",_NODE_NAME_);
		return;
	}
	else
		ROS_INFO("[%s]: image enhancement start! mode:0",_NODE_NAME_);
		
	static int width, height;
	width = cv_ptr_->image.cols;
	height = cv_ptr_->image.rows;
        //创建一个enhanced_image对象
	cv::Mat enhanced_image(height, width, CV_8UC3);//cv::Mat是OpenCV2和OpenCV3中基本的数据类型长宽和文件格式
	enhanced_image.setTo(0);//初始化增强后的图
	cv_ptr_->image.copyTo(enhanced_image(Rect(0, 0, width, height)));

	cv::Mat imageRGB[3];
	split(enhanced_image, imageRGB);
	for (int i=0; i<3; ++i)
    {
        equalizeHist(imageRGB[i], imageRGB[i]);
    }
    merge(imageRGB, 3, enhanced_image);
    //定义发布消息的内容
    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", enhanced_image).toImageMsg();
	imageMsg->header.frame_id = std::string("enhance image");
	imageMsg->header.stamp = ros::Time::now();
	enhance_image_pub_.publish(imageMsg);//发布消息
	ROS_INFO("[%s]: image enhancement done!",_NODE_NAME_);
	image_enhanced_ = true;
}

void ImageEnhancement::enhancepub1(const ros::TimerEvent&)
{
	if (image_ok_ == false)
	{
		ROS_ERROR("[%s]: no image input!",_NODE_NAME_);
		return;
	}
	else if (image_enhanced_ == true)
	{
		ROS_INFO("[%s]: waiting for new image!",_NODE_NAME_);
		return;
	}
	else
		ROS_INFO("[%s]: image enhancement start! mode:0",_NODE_NAME_);
		
	static int width, height;
	width = cv_ptr_->image.cols;
	height = cv_ptr_->image.rows;
	cv::Mat enhanced_image(height, width, CV_8UC3);
	enhanced_image.setTo(0);
	cv_ptr_->image.copyTo(enhanced_image(Rect(0, 0, width, height)));
	
	cv::Mat srcLog(enhanced_image.size(), CV_32FC3);
	
	for (int i=0; i<enhanced_image.rows; ++i)
    {
        for (int j=0; j<enhanced_image.cols; ++j)
        {
            srcLog.at<Vec3f>(i, j)[0] = log(1 + enhanced_image.at<Vec3b>(i, j)[0]);
            srcLog.at<Vec3f>(i, j)[1] = log(1 + enhanced_image.at<Vec3b>(i, j)[1]);
            srcLog.at<Vec3f>(i, j)[2] = log(1 + enhanced_image.at<Vec3b>(i, j)[2]);
        }
    }
        //归一化
	normalize(srcLog, srcLog, 0, 255, NORM_MINMAX);
        //
	convertScaleAbs(srcLog, srcLog);
	
    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcLog).toImageMsg();
	imageMsg->header.frame_id = std::string("enhance image");
	imageMsg->header.stamp = ros::Time::now();
	enhance_image_pub_.publish(imageMsg);
	ROS_INFO("[%s]: image enhancement done!",_NODE_NAME_);
	image_enhanced_ = true;
}

void ImageEnhancement::enhancepub2(const ros::TimerEvent&)
{
	if (image_ok_ == false)
	{
		ROS_ERROR("[%s]: no image input!",_NODE_NAME_);
		return;
	}
	else if (image_enhanced_ == true)
	{
		ROS_INFO("[%s]: waiting for new image!",_NODE_NAME_);
		return;
	}
	else
		ROS_INFO("[%s]: image enhancement start! mode:0",_NODE_NAME_);
		
	static int width, height;
	width = cv_ptr_->image.cols;
	height = cv_ptr_->image.rows;
	cv::Mat enhanced_image(height, width, CV_8UC3);
	enhanced_image.setTo(0);
	cv_ptr_->image.copyTo(enhanced_image(Rect(0, 0, width, height)));
	
	
	float pixels[256];
	for (int i=0; i<256; ++i)
    {
         pixels[i] = powf(i,2);
    }

    cv::Mat srcLog(enhanced_image.size(), CV_32FC3);
    for (int i=0; i<enhanced_image.rows; ++i)
    {
        for (int j=0; j<enhanced_image.cols; ++j)
        {
			srcLog.at<Vec3f>(i, j)[0] = pixels[enhanced_image.at<Vec3b>(i, j)[0]];
			srcLog.at<Vec3f>(i, j)[1] = pixels[enhanced_image.at<Vec3b>(i, j)[1]];
			srcLog.at<Vec3f>(i, j)[2] = pixels[enhanced_image.at<Vec3b>(i, j)[2]];        
        }
    }
    normalize(srcLog, srcLog, 0, 255, NORM_MINMAX);
    //Mat img;
    //resize(srcLog, img, Size(640, 360), 0, 0, INTER_CUBIC);
    convertScaleAbs(srcLog, srcLog, 4, 30); 
	resize(srcLog, srcLog,Size(1920,1440),INTER_LINEAR);
    medianBlur(srcLog,srcLog, 3);
 

    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",srcLog).toImageMsg();
	imageMsg->header.frame_id = std::string("enhance image");
	imageMsg->header.stamp = ros::Time::now();
	enhance_image_pub_.publish(imageMsg);
	image_enhanced_ = true;
	ROS_INFO("[%s]: image enhancement done!",_NODE_NAME_);
}











