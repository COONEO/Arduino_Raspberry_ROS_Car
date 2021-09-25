#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static Mat result_img;
std::string video_device_name_;

int redThre ;
int saturationTh;

static const std::string OPENCV_WINDOW_NAME = "ROS Image window";
Mat CheckColor( Mat &inImg) ;
void DrawFire( Mat &inputImg, Mat foreImg) ;

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "fire_detect_node");
  ros::NodeHandle nh_;

  nh_.param("video_device", video_device_name_, std::string("/dev/video0"));
  nh_.param("redThre", redThre, 49);
  nh_.param("saturationTh", saturationTh, 7);

  image_transport::ImageTransport it(nh_);    
  image_transport::Publisher image_pub_ = it.advertise("fire_detect_image/Compress", 1);

  Mat current_frame;
  Mat result_frame;

  VideoCapture cap(video_device_name_);
  if (cap.isOpened()== false)
    ROS_WARN("Camera open failed!!!");

  ros::Rate r(20);
  while(ros::ok())
  {
    cap >> current_frame ;
    //imshow("current frame",current_frame);
    CheckColor(current_frame);  
    sensor_msgs::ImagePtr result_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_img).toImageMsg();
    image_pub_.publish(result_msg);
    waitKey(2);
    ros::spinOnce();
  }
  return 0;
}

Mat CheckColor( Mat &inImg)  
{  
    Mat fireImg;  
    fireImg.create(inImg.size(),CV_8UC1);  
      
    Mat multiRGB[3];  
    int a = inImg.channels();  
    split(inImg,multiRGB);    //将图片拆分成R,G,B,三通道的颜色  
  
    for (int i = 0; i < inImg.rows; i ++)  
    {  
        for (int j = 0; j < inImg.cols; j ++)  
        {  
            float B,G,R;  
            B = multiRGB[0].at<uchar>(i,j);    //每个像素的R,G,B值  
            G = multiRGB[1].at<uchar>(i,j);  
            R = multiRGB[2].at<uchar>(i,j);     
  
            int maxValue = max(max(B,G),R);  
            int minValue = min(min(B,G),R);  
  
            double S = (1-3.0*minValue/(R+G+B));  
  
            if(R > redThre && R >= G && G >= B && S >0.20 && S >((255 - R) * saturationTh/redThre))  
            {  
                fireImg.at<uchar>(i,j) = 255;  
            }  
            else  
            {  
                fireImg.at<uchar>(i,j) = 0;  
            }  
        }  
    } 
    dilate(fireImg,fireImg, Mat(5,5,CV_8UC1));    
    DrawFire(inImg,fireImg);   
    return fireImg;  
}

void DrawFire( Mat &inputImg, Mat foreImg) 
{  
    vector<vector< Point>> contours_set;//保存轮廓提取后的点集及拓扑关系   

    findContours(foreImg,contours_set,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);     
  
     Mat result0;
     Scalar holeColor;
     Scalar externalColor;
  
    vector<vector< Point> >::iterator iter = contours_set.begin() ;  
    for(; iter!= contours_set.end(); )  
    {  
         Rect rect = boundingRect(*iter );  
        float radius;    
         Point2f center;    
        minEnclosingCircle(*iter,center,radius);    
          
        if (rect.area()> 0)        
        {  
            rectangle(inputImg,rect, Scalar(0,255,0));     
            ++ iter;  
        }  
        else  
        {  
            iter = contours_set.erase(iter);  
        }  
    }  
    //imshow(OPENCV_WINDOW_NAME, inputImg);

    result_img = inputImg.clone();  
}  
 
 