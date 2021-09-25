#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

unsigned char tag=0x00;

bool DecodeIMUData(double result[], unsigned char chrTemp[]);

int main(int argc, char** argv)
{
    serial::Serial ser;
    std::string port;
    std::string imu_frame_id;
    int baudrate;

    unsigned char chrBuffer[1024];
    unsigned char chrTemp[1024];
    double Decode_data[4] = {0,0,0,1};
    unsigned int remove_num = 0;
    size_t num = 0;

    ros::init(argc, argv, "imu_901_node");

    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
    private_node_handle.param<int>("baudrate", baudrate, 9600);
    
    ros::NodeHandle nh; 
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 200);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 200);
    geometry_msgs::Quaternion Qua;
    sensor_msgs::Imu imu;
    sensor_msgs::MagneticField mag;

    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        ROS_INFO("Serial port open Successful");
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << " ");
        return -1;
    }

    ros::Rate r(200);      // 200 hz
    while(ros::ok())
    {
        num = ser.available();
        if(num != 0)
        {
            num = ser.read(chrBuffer,num);
            if (num>0)
            {
                while (num >= 11)
                {
                    memcpy(chrTemp,chrBuffer,num);
                    if ( ! ( (chrTemp[0] == 0x55) && ( (chrTemp[1] == 0x51) ||(chrTemp[1] == 0x52) ||(chrTemp[1] == 0x53) || (chrTemp[1] == 0x54) || (chrTemp[1] == 0x59) ) ) )
                    {
                        for (int i = 1; i < num; i++)
                            chrBuffer[i - 1] = chrBuffer[i];
                        remove_num += 1;
                        num-=1;
                        continue;
                    }
                    //std::cout<<"remove Tmes: "<<remove_num<<std::endl;
                    remove_num = 0;

                    if(DecodeIMUData(Decode_data, chrTemp))
                    {
                        imu.header.stamp = ros::Time::now();
                        imu.header.frame_id = imu_frame_id;
                    
                        if(tag==0x51)        //0x51 三轴加速度
                        {
                            imu.linear_acceleration.x=Decode_data[0];
                            imu.linear_acceleration.y=Decode_data[1];
                            imu.linear_acceleration.z=Decode_data[2];
                        }
                        else if(tag==0x52)     //0x52 三轴角速度
                        {
                            imu.angular_velocity.x=Decode_data[0];
                            imu.angular_velocity.y=Decode_data[1];
                            imu.angular_velocity.z=Decode_data[2];
                        }
                        else if(tag==0x53)     //0x53 三轴旋转角度
                        {
                            //std::cout<<"Test Yall angle in static status"<<Decode_data[2]<<std::endl;
                            Qua=tf::createQuaternionMsgFromRollPitchYaw(Decode_data[0],Decode_data[0],Decode_data[2]);
                            imu.orientation=Qua;
                        }
                        else if(tag==0x54)    // 0x54 磁场
                        {
                            mag.magnetic_field.x = Decode_data[0];
                            mag.magnetic_field.y = Decode_data[1];
                            mag.magnetic_field.z = Decode_data[2];
                        }
						else if(tag == 0x59)                     // 0x59 四元数
						{
                            // imu.orientation.x = Decode_data[0];
                            // imu.orientation.y = Decode_data[1];
                            // imu.orientation.z = Decode_data[2];
                            // imu.orientation.w = Decode_data[3];
						}
                        else
                        {
                            std::cout<<"*******"<<std::endl;
                        }
                    }
                    else
                    {
                        std::cout<<"Decode data Failed!"<<std::endl;
                        continue;
                    }
                    // begin check next frame data
                    for (int i = 11; i < num; i++)
                        chrBuffer[i - 11] = chrBuffer[i];
                    num -= 11;
                }
                imu.header.frame_id = "imu_link";
                imu.header.stamp = ros::Time::now();
                imu_pub.publish(imu);

                mag.header.frame_id = "mag_link";
                mag.header.stamp = ros::Time::now();
                mag_pub.publish(mag);

                memset(Decode_data,0,4);
                tag = 0x00;
            }
            else
            {
                std::cout<<"recv num < 11 "<<std::endl;
            }
        }
        else  // ser not available
        {
            std::cout<<"not rev data"<<std::endl;
        }

        ros::spinOnce();
        r.sleep();
    }

}


bool DecodeIMUData(double result[], unsigned char chrTemp[])
{
    int scale;
    if( (chrTemp[1]==0x51)||(chrTemp[1]==0x52)||(chrTemp[1]==0x53)||(chrTemp[1]==0x54)||(chrTemp[1]==0x59))
    {
        switch(chrTemp[1])
        {
        case 0x51:                                              // 加速度
            scale = 16*9.8; 
            result[0] = (  short(chrTemp[3]<<8|chrTemp[2]) ) / 32768.0 * scale ;
            result[1] = (  short(chrTemp[5]<<8|chrTemp[4]) ) / 32768.0 * scale ;
            result[2] = (  short(chrTemp[7]<<8|chrTemp[6]) ) / 32768.0 * scale ;
            result[3] = (  short(chrTemp[9]<<8|chrTemp[8]) ) / 100.0;
            tag=0x51;
            break;
        case 0x52:                                               // 角速度
            scale = 2000*3.1415926/180; 
            result[0] = ( short(chrTemp[3]<<8|chrTemp[2] ) ) / 32768.0 * scale;
            result[1] = ( short(chrTemp[5]<<8|chrTemp[4] ) ) / 32768.0 * scale;
            result[2] = ( short(chrTemp[7]<<8|chrTemp[6] ) ) / 32768.0 * scale;
            result[3] = ( short(chrTemp[9]<<8|chrTemp[8] ) ) / 100.0;
            tag=0x52;
            break;
        case 0x53:                                               // 角度
            scale = 180; 
            result[0] = ( (short(chrTemp[3]<<8|chrTemp[2] ) ) / 32768.0 * scale) *3.1415926 / 180.0;
            result[1] = ( (short(chrTemp[5]<<8|chrTemp[4] ) ) / 32768.0 * scale) *3.1415926 / 180.0;
            result[2] = ( (short(chrTemp[7]<<8|chrTemp[6] ) ) / 32768.0 * scale) *3.1415926 / 180.0;
            result[3] = (short(chrTemp[9]<<8|chrTemp[8] ) ) ;    // 固件版本计算
            tag=0x53;
            break;
        case 0x54:                                              // 磁场
            scale = 1;
            result[0] = ( short(chrTemp[3]<<8|chrTemp[2] ) ) / scale;
            result[1] = ( short(chrTemp[5]<<8|chrTemp[4] ) ) / scale;
            result[2] = ( short(chrTemp[7]<<8|chrTemp[6] ) ) / scale;
            result[3] = ( short(chrTemp[9]<<8|chrTemp[8] ) ) / 100.0;
            tag=0x54;
            break;        
        case 0x59:                                              // 四元素
            scale = 1.0;//Decode_data rad
            result[0] = ( short(chrTemp[3]<<8|chrTemp[2] ) ) / 32768.0 * scale;
            result[1] = ( short(chrTemp[5]<<8|chrTemp[4] ) ) / 32768.0 * scale;
            result[2] = ( short(chrTemp[7]<<8|chrTemp[6] ) ) / 32768.0 * scale;
            result[3] = ( short(chrTemp[9]<<8|chrTemp[8] ) ) / 32768.0 * scale;
            tag=0x59;
            break;
        default:
            tag = 0x00;
        }
        return true;
    }
    else
    {
        std::cout<<"Header error"<<std::endl;
    }
    return false;
}
