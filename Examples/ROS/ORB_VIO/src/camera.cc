#include <sstream> // for converting the command line parameter to integer

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "camera.h"
#include "utility.h"

using namespace std;
using namespace mynteye;

sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t) {
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *) &num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*) (&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*) (&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }

    return ptr;
}


int main(int argc, char** argv)
{

    // Check if video source has been passed as a parameter
    if(argv[1] == NULL) return 1;

    ros::init(argc, argv, "mynteye_publisher");
    ros::NodeHandle nh;
    //ros::NodeHandle ni;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pubimage = it.advertise("/cam0/image_raw", 1);
    ros::Publisher pubimu = nh.advertise<sensor_msgs::Imu>("/imu0",1);



    cout << "Open Camera: "<< endl;
    stringstream ss;
    ss << argv[1];
    mynteye::Camera cam;
    InitParameters params(ss.str());
    cout << "Open Camera:1 " << endl;

    cam.Open(params);
    std::cout << "Open Camera: "  << std::endl;
    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    std::cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m\n";

    while(cam.Grab() != ErrorCode::SUCCESS) {
        std::cout << "Warning: Grab failed <" << ">" << std::endl;
    }

    std::cout <<"camera ok"<<endl;
#if 0
    // Convert the passed as command line parameter index for the video device to an integer
    std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    // Check if it is indeed a number
    if(!(video_sourceCmd >> video_source)) return 1;
    cv::VideoCapture cap(video_source);
    // Check if video device can be opened with the given index
    if(!cap.isOpened()) return 1;
#endif

    cv::Mat img_left;
    ErrorCode code;

    IMUData imudata;
    vector<IMUData> imudatas;
    std::uint32_t timestamp;

    sensor_msgs::ImagePtr msgimage;
    sensor_msgs::Imu msgimu;
    uint32_t image_sec;
    uint32_t image_nsec;
    uint32_t imu_sec;
    uint32_t imu_nsec;
    uint32_t imu_time;
    int i = 0;
    ros::Rate loop_rate(800);

    while (nh.ok()) {
        cam.Grab();
        cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED);
        // Check if grabbed frame is actually full with some content
        if(!img_left.empty()) {
            cam.RetrieveIMUData(imudatas,timestamp);
            image_sec = timestamp/10000;
            image_nsec = (timestamp - image_sec * 10000) * 100000;
            cout << "timestamp" << timestamp << std::endl;
            //cout << "image_sec" << image_sec << "image_nsec" << image_nsec << endl;
            ros::Time time(image_sec, image_nsec);
            msgimage = imageToROSmsg(img_left, sensor_msgs::image_encodings::MONO8,"1",time);
            //if(i%3 == 0)
            {
                pubimage.publish(msgimage);
            }
            size_t size = imudatas.size();
            for (int k = 0; k < size; k++)
            {
                imudata = imudatas[k];
                imu_time = timestamp + imudata.time_offset;
                imu_sec = imu_time/10000;
                imu_nsec = (imu_time - imu_sec * 10000)*100000;
                cout << "imu_time" << imu_time << "imu_sec" << imu_sec << "imu_nsec" << imu_nsec << endl;
                ros::Time time(imu_sec, imu_nsec);
                msgimu.header.stamp = time;
                msgimu.angular_velocity.x = imudata.gyro_x/57.2956;
                msgimu.angular_velocity.y = imudata.gyro_y/57.2956;
                msgimu.angular_velocity.z = imudata.gyro_z/57.2956;

                msgimu.linear_acceleration.x = imudata.accel_x*9.8;
                msgimu.linear_acceleration.y = imudata.accel_y*9.8;
                msgimu.linear_acceleration.z = imudata.accel_z*9.8;

                pubimu.publish(msgimu);
                ros::spinOnce();
                loop_rate.sleep();
            }
            i++;
        }

    }

}
