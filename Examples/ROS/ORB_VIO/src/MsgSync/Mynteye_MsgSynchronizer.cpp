#include "MsgSynchronizer.h"
#include "../../../src/IMU/configparam.h"

namespace ORBVIO
{

MsgSynchronizer::MsgSynchronizer(const double& imagedelay):
    _imageMsgDelaySec(imagedelay), _status(NOTINIT),
    _dataUnsyncCnt(0)
{
    printf("image delay set as %.1fms\n",_imageMsgDelaySec*1000);
}

MsgSynchronizer::~MsgSynchronizer()
{

}


bool MsgSynchronizer::getRecentMsgs(sensor_msgs::ImageConstPtr &imgmsg, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs)
{
    //unique_lock<mutex> lock2(_mutexIMUQueue);
    unique_lock<mutex> lock1(_mutexImageQueue);

    if(_status == NOTINIT || _status == INIT)
    {
        //ROS_INFO("synchronizer not inited");
        return false;
    }

    if(_imageMsgQueue.empty())
    {
        //ROS_INFO("no image stored in queue currently.");
        return false;
    }

    if(_imuMsgQueue.empty())
    {
        //ROS_WARN("no imu message stored, shouldn't");
        return false;
    }

    // get image message
    imgmsg = _imageMsgQueue.front();
    _imageMsgQueue.pop();
    //ROS_INFO("image timedrect: %.4f",imgmsg->header.stamp.toSec());
    // clear imu message vector, and push all imu messages whose timestamp is earlier than image message
    vimumsgs.clear();

    while(true)
    {
        // if no more imu messages, stop loop
        if(_imuMsgQueue.empty())
            break;

        // consider delay between image and imu serial
        sensor_msgs::ImuConstPtr tmpimumsg = _imuMsgQueue.front();
        if(tmpimumsg->header.stamp.toSec() <= imgmsg->header.stamp.toSec())
        {
            // add to imu message vector
            vimumsgs.push_back(tmpimumsg);
            {
                unique_lock<mutex> lock(_mutexIMUQueue);
                _imuMsgQueue.pop();
            }

            _dataUnsyncCnt = 0;
        }
        else
        {
            if(_dataUnsyncCnt++>10)
            {
                _dataUnsyncCnt = 0;
                //_imuMsgQueue = std::queue<sensor_msgs::ImuConstPtr>();
                clearMsgs();
                ROS_ERROR("data unsynced many times, reset sync");
                return false;
            }
            // stop loop
            break;
        }
    }

    if(vimumsgs.size()==0)
        ROS_ERROR("no imu message between images!");

    return true;
}

void MsgSynchronizer::addImuMsg(const sensor_msgs::ImuConstPtr &imumsg)
{
    unique_lock<mutex> lock(_mutexIMUQueue);

    if(_imageMsgDelaySec>=0) {
        _imuMsgQueue.push(imumsg);
        if(_status == NOTINIT)
        {
            _imuMsgTimeStart = imumsg->header.stamp;
            _status = INIT;
        }
    }
    else {
        // if there's no image messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT)
        {
            _imuMsgQueue.push(imumsg);
            _status = NORMAL;
        }
        else
        {
            // push message into queue
            _imuMsgQueue.push(imumsg);
        }
    }


}

void MsgSynchronizer::addImageMsg(const sensor_msgs::ImageConstPtr &imgmsg)
{
    unique_lock<mutex> lock(_mutexImageQueue);

    if(_imageMsgDelaySec >= 0) {
        // if there's no imu messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT)
        {
            // ignore all image messages with no imu messages between them
            // only add below images
            //if(imgmsg->header.stamp.toSec() - _imageMsgDelaySec > _imuMsgTimeStart.toSec())
            if(imgmsg->header.stamp.toSec() > _imuMsgTimeStart.toSec())
            {
                _imageMsgQueue.push(imgmsg);
                _status = NORMAL;
            }
        }
        else
        {
            // push message into queue
            _imageMsgQueue.push(imgmsg);
        }
    }
    else {  // start by image message
        if(_status == NOTINIT)
        {
            _imuMsgTimeStart = imgmsg->header.stamp;
            _status = INIT;
        }
        else
        {   // no image data if there's no imu message
            _imageMsgQueue.push(imgmsg);
        }
    }
}


void MsgSynchronizer::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    addImageMsg(msg);
}

void MsgSynchronizer::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    addImuMsg(msg);
}

void MsgSynchronizer::clearMsgs(void)
{
    _imuMsgQueue = std::queue<sensor_msgs::ImuConstPtr>();
    _imageMsgQueue = std::queue<sensor_msgs::ImageConstPtr>();
    //    while(!_imageMsgQueue.empty())
    //    {
    //        _imageMsgQueue.pop();
    //    }
    //    while(!_imuMsgQueue.empty())
    //    {
    //        _imuMsgQueue.pop();
    //    }
}

}
