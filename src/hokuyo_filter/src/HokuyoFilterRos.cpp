#include "hokuyo_filter/HokuyoFilterRos.h"
#include <utility> // for std::pair
#include <stdexcept> // for std::invalid_argument


HokuyoFilterRos::HokuyoFilterRos()
{
    ros::NodeHandle private_nh_("~");

    private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
    private_nh_.param("scan_filter_topic", p_scan_filter_topic_, std::string("scan_filter"));
    private_nh_.param("scan_subscriber_queue_size_", p_scan_subscriber_queue_size_, 10);
    private_nh_.param("LASER_RANGE_MIN", LASER_RANGE_MIN, float(0.2));
    private_nh_.param("LASER_RANGE_MAX", LASER_RANGE_MAX, float(4.5));
    private_nh_.param("num_readings", num_readings, 726);

    ROS_INFO("HokuyoFilt p_scan_topic_: %s", p_scan_topic_.c_str());
    ROS_INFO("HokuyoFilt p_scan_filter_topic_: %s", p_scan_filter_topic_.c_str());

    scanSubscriber_ = node_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HokuyoFilterRos::scanCallback, this);
    scanPublisher_ = node_.advertise<sensor_msgs::LaserScan>(p_scan_filter_topic_, p_scan_subscriber_queue_size_);

    // Subscriber for fire_detection topic
    fireSub_ = node_.subscribe("/fire_detection", 10, &HokuyoFilterRos::fireCallback, this);

    // Publisher for min_pose topic
    posePub_ = node_.advertise<geometry_msgs::Pose2D>("/min_pose", 10);
    ROS_INFO("Filt: Init done!");
}

HokuyoFilterRos::~HokuyoFilterRos()
{

    // Shutdown ROS publishers or subscribers if necessary
    posePub_.shutdown();
    fireSub_.shutdown();
    scanPublisher_.shutdown();
    scanSubscriber_.shutdown();

    // Any other cleanup code required
}

void HokuyoFilterRos::scanCallback(const sensor_msgs::LaserScan& msg)
{
    //ROS_INFO("Filt: Received Scan! %d",msg.ranges.size());
    ranges=msg.ranges;
	cleanLaser(msg.ranges, LASER_RANGE_MIN, LASER_RANGE_MAX); //clean data and populate global variable ranges
    sensor_msgs::LaserScan newmsg = msg;
    for (int i = 0; i < msg.ranges.size(); i++)
        newmsg.ranges[i] = ranges[i];
    scanPublisher_.publish(newmsg);
}

void HokuyoFilterRos::fireCallback(const std_msgs::Int8& fire_msg)
{
	ROS_INFO("Fire callback!");
	if(ranges.size()<=0) return;
    try {
        std::vector<float> subArray = getSubArray(ranges, fire_msg.data);
        std::pair<size_t, float> res = findMinElement(subArray);
        int minIndex=res.first;
        float minValue=res.second;
        ROS_INFO("Fire callback! %d %f",minIndex,minValue);
        float angle = calculateAngle(minIndex, fire_msg.data);
        // Publishing pose
        geometry_msgs::Pose2D pose;
        pose.x = minValue;  // distance
        pose.theta = angle; // angle
        posePub_.publish(pose);
    } catch (const std::exception& e) {
        ROS_ERROR("Error in fireCallback: %s", e.what());
    }
}

void HokuyoFilterRos::cleanLaser(std::vector<float> msg_ranges, float range_min, float range_max)
{
    int k = 0, prev_range = 0;
    num_readings = msg_ranges.size();
        //ROS_INFO("Filt: Cleaning..");
    for (int i = 153; i < num_readings-154; i++)
    {
        if (!isnan(msg_ranges[i])) // not nan messages
        {
            ranges[k] = msg_ranges[i];
            if (i > 0)
            {
                if ((isnan(msg_ranges[i - 1]) || isnan(msg_ranges[i + 1])) && (ranges[k] < range_min || ranges[k] > range_max)) // invalid ranges not allowed to terminate NaNs
                {
                    ranges[k] = msg_ranges[prev_range];
                }
                if (ranges[k] >= range_min && !isnan(msg_ranges[i - 1]) && !isnan(msg_ranges[i + 1])) // valid range value (minimum only) and no NaNs around
                {
                    prev_range = i;
                }
            }
        }
        else
        {
            ranges[k] = msg_ranges[prev_range]; // nan is replaced by previous best value
        }
        if (ranges[k] < range_min) ranges[k] = range_min;
        if (ranges[k] > range_max) ranges[k] = range_max;
        k++;
    }
            //ROS_INFO("Filt: Cleaning done");
}

std::vector<float> HokuyoFilterRos::getSubArray(const std::vector<float>& ranges, int fire_msg) //value to select which laser is to be used
{

    // Define the start index for each sub-array based on the fire_msg
    size_t startIndex;
    if (fire_msg == -1) {
        startIndex = 0;
    } else if (fire_msg == 0) {
        startIndex = SUB_ARRAY_SIZE;
    } else if (fire_msg == 1) {
        startIndex = 2 * SUB_ARRAY_SIZE;
    } else {
        // Handle invalid fire_msg value
        throw std::invalid_argument("Invalid fire_msg value. Must be -1, 0, or 1.");
    }

    // Extract and return the sub-array
    std::vector<float> subArray(ranges.begin() + startIndex, ranges.begin() + startIndex + SUB_ARRAY_SIZE);
    return subArray;
}

std::pair<size_t, float> HokuyoFilterRos::findMinElement(const std::vector<float>& array) {
    if (array.empty()) {
        throw std::invalid_argument("Array is empty.");
    }

    // Initialize the minimum value and index
    size_t minIndex = 0;
    float minValue = 9999;

    // Iterate through the array to find the minimum element
    for (size_t i = 1; i < array.size(); ++i) {
        if (array[i] < minValue &&array[i]>(LASER_RANGE_MIN+0.1)) {
            minValue = array[i];
            minIndex = i;
        }
    }
    // Return the index and value of the minimum element
    return {minIndex, minValue};
}

float HokuyoFilterRos::calculateAngle(size_t minIndex, int fire_msg) {
    // Calculate the final index based on the array selector
    int finalIndex;
    switch (fire_msg) {
        case -1:
            finalIndex = minIndex;
            break;
        case 0:
            finalIndex = minIndex + SUB_ARRAY_SIZE;
            break;
        case 1:
            finalIndex = minIndex + (2 * SUB_ARRAY_SIZE);
            break;
        default:
            throw std::invalid_argument("Invalid array selector. Must be -1, 0, or 1.");
    }
    
    // Calculate the angle
    float angle = (float)((finalIndex-210) / 3.025);
    ROS_INFO("filt: Fire callback! %d %f ",finalIndex,angle);
    return angle;
}
