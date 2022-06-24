/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/racecar_cloud/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace racecar_cloud {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"racecar_cloud");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    navigation_points_publisher_ = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    rviz_pose_sub_ = n.subscribe("/move_base_simple/goal", 1 , &QNode::getRvizPublishPoseStamped, this);
    sub_trafficlight_status_ = n.subscribe<visionmsg::trafficlight>("trafficLight", 1, &QNode::SubTrafficlightCallback,this);
    sub_aruco_status_ = n.subscribe<visionmsg::arucostatus>("/aruco_status", 1, &QNode::SubArucoStatusCallback, this);
    sub_racecar_current_carto_pose_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("racecar_current_carto_pose",1,&QNode::SubRacecarCurrentCartoPose,this);
    //    sub_racecar_image_ = n.subscribe<sensor_msgs::CompressedImage>("/image_view/image_raw/compressed",1,&QNode::SubRacecarSrcImageCallback,this);

    cancel_current_nav_goal_pub_ = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1 );
    traffic_light_status_publisher_ = n.advertise<visionmsg::arucotrafficlight>("traffic_light_status",1);
    force_racecar_nav_stop_or_move_pub_ = n.advertise<std_msgs::Bool>("force_racecar_nav_stop_or_move", 1);
    force_racecar_vision_open_or_close_pub_ = n.advertise<std_msgs::Bool>("force_racecar_vision_close_or_open", 1);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"racecar_cloud");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    navigation_points_publisher_ = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    rviz_pose_sub_ = n.subscribe("/move_base_simple/goal", 1 , &QNode::getRvizPublishPoseStamped, this);
    sub_trafficlight_status_ = n.subscribe<visionmsg::trafficlight>("trafficLight", 1, &QNode::SubTrafficlightCallback,this);
    sub_aruco_status_ = n.subscribe<visionmsg::arucostatus>("/aruco_status", 1, &QNode::SubArucoStatusCallback, this);
    sub_racecar_current_carto_pose_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("racecar_current_carto_pose",1,&QNode::SubRacecarCurrentCartoPose,this);
//    sub_racecar_image_ = n.subscribe<sensor_msgs::CompressedImage>("/image_view/image_raw/compressed",1,&QNode::SubRacecarSrcImageCallback,this);

    cancel_current_nav_goal_pub_ = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1 );
    traffic_light_status_publisher_ = n.advertise<visionmsg::arucotrafficlight>("traffic_light_status",1);
    force_racecar_nav_stop_or_move_pub_ = n.advertise<std_msgs::Bool>("force_racecar_nav_stop_or_move", 1);
    force_racecar_vision_open_or_close_pub_ = n.advertise<std_msgs::Bool>("force_racecar_vision_close_or_open", 1);

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
    int count = 0;
	while ( ros::ok() ) {

//        std_msgs::String msg;
//        std::stringstream ss;
//        ss << "hello world " << count;
//        msg.data = ss.str();
//        chatter_publisher.publish(msg);
//        log(Info,std::string("I sent: ")+msg.data);
        ros::spinOnce();
//		loop_rate.sleep();
//		++count;
	}
    cv::destroyAllWindows();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
//				ROS_INFO_STREAM(msg);
//				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
//				break;
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO]: " << msg << "\n";
        break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::getRvizPublishPoseStamped(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    rviz_set_pose_.pose.position = msg->pose.position;
    rviz_set_pose_.pose.orientation = msg->pose.orientation;
    rviz_set_pose_.header = msg->header;
    ROS_INFO("sub");
    Q_EMIT rvizGetPose();
}

void QNode::SubTrafficlightCallback(const visionmsg::trafficlight::ConstPtr &trafficlight_msg)
{
    if (trafficlight_msg->trafficstatus == 0)
    {
        std::cout << "no detect" << std::endl;
    }
}

void QNode::SubRacecarSrcImageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    ROS_INFO("I have image");
    try {

        cv_bridge::CvImagePtr cv_ptr_image_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        racecar_src_image_ = cv_ptr_image_compressed->image;
//        cv::imshow("racecar_image",racecar_src_image_);
//        cv::waitKey(1);
        Q_EMIT getRacecarImageSignal();

        ROS_INFO("hellow i get racecar image");

    } catch (cv_bridge::Exception &e) {

    }
}

void QNode::SubArucoStatusCallback(const visionmsg::arucostatus::ConstPtr &msg)
{
    aruco_status_msg_ = *msg;
    Q_EMIT getArucoStatusSignal();
//    std::cout << "get aruco msg" << aruco_status_msg_.have_aruco << " " << aruco_status_msg_.aruco_distance << std::endl;
}

void QNode::SubRacecarCurrentCartoPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    racecar_current_pose_ = *msg;
//    qDebug() << "x: " << racecar_current_pose_.pose.pose.position.x
//             << "y: " << racecar_current_pose_.pose.pose.position.y;
    Q_EMIT getRacecarCurrentCartoPoseSignal();
}


}  // namespace racecar_cloud
