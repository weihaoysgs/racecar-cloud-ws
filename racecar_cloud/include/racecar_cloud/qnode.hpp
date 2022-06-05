/**
 * @file /include/racecar_cloud/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef racecar_cloud_QNODE_HPP_
#define racecar_cloud_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "../include/racecar_cloud/roadLine.h"
#include "../include/racecar_cloud/trafficlight.h"
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/Bool.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace racecar_cloud {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

    ros::Publisher& get_navigation_point_publisher()
    {
        return this->navigation_points_publisher_;
    }

    ros::Publisher& getCancelCurrentNavGoalPublisher()
    {
        return cancel_current_nav_goal_pub_;
    }

    ros::Publisher& getForceRacecarVisionCloseOrOpenPublisher()
    {
        return force_racecar_vision_open_or_close_pub_;
    }

    ros::Publisher& getForceRacecarNavStopOrMovePublisher()
    {
        return force_racecar_nav_stop_or_move_pub_;
    }

    geometry_msgs::PoseStamped& getRvizSetPose()
    {
        return this->rviz_set_pose_;
    }

    cv::Mat &getRacecarSrcImage()
    {
        return racecar_src_image_;
    }

    void getRvizPublishPoseStamped(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void SubTrafficlightCallback(const visionmsg::trafficlight::ConstPtr &trafficlight_msg);
    void SubRacecarSrcImageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg);
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void rvizGetPose();
    void getRacecarImageSignal();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher cancel_current_nav_goal_pub_;

    ros::Publisher force_racecar_nav_stop_or_move_pub_;
    ros::Publisher force_racecar_vision_open_or_close_pub_;

    ros::Publisher navigation_points_publisher_;

    ros::Subscriber rviz_pose_sub_;
    ros::Subscriber sub_trafficlight_status_;
    ros::Subscriber sub_racecar_image_;
    geometry_msgs::PoseStamped rviz_set_pose_;
    QStringListModel logging_model;
    cv::Mat racecar_src_image_;
};

}  // namespace racecar_cloud

#endif /* racecar_cloud_QNODE_HPP_ */
