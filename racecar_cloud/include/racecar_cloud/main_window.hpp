/**
 * @file /include/racecar_cloud/main_window.hpp
 *
 * @brief Qt based gui for racecar_cloud.
 *
 * @date November 2010
 **/
#ifndef racecar_cloud_MAIN_WINDOW_H
#define racecar_cloud_MAIN_WINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <QString>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace racecar_cloud {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void initAllPoints(const std::string& param_file_path);
    void updateCurrentPoints();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
    void on_pushButtonGoToStartPoint_clicked(bool check);
    void on_pushButtonGoToLoadingPoint_clicked(bool check);
    void on_pushButtonGoToUnLoadingPoint_clicked(bool check);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void rvizGetNavigtionPose();
    void writeCurrentPointsToYAML();
    void pushbuttonStartGetCurrentRvizPointCallback();
    void pushbuttonLoadingGetCurrentRvizPointCallback();
    void pushbuttonUnLoadingGetCurrentRvizPointCallback();
    void pushbuttonGetFirstRestRvizPointCallback();
    void pushbuttonGetSecondRestRvizPointCallback();
    void pushbuttonGoToFirstRestPointCallback();
    void pushbuttonGoToSecondRestPointCallback();
    void labelShowRacecarImageUpdateCallback();
private:
	Ui::MainWindowDesign ui;

    cv::Point3d start_point_;
    cv::Point3d loading_point_;
    cv::Point3d unloading_point_;
    cv::Point3d first_rest_point_;
    cv::Point3d second_rest_point_;

    std::string points_params_yaml_path_;
	QNode qnode;
};

}  // namespace racecar_cloud

#endif // racecar_cloud_MAIN_WINDOW_H
