/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/racecar_cloud/main_window.hpp"

namespace racecar_cloud {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));

    this->setWindowTitle("2022 Racecar Upper Machine");

	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    ui.dockWidgetContents_2->show();
    ui.dock_status->show();
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    QObject::connect(&qnode, SIGNAL(rvizGetPose()), this, SLOT(rvizGetNavigtionPose()));

    QObject::connect(&qnode, SIGNAL(getRacecarImageSignal()), this, SLOT(labelShowRacecarImageUpdateCallback()));

    connect(ui.pushButtonWriteCurrentPoints, SIGNAL(clicked()), this, SLOT(writeCurrentPointsToYAML()));

    connect(ui.pushbuttonStartGetCurrentRvizPoint, SIGNAL(clicked()), this, SLOT(pushbuttonStartGetCurrentRvizPointCallback()));

    connect(ui.pushbuttonLoadingGetCurrentRvizPoint, SIGNAL(clicked()), this, SLOT(pushbuttonLoadingGetCurrentRvizPointCallback()));

    connect(ui.pushbuttonUnLoadingGetCurrentRvizPoint, SIGNAL(clicked()), this, SLOT(pushbuttonUnLoadingGetCurrentRvizPointCallback()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    points_params_yaml_path_ = "/home/weihao/qt_ws/src/racecar_cloud/params/points.yaml";
    initAllPoints(points_params_yaml_path_);
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "racecar_cloud");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "racecar_cloud");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}


void MainWindow::on_pushButtonGoToStartPoint_clicked(bool check)
{
    updateCurrentPoints();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = start_point_.x;
    pose.pose.position.y = start_point_.y;
    pose.pose.position.z = 0;

    tf::Quaternion qtn = tf::createQuaternionFromYaw(start_point_.z);

    pose.pose.orientation.w = qtn.w();
    pose.pose.orientation.x = qtn.x();
    pose.pose.orientation.y = qtn.y();
    pose.pose.orientation.z = qtn.z();

    qnode.get_navigation_point_publisher().publish(pose);
    std::cout << "Start Go To Start Point" << std::endl;
}

void MainWindow::on_pushButtonGoToLoadingPoint_clicked(bool check)
{
    updateCurrentPoints();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = loading_point_.x;
    pose.pose.position.y = loading_point_.y;
    pose.pose.position.z = 0;

    tf::Quaternion qtn = tf::createQuaternionFromYaw(loading_point_.z);

    pose.pose.orientation.w = qtn.w();
    pose.pose.orientation.x = qtn.x();
    pose.pose.orientation.y = qtn.y();
    pose.pose.orientation.z = qtn.z();

    qnode.get_navigation_point_publisher().publish(pose);
    std::cout << "Start Go To Loading Point" << std::endl;
}

void MainWindow::on_pushButtonGoToUnLoadingPoint_clicked(bool check)
{
    updateCurrentPoints();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = unloading_point_.x;
    pose.pose.position.y = unloading_point_.y;
    pose.pose.position.z = 0;

    tf::Quaternion qtn = tf::createQuaternionFromYaw(unloading_point_.z);

    pose.pose.orientation.w = qtn.w();
    pose.pose.orientation.x = qtn.x();
    pose.pose.orientation.y = qtn.y();
    pose.pose.orientation.z = qtn.z();

    qnode.get_navigation_point_publisher().publish(pose);
    std::cout << "Start Go To UnLoading Point" << std::endl;
}

void MainWindow::initAllPoints(const std::string &param_file_path)
{
    cv::FileStorage file(param_file_path, cv::FileStorage::READ);
    if (!file.isOpened())
    {
        qDebug() << "Not Correct Open YAML Point Params";
        return;
    }

    file["start_point"] >> start_point_;
    file["loading_point"] >> loading_point_;
    file["unloading_point"] >> unloading_point_;

    file.release();

    qDebug() << "Start Point" << start_point_.x << " " << start_point_.y << " " << start_point_.z << endl;
    qDebug() << "Loading Point" << loading_point_.x << " " << loading_point_.y << " " << loading_point_.z << endl;
    qDebug() << "Unloading Point" << unloading_point_.x << " " << unloading_point_.y << " " << unloading_point_.z << endl;

    ui.lineEditStartPointX->setText(QString::number(start_point_.x));
    ui.lineEditStartPointY->setText(QString::number(start_point_.y));
    ui.lineEditStartPointTheta->setText(QString::number(start_point_.z));

    ui.lineEditLoadingPointX->setText(QString::number(loading_point_.x));
    ui.lineEditLoadingPointY->setText(QString::number(loading_point_.y));
    ui.lineEditLoadingPointTheta->setText(QString::number(loading_point_.z));

    ui.lineEditUnLoadingPointX->setText(QString::number(unloading_point_.x));
    ui.lineEditUnLoadingPointY->setText(QString::number(unloading_point_.y));
    ui.lineEditUnLoadingPointTheta->setText(QString::number(unloading_point_.z));
}


void MainWindow::writeCurrentPointsToYAML()
{
    const std::string yaml_write_path = points_params_yaml_path_;
    cv::FileStorage file(yaml_write_path, cv::FileStorage::WRITE);

    updateCurrentPoints();

    file << "start_point " << start_point_;
    file << "loading_point " << loading_point_;
    file << "unloading_point " << unloading_point_;

    file.release();

    qDebug() << "Write New YAML File OK!";
}

void MainWindow::updateCurrentPoints()
{
    start_point_.x = ui.lineEditStartPointX->text().toDouble();
    start_point_.y = ui.lineEditStartPointY->text().toDouble();
    start_point_.z = ui.lineEditStartPointTheta->text().toDouble();

    loading_point_.x = ui.lineEditLoadingPointX->text().toDouble();
    loading_point_.y = ui.lineEditLoadingPointY->text().toDouble();
    loading_point_.z = ui.lineEditLoadingPointTheta->text().toDouble();


    unloading_point_.x = ui.lineEditUnLoadingPointX->text().toDouble();
    unloading_point_.y = ui.lineEditUnLoadingPointY->text().toDouble();
    unloading_point_.z = ui.lineEditUnLoadingPointTheta->text().toDouble();
}

/**
 * @brief Get the RVIZ set navigation point
 */
void MainWindow::rvizGetNavigtionPose()
{
    double pose_x = qnode.getRvizSetPose().pose.position.x;
    double pose_y = qnode.getRvizSetPose().pose.position.y;
    double pose_z = qnode.getRvizSetPose().pose.position.z;

    double oriention_w = qnode.getRvizSetPose().pose.orientation.w;
    double oriention_x = qnode.getRvizSetPose().pose.orientation.x;
    double oriention_y = qnode.getRvizSetPose().pose.orientation.y;
    double oriention_z = qnode.getRvizSetPose().pose.orientation.z;

    qDebug() << qnode.getRvizSetPose().pose.position.x << " " ;
    ui.labelPositionX->setNum(pose_x);

    qDebug() << qnode.getRvizSetPose().pose.position.y << " " ;
    ui.labelPositionY->setNum(pose_y);

    qDebug() << qnode.getRvizSetPose().pose.position.z << " " ;
    ui.labelPositionZ->setNum(pose_z);

    qDebug() << qnode.getRvizSetPose().pose.orientation.w << " " ;
    ui.labelOrientionW->setNum(oriention_w);

    qDebug() << qnode.getRvizSetPose().pose.orientation.x << " " ;
    ui.labelOrientionX->setNum(oriention_x);

    qDebug() << qnode.getRvizSetPose().pose.orientation.y << " " ;
    ui.labelOrientionY->setNum(oriention_y);

    qDebug() << qnode.getRvizSetPose().pose.orientation.z << " " ;
    ui.labelOrientionZ->setNum(oriention_z);

    tf2::Quaternion qtn = (tf2::Quaternion(oriention_x,oriention_y,oriention_z,oriention_w));
    tf2::Matrix3x3 m(qtn);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ui.labelPositionTheta->setNum(yaw*180.0/M_PI);
}

void MainWindow::pushbuttonStartGetCurrentRvizPointCallback()
{

    tf2::Matrix3x3 m(tf2::Quaternion(qnode.getRvizSetPose().pose.orientation.x,
                                     qnode.getRvizSetPose().pose.orientation.y,
                                     qnode.getRvizSetPose().pose.orientation.z,
                                     qnode.getRvizSetPose().pose.orientation.w));
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ui.lineEditStartPointX->setText(QString::number(qnode.getRvizSetPose().pose.position.x));
    ui.lineEditStartPointY->setText(QString::number(qnode.getRvizSetPose().pose.position.y));
    ui.lineEditStartPointTheta->setText(QString::number(yaw * 180.0/ M_PI));
}

void MainWindow::pushbuttonLoadingGetCurrentRvizPointCallback()
{
    tf2::Matrix3x3 m(tf2::Quaternion(qnode.getRvizSetPose().pose.orientation.x,
                                     qnode.getRvizSetPose().pose.orientation.y,
                                     qnode.getRvizSetPose().pose.orientation.z,
                                     qnode.getRvizSetPose().pose.orientation.w));
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ui.lineEditLoadingPointX->setText(QString::number(qnode.getRvizSetPose().pose.position.x));
    ui.lineEditLoadingPointY->setText(QString::number(qnode.getRvizSetPose().pose.position.y));
    ui.lineEditLoadingPointTheta->setText(QString::number(yaw * 180.0/ M_PI));
}

void MainWindow::pushbuttonUnLoadingGetCurrentRvizPointCallback()
{
    tf2::Matrix3x3 m(tf2::Quaternion(qnode.getRvizSetPose().pose.orientation.x,
                                     qnode.getRvizSetPose().pose.orientation.y,
                                     qnode.getRvizSetPose().pose.orientation.z,
                                     qnode.getRvizSetPose().pose.orientation.w));
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ui.lineEditUnLoadingPointX->setText(QString::number(qnode.getRvizSetPose().pose.position.x));
    ui.lineEditUnLoadingPointY->setText(QString::number(qnode.getRvizSetPose().pose.position.y));
    ui.lineEditUnLoadingPointTheta->setText(QString::number(yaw * 180.0/ M_PI));
}

void MainWindow::labelShowRacecarImageUpdateCallback()
{
    cv::Mat img = qnode.getRacecarSrcImage();

    cvtColor(img, img, CV_BGR2RGB);
    QImage Qtemp = QImage((const unsigned char *)(img.data), img.cols, img.rows, img.step,
                          QImage::Format_RGB888);
    ui.labelShowRacecarImage->setPixmap(QPixmap::fromImage(Qtemp));
    ui.labelShowRacecarImage->resize(Qtemp.size());
    ui.labelShowRacecarImage->show();
    qnode.log(QNode::Info,std::string("I Receive Image"));
//    cv::imshow("img",img);
//    cv::waitKey(1);

}

}  // namespace racecar_cloud

