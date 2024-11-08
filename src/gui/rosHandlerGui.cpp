//
// Created by tim-linux on 14.12.21.
//

#include "rosHandlerGui.h"

//void rosHandlerGui::updateConfigSonar(int stepSize, int rangeSonar,int frequencyRange,int numberOfSamples) {
//    ping360_sonar::sendingSonarConfig srv;
//    srv.request.range = rangeSonar;
//    srv.request.stepSize = stepSize;
//    srv.request.frequencyRange = frequencyRange;
//    srv.request.numberOfSamples = numberOfSamples;
//
//    clientSonar.call(srv);
////    std::cout << "changed sonar Config" << std::endl;
//}


void rosHandlerGui::positionCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
//    std::cout << "test1332"<< std::endl;
    this->xPositionRobot.push_back(msg->pose.pose.position.x);
    this->yPositionRobot.push_back(msg->pose.pose.position.y);

    Eigen::Quaterniond rotation;
    rotation.x() = msg->pose.pose.orientation.x;
    rotation.y() = msg->pose.pose.orientation.y;
    rotation.z() = msg->pose.pose.orientation.z;
    rotation.w() = msg->pose.pose.orientation.w;
    Eigen::MatrixXd covariance;
    covariance = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 6; i++) {
        covariance(i, i) = msg->pose.covariance.at(i);
    }

    Eigen::Vector3d rollPitchYaw = this->getRollPitchYaw(rotation);
    this->yawPositionRobot.push_back(rollPitchYaw[2]);
//    std::cout << "test132"<< std::endl;
    emit this->updatePlotPositionVectorROS(this->xPositionRobot, this->yPositionRobot, this->yawPositionRobot);
    emit this->updateStateOfRobotROS(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                                     rollPitchYaw[0], rollPitchYaw[1], rollPitchYaw[2], covariance);
}


void rosHandlerGui::leakageTopTubeCallback(const commonbluerovmsg::msg::LeakageDetection::SharedPtr msg) {
//    std::cout << "test" << std::endl;
//    msg->leakage_detected
    emit updateLeakageStatusTopTubeROS(msg->leakage_detected);
}

void rosHandlerGui::leakageSensorTubeCallback(const commonbluerovmsg::msg::LeakageDetection::SharedPtr msg) {
//    std::cout << "test" << std::endl;
//    msg->leakage_detected
    emit updateLeakageStatusSensorTubeROS(msg->leakage_detected);
}

void rosHandlerGui::DVLCallback(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
//    std::cout << "test" << std::endl;

    emit updateDVLStateROS(msg->report.transducers[0].distance, msg->report.transducers[1].distance,
                           msg->report.transducers[2].distance, msg->report.transducers[3].distance);
}


void rosHandlerGui::updateAngleCamera(int angleCamera) {
    auto srv = std::make_shared<commonbluerovmsg::srv::CameraAngle::Request>();

    srv->angle = angleCamera;
    auto result = this->clientCameraAngle->async_send_request(srv);
}

void rosHandlerGui::updateLightIntensity(int intensityLight) {
    auto srv = std::make_shared<commonbluerovmsg::srv::LightDensity::Request>();


    srv->intensity = intensityLight;
    auto result = this->clientLight->async_send_request(srv);

}

void rosHandlerGui::ping360ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//    std::cout << "receaving image" << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat testImage = cv_bridge::cvtColor(cv_ptr, "rgb8")->image;

    cv::applyColorMap(testImage, testImage, cv::COLORMAP_JET);
    QImage imgIn = QImage((uchar *) testImage.data, testImage.cols, testImage.rows, testImage.step,
                          QImage::Format_RGB888);
//    QImage imgIn = QImage(&msg->data[0], msg->width, msg->height, QImage::Format_RGB888);
    imgIn = imgIn.rgbSwapped();
    QPixmap myPixMap = QPixmap::fromImage(imgIn);

    emit updatePing360SonarImageROS(myPixMap);
}

void rosHandlerGui::micronImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//    std::cout << "receaving image" << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat testImage = cv_bridge::cvtColor(cv_ptr, "rgb8")->image;

    cv::applyColorMap(testImage, testImage, cv::COLORMAP_JET);
    QImage imgIn = QImage((uchar *) testImage.data, testImage.cols, testImage.rows, testImage.step,
                          QImage::Format_RGB888);
//    QImage imgIn = QImage(&msg->data[0], msg->width, msg->height, QImage::Format_RGB888);
    imgIn = imgIn.rgbSwapped();
    QPixmap myPixMap = QPixmap::fromImage(imgIn);

    emit updateMicronSonarImageROS(myPixMap);
}

void rosHandlerGui::cameraImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr = cv_bridge::cvtColor(cv_ptr, "rgb8");

    QImage imgIn = QImage((uchar *) cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step,
                          QImage::Format_RGB888);

    //QImage::Format_BGR30  Format_RGB888
    QPixmap myPixMap = QPixmap::fromImage(imgIn);

    emit updateCameraImageROS(myPixMap);
}

void rosHandlerGui::updateDesiredState(double desiredHeight, double desiredRoll, double desiredPitch, double desiredYaw,
                                       double desiredXMovement, double desiredYMovement, bool holdPosition) {

    //this just sends the data to ros, the frame rate is made by mainwindow
    commonbluerovmsg::msg::DesiredStateForRobot msg;
    msg.timestamp = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
    msg.desired_height = desiredHeight;
    msg.desired_roll = desiredRoll;
    msg.desired_pitch = desiredPitch;
    msg.desired_x_thrust = desiredXMovement;
    msg.desired_yaw = desiredYaw;
    msg.desired_y_thrust = desiredYMovement;
    msg.hold_position = holdPosition;
    this->publishingDesiredState->publish(msg);

}

void rosHandlerGui::resetEKFEstimator(bool resetOnlyGraph) {
//    if (not resetOnlyGraph) {
//        commonbluerovmsg::resetekf srv;
//        srv.request.xPos = 0;
//        srv.request.yPos = 0;
//        srv.request.yaw = 0;
//        srv.request.resetCovariances = true;
//        this->clientEKF.call(srv);
//    }

    this->xPositionRobot.clear();
    this->yPositionRobot.clear();
    this->yawPositionRobot.clear();
}
