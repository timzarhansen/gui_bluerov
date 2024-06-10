
#include "mainwindow.h"


//void init(){
//    ros::spin();
//}


int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);

    auto nodeShared = std::make_shared<rosHandlerGui>();



//    Q_INIT_RESOURCE(application);
    QApplication app(argc, argv);
//    std::cout << QT_VERSION_STR << std::endl;
//    QCoreApplication::setApplicationVersion(QT_VERSION_STR);

    qRegisterMetaType<Eigen::MatrixXd>("Eigen::MatrixXd");
    qRegisterMetaType<std::vector<double>>("std::vector<double>");
    MainWindow mainWindow;


    std::thread t2(&MainWindow::threadSendCurrentDesiredPoseRobot, &mainWindow);
    std::thread t3(&MainWindow::threadUpdateCurrentDesiredPoseRobot, &mainWindow);
    std::thread t4(&MainWindow::updateDesiredPosition, &mainWindow);
    mainWindow.setStyleSheet("background-color: rgb(177,205,186); ");
    //ROS to GUI
    QObject::connect(&(*nodeShared), &rosHandlerGui::updatePlotPositionVectorROS,
                     &mainWindow, &MainWindow::updateStateForPlotting,Qt::BlockingQueuedConnection);
    QObject::connect(&(*nodeShared), &rosHandlerGui::updateStateOfRobotROS,
                     &mainWindow, &MainWindow::updateStateOfRobot,Qt::AutoConnection);

    QObject::connect(&(*nodeShared), &rosHandlerGui::updatePing360SonarImageROS,
                     &mainWindow, &MainWindow::updatePing360SonarImage, Qt::BlockingQueuedConnection);

    QObject::connect(&(*nodeShared), &rosHandlerGui::updateMicronSonarImageROS,
                     &mainWindow, &MainWindow::updateMicronSonarImage, Qt::BlockingQueuedConnection);


    QObject::connect(&(*nodeShared), &rosHandlerGui::updateCameraImageROS,
                     &mainWindow, &MainWindow::updateCameraImage,Qt::BlockingQueuedConnection);
    QObject::connect(&(*nodeShared), &rosHandlerGui::updateDVLStateROS,
                     &mainWindow, &MainWindow::updateDVLState,Qt::AutoConnection);

    QObject::connect(&(*nodeShared), &rosHandlerGui::updateLeakageStatusTopTubeROS,
                     &mainWindow, &MainWindow::updateLeakageStatusTopTube,Qt::AutoConnection);
    QObject::connect(&(*nodeShared), &rosHandlerGui::updateLeakageStatusSensorTubeROS,
                     &mainWindow, &MainWindow::updateLeakageStatusSensorTube,Qt::AutoConnection);


    //GUI to ROS
    QObject::connect(&mainWindow, &MainWindow::updateDesiredState, &(*nodeShared), &rosHandlerGui::updateDesiredState,Qt::AutoConnection);
    QObject::connect(&mainWindow, &MainWindow::resetEKFEstimator, &(*nodeShared), &rosHandlerGui::resetEKFEstimator,Qt::AutoConnection);
//    QObject::connect(&mainWindow, &MainWindow::updateConfigSonar, &(*nodeShared), &rosHandlerGui::updateConfigSonar,Qt::AutoConnection);
    QObject::connect(&mainWindow, &MainWindow::updateAngleCamera, &(*nodeShared), &rosHandlerGui::updateAngleCamera,Qt::AutoConnection);
    QObject::connect(&mainWindow, &MainWindow::updateLightIntensity, &(*nodeShared), &rosHandlerGui::updateLightIntensity,Qt::AutoConnection);


//    rclcpp::spin(nodeShared);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nodeShared);
//    executor.spin();
    std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));


    mainWindow.showMaximized();

    app.exec();


    rclcpp::shutdown();

//    mainWindow.updateDesiredPosition();
    return 1;
}
