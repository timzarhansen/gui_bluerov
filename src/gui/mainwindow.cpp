//
// Created by tim-linux on 14.12.21.
//
#include "mainwindow.h"

#include <utility>


void MainWindow::updateStateForPlotting(std::vector<double> xPositionRobot, std::vector<double> yPositionRobot,
                                        std::vector<double> yawPositionRobot) {

    if (xPositionRobot.size() > 300) {

        int everyNthElement = xPositionRobot.size() / 300;
        this->xPositionRobot = this->keepEveryNthElementWithAverage(xPositionRobot, everyNthElement, 50);
        this->yPositionRobot = this->keepEveryNthElementWithAverage(yPositionRobot, everyNthElement, 50);
        this->yawPositionRobot = this->keepEveryNthElementWithAverage(yawPositionRobot, everyNthElement, 50);
    } else {
        this->xPositionRobot = this->keepEveryNthElementWithAverage(xPositionRobot, 1, 0);
        this->yPositionRobot = this->keepEveryNthElementWithAverage(yPositionRobot, 1, 0);
        this->yawPositionRobot = this->keepEveryNthElementWithAverage(yawPositionRobot, 1, 0);
    }


    //std::nth_element(this->xPositionRobot.begin(),this->xPositionRobot.begin()+5,this->xPositionRobot.end());
//    std::cout << this->xPositionRobot.size() << std::endl;
    double xMin = *std::min_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
    double xMax = *std::max_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
    double yMin = *std::min_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
    double yMax = *std::max_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
    this->customPlot->xAxis->setRange(yMin - 1, yMax + 1);
    this->customPlot->yAxis->setRange(xMin - 1, xMax + 1);
    //std::cout << this->xPositionRobot.size() << std::endl;
    //this->customPlot->clearGraphs();
    this->customPlot->graph(0)->setData(this->yPositionRobot, this->xPositionRobot);
    this->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
    this->customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 2));
    this->customPlot->xAxis->setLabel("y");
    this->customPlot->yAxis->setLabel("x");
//    std::cout << "test" << std::endl;
    this->customPlot->replot();

}

void MainWindow::updatePing360SonarImage(QPixmap sonarImage) {
    this->ping360SonarLabel->setPixmap(
            sonarImage.scaled(ping360SonarLabel->width(), ping360SonarLabel->height(), Qt::KeepAspectRatio));
}


void MainWindow::updateMicronSonarImage(QPixmap sonarImage) {
    this->micronSonarLabel->setPixmap(
            sonarImage.scaled(micronSonarLabel->width(), micronSonarLabel->height(), Qt::KeepAspectRatio));
}


void MainWindow::updateCameraImage(QPixmap cameraImage) {
    this->cameraImageLabel->setPixmap(
            cameraImage.scaled(ping360SonarLabel->width(), ping360SonarLabel->height(), Qt::KeepAspectRatio));
}

//void MainWindow::handleSonarSlider(int sonarRange) {
//
//    QString xstr = QString::number(sonarRange);
//    this->sonarRange = sonarRange;
//    this->currentSonarRange->setText(xstr);
//
//}



//void MainWindow::handleSonarSliderReleased() {
//    //send current distance to the Sonar
////    std::cout << "sonarRange send to Sonar: " << this->sonarRange << std::endl;
//    emit this->updateConfigSonar(this->sonarStepSize, this->sonarRange,this->frequencyRangeValue,this->numberOfSamplesValue);
//}

void MainWindow::handleStrengthXYMovementSlider(int strength) {
    QString xstr = QString::number(((double) strength) / 10.0);
    this->strengthXYMovement = ((double) strength) / 10.0;
//    std::cout << this->strengthXYMovement << std::endl;
    this->currentStrengthXYMovement->setText(xstr);
}




//void MainWindow::handleSonarStepSlider(int angularStepSize) {
//    QString xstr = QString::number(angularStepSize);
//    this->sonarStepSize = angularStepSize;
//    this->currentSonarStepSize->setText(xstr);
//
//}

//void MainWindow::handleSonarStepReleased() {
//    //send current distance to the Sonar
////    std::cout << "sonarStepSize send to Sonar: " << this->sonarStepSize << std::endl;
//    emit this->updateConfigSonar(this->sonarStepSize, this->sonarRange,this->frequencyRangeValue,this->numberOfSamplesValue);
//}

//void MainWindow::handleFrequencyRangeSlider(int angularStepSize) {
//    QString xstr = QString::number(angularStepSize);
//    this->frequencyRangeValue = angularStepSize;
//    this->currentfrequencyRange->setText(xstr);
////    std::cout << "0" << std::endl;
//}

//void MainWindow::handleFrequencyRangeReleased() {
//    //send current distance to the Sonar
////    std::cout << "sonarStepSize send to Sonar: " << this->sonarStepSize << std::endl;
//    emit this->updateConfigSonar(this->sonarStepSize, this->sonarRange,this->frequencyRangeValue,this->numberOfSamplesValue);
//
//}

//void MainWindow::handleNumberOfSamplesSlider(int angularStepSize) {
//    QString xstr = QString::number(angularStepSize);
//    this->numberOfSamplesValue = angularStepSize;
//    this->currentNumberOfSamples->setText(xstr);
////    std::cout << "0" << std::endl;
//}

//void MainWindow::handleNumberOfSamplesReleased() {
//    //send current distance to the Sonar
////    std::cout << "sonarStepSize send to Sonar: " << this->sonarStepSize << std::endl;
//    emit this->updateConfigSonar(this->sonarStepSize, this->sonarRange,this->frequencyRangeValue,this->numberOfSamplesValue);
//
//}



void MainWindow::handleEKFReset() {
    std::cout << "send reset to EKF" << std::endl;
    emit this->resetEKFEstimator(false);

}

void MainWindow::handleResetEKFGraph() {

    emit this->resetEKFEstimator(true);
}

void MainWindow::handleHoldPosition() {
    std::cout << "send hold Position" << std::endl;
    this->holdPositionStatus = not this->holdPositionStatus;

    QPalette pal = this->holdPos->palette();
    if (this->holdPositionStatus) {
        pal.setColor(QPalette::Button, QColor(Qt::green));
    } else {
        pal.setColor(QPalette::Button, QColor(Qt::red));

        double tmpNumber = this->currentYaw;
        this->desiredYaw = tmpNumber;
    }

    this->holdPos->setAutoFillBackground(true);
    this->holdPos->setFlat(true);
    this->holdPos->setPalette(pal);
    this->holdPos->update();



}

void MainWindow::handleControlWithController() {
    std::cout << "send control Robot with Controller" << std::endl;
}

void MainWindow::handleLightSlider(int lightIntensity) {
    QString xstr = QString::number(lightIntensity);
    this->lightIntensity = lightIntensity;
    this->currentLightIntensity->setText(xstr);

}

void MainWindow::handleLightSliderReleased() {
//    std::cout << "send Light to Robot: " << this->lightIntensity << std::endl;
    emit this->updateLightIntensity(this->lightIntensity);
}

void MainWindow::handleCameraAngleSlider(int cameraAngle) {
    QString xstr = QString::number(cameraAngle);
    this->cameraAngleDesired = cameraAngle;
    this->currentCameraAngle->setText(xstr);
}

void MainWindow::handleCameraAngleSliderReleased() {
//    std::cout << "send Camera Angle to Robot: " << this->cameraAngleDesired << std::endl;
    emit this->updateAngleCamera(this->cameraAngleDesired);
}

//move x body axis
void MainWindow::updateRightX(double value) {
//    std::cout << "Right X: " << value << std::endl;
    this->desiredYMovement = 0.5 * this->strengthXYMovement * value;
    QString xstr = "Thrust Y: " + QString::number(this->desiredYMovement, 'f', 2);
    this->currentYThrustLabel->setText(xstr);
}

//move y body axis
void MainWindow::updateRightY(double value) {
//    std::cout << "Right Y: " << value << std::endl;
    this->desiredXMovement = -0.3 * this->strengthXYMovement * value;
    QString xstr = "Thrust X: " + QString::number(this->desiredXMovement, 'f', 2);
    this->currentXThrustLabel->setText(xstr);
}

//yaw rotation
void MainWindow::updateLeftX(double value) {
//    std::cout << "Left X: " << value << std::endl;
    double stepSize = 0.02;
    if (abs(value) > 0.05) {
        this->desiredYaw = this->desiredYaw + stepSize * value;
    }
    //make sure to hold yaw in range of +- pi
    if (this->desiredYaw > M_PI) {
        this->desiredYaw = this->desiredYaw - 2 * M_PI;
    }
    if (this->desiredYaw < -M_PI) {
        this->desiredYaw = this->desiredYaw + 2 * M_PI;
    }
    QString xstr = "Yaw: " + QString::number(this->desiredYaw * 180 / M_PI, 'f', 2);
    this->currentDesiredYawLabel->setText(xstr);
    this->currentDesiredYawLabel->update();
//    std::cout<< "AFTER UPDATE YAW DES" << std::endl;
}

void MainWindow::updateLeftY(double value) { std::cout << "Left Y: " << value << std::endl; }//nothing

//roll +
void MainWindow::updateXButton(bool pressed) {
//    std::cout << "X button Pressed: " << pressed << std::endl;
    if (pressed) {
        this->desiredRoll = this->desiredRoll + 0.01;
        if (this->desiredRoll > M_PI) {
            this->desiredRoll = this->desiredRoll - 2 * M_PI;
        }

        QString xstr = "Roll: " + QString::number(this->desiredRoll * 180 / M_PI, 'f', 2);
        this->currentDesiredRollLabel->setText(xstr);
    }

}

//pitch +
void MainWindow::updateSquareButton(bool pressed) {
//    std::cout << "Square button Pressed: " << pressed << std::endl;
    if (pressed) {
        this->desiredPitch = this->desiredPitch + 0.01;

        if (this->desiredPitch > M_PI / 2) {
            this->desiredPitch = M_PI / 2;
        }
        QString xstr = "Pitch: " + QString::number(this->desiredPitch * 180 / M_PI, 'f', 2);
        this->currentDesiredPitchLabel->setText(xstr);
    }
}

//roll -
void MainWindow::updateCircleButton(bool pressed) {
//    std::cout << "Circle button Pressed: " << pressed << std::endl;
    if (pressed) {
        this->desiredRoll = this->desiredRoll - 0.01;

        if (this->desiredRoll < -M_PI) {
            this->desiredRoll = this->desiredRoll + 2 * M_PI;
        }
        QString xstr = "Roll: " + QString::number(this->desiredRoll * 180 / M_PI, 'f', 2);
        this->currentDesiredRollLabel->setText(xstr);
    }
}

//pitch -
void MainWindow::updateTriangleButton(bool pressed) {
//    std::cout << "Triangle button Pressed: " << pressed << std::endl;
    if (pressed) {
        this->desiredPitch = this->desiredPitch - 0.01;

        if (this->desiredPitch < -M_PI / 2) {
            this->desiredPitch = -M_PI / 2;
        }
        QString xstr = "Pitch: " + QString::number(this->desiredPitch * 180 / M_PI, 'f', 2);
        this->currentDesiredPitchLabel->setText(xstr);
    }
}

//height +
void MainWindow::updateR1Button(bool pressed) {
//    std::cout << "R1 button Pressed: " << pressed << std::endl;

    if (pressed) {
        if (abs(this->desiredHeight - 0.02 - this->currentHeight) < 10.0f) {
            this->desiredHeight = this->desiredHeight - 0.02;
        }
        QString xstr = "Height: " + QString::number(this->desiredHeight, 'f', 2);
        this->currentHeightDesiredLabel->setText(xstr);
    }
}

//height -
void MainWindow::updateR2Button(double pressedValue) {
//    std::cout << "R2 button Pressed: " << pressedValue << std::endl;
    if (pressedValue > 0.2) {
        if (abs(this->desiredHeight + 0.02 - this->currentHeight) < 10.0f) {
            this->desiredHeight = this->desiredHeight + 0.02;
        }
        QString xstr = "Height: " + QString::number(this->desiredHeight, 'f', 2);
        this->currentHeightDesiredLabel->setText(xstr);
    }
}

void MainWindow::updateL1Button(bool pressed) { std::cout << "L1 button Pressed: " << pressed << std::endl; }//nothing

void MainWindow::updateL2Button(bool pressed) { std::cout << "L2 button Pressed: " << pressed << std::endl; }//nothing


void MainWindow::updateStateOfRobot(double xPos, double yPos, double zPos, double roll, double pitch, double yaw,
                                    Eigen::MatrixXd covariance) {
    this->currentHeight = zPos;
    QString xstr = "Height: " + QString::number(this->currentHeight, 'f', 2);
    this->currentPositionZLabel->setText(xstr);

    this->currentRoll = roll;
    this->currentPitch = pitch;
    this->currentYaw = yaw;
    xstr = "Yaw: " + QString::number(this->currentYaw * 180 / M_PI, 'f', 2);
    this->currentPositionYawLabel->setText(xstr);
    this->currentPositionYLabel->update();
//    std::cout<< "AFTER UPDATE YAW CURRENT" << std::endl;

    this->currentXPos = xPos;
    this->currentYPos = yPos;


    xstr = "X: " + QString::number(this->currentXPos, 'f', 2);
    this->currentPositionXLabel->setText(xstr);

    xstr = "Y: " + QString::number(this->currentYPos, 'f', 2);
    this->currentPositionYLabel->setText(xstr);

    xstr = "Roll: " + QString::number(this->currentRoll* 180 / M_PI, 'f', 2);
    this->currentPositionRollLabel->setText(xstr);

    xstr = "Pitch: " + QString::number(this->currentPitch* 180 / M_PI, 'f', 2);
    this->currentPositionPitchLabel->setText(xstr);

}


void MainWindow::updateDVLState(double distance1, double distance2, double distance3, double distance4) {

    QString xstr = "Distance 1:    " + QString::number(distance1, 'f', 2);
    this->currentDistanceDVL1->setText(xstr);
    xstr = "Distance 2:    " + QString::number(distance2, 'f', 2);
    this->currentDistanceDVL2->setText(xstr);
    xstr = "Distance 3:    " + QString::number(distance3, 'f', 2);
    this->currentDistanceDVL3->setText(xstr);
    xstr = "Distance 4:    " + QString::number(distance4, 'f', 2);
    this->currentDistanceDVL4->setText(xstr);
    xstr = "DVL distances:  " + QString::number((distance1 + distance2 + distance3 + distance4) / 4, 'f', 2);
    this->currentDistanceToBottom->setText(xstr);

}

void MainWindow::updateLeakageStatusTopTube(bool leakageStatus) {

    QPalette pal1 = this->leakageTopButton->palette();

    if(leakageStatus){
        pal1.setColor(QPalette::Button, QColor(Qt::red));
    }else{
        pal1.setColor(QPalette::Button, QColor(Qt::green));
    }
    this->leakageTopButton->setAutoFillBackground(true);
    this->leakageTopButton->setFlat(true);
    this->leakageTopButton->setPalette(pal1);
    this->leakageTopButton->update();

}

void MainWindow::updateLeakageStatusSensorTube(bool leakageStatus) {

    QPalette pal1 = this->leakageSensorButton->palette();

    if(leakageStatus){
        pal1.setColor(QPalette::Button, QColor(Qt::red));
    }else{
        pal1.setColor(QPalette::Button, QColor(Qt::green));
    }
    this->leakageSensorButton->setAutoFillBackground(true);
    this->leakageSensorButton->setFlat(true);
    this->leakageSensorButton->setPalette(pal1);
    this->leakageSensorButton->update();

}

