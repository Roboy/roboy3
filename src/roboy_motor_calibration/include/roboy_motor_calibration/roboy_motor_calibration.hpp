#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <roboy_motor_calibration/ui_roboy_motor_calibration.h>
#include <roboy_middleware_msgs/ADCvalue.h>
#include <roboy_middleware_msgs/MotorAngle.h>
#include <roboy_middleware_msgs/MotorCalibrationService.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorState.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QPushButton>
#include <QLineEdit>
#include <QFileDialog>
#include <map>
#include <mutex>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/MotorConfig.hpp>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <boost/thread/thread.hpp>
#include <qcustomplot.h>
#include <QFileInfo>
#include <fstream>
#include <stdlib.h>
#include <chrono>

#endif

using namespace std;
using namespace chrono;

class RoboyMotorCalibration
        : public rqt_gui_cpp::Plugin, MotorConfig {
    Q_OBJECT
public:
    RoboyMotorCalibration();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);
public Q_SLOTS:
    void stopButtonAllClicked();
    void MotorCalibration();
    void plotData();
    void loadConfig();
    void fitCurve();
private:
    void MotorState(const roboy_middleware_msgs::MotorState::ConstPtr &msg);
    void LoadCell(const std_msgs::Float32::ConstPtr &msg);
    /**
	 * Performs polynomial regression (http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/)
	 * @param degree (e.g. 2 -> a * x^0 + b * x^1 + c * x^2)
	 * @param coeffs the estimated coefficients
	 * @param X the x-data
	 * @param Y the y-data
	 */
    void polynomialRegression(int degree, vector<double> &x, vector<double> &y,
                              vector<float> &coeffs);
    /**
     * Estimates the spring parameters
     * @param force vector of samples recorded with load cell
     * @param displacement vector of samples recorded with load cell
     * @param coefficients_displacement_force returns these polynomial parameters
     * @param coefficients_force_displacement returns these polynomial parameters
     */
    void estimateSpringParameters(vector<double> &force,
                                  vector<double> &displacement,
                                  vector<float> &coefficients_displacement_force,
                                  vector<float> &coefficients_force_displacement);

    void estimateMyoBrickSpringParameters();
    void estimateMyoMuscleSpringParameters();
Q_SIGNALS:
    void newData();

private:
    Ui::RoboyMotorCalibration ui;
    QWidget *widget_;
    ros::NodeHandlePtr nh;
    ros::Subscriber motorState, loadCell;
    ros::Publisher motorCommand;
    ros::ServiceClient motorCalibration, emergencyStop;
private:
    mutex mux;
    boost::shared_ptr<boost::thread> calibration_thread;
    QVector<double> time,timeMotorData;
    map<int,vector<float>> coeffs_displacement2force,coeffs_force2displacement;
    int counter = 0, samples_per_plot = 200;
    QVector<double> loadCellValue;
    map<int,QVector<double>> motorData, motorDataCalibrated;
    map<int,double> offset;
    map<int,int> rotationCounter;
    bool stopButton;
    map<string, QPushButton*> button;
    map<string, QLineEdit*> text;
    QString motorConfigFile;
    QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};

    enum{
        POSITION, // this is the raw motor angle
        POSITIONABSOLUT, // this is the motor angle with applied offset
        DISPLACEMENT,
        ANGLE, // this is the winch angle
        ANGLEABSOLUT, // this is the winch angle with applied offset
        SPRING
    };

    enum{
        MYOMUSLCE,
        MUSCLEMUSCLE
    };
};
