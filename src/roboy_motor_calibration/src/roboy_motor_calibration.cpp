#include <roboy_motor_calibration/roboy_motor_calibration.hpp>

RoboyMotorCalibration::RoboyMotorCalibration()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyMotorCalibration");
}

void RoboyMotorCalibration::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    stopButton = false;

    button["stop_button_all"] = widget_->findChild<QPushButton *>("stop_button_all");
    button["calibrate"] = widget_->findChild<QPushButton *>("calibrate");
    button["load_motor_config"] = widget_->findChild<QPushButton *>("load_motor_config");
    button["fit_curve"] = widget_->findChild<QPushButton *>("fit_curve");
    button["zero_testrig_position"] = widget_->findChild<QPushButton *>("zero_testrig_position");

    button["fit_curve"]->setToolTip("estimates spring parameters read from csv file");

    text["data_points"] = widget_->findChild<QLineEdit *>("data_points");
    text["timeout"] = widget_->findChild<QLineEdit *>("timeout");
    text["degree"] = widget_->findChild<QLineEdit *>("degree");
    text["setpoint_min"] = widget_->findChild<QLineEdit *>("setpoint_min");
    text["setpoint_max"] = widget_->findChild<QLineEdit *>("setpoint_max");
    text["motor_config_path"] = widget_->findChild<QLineEdit *>("motor_config_path");

    text["data_points"]->setToolTip("amount of samples to use for regression");
    text["timeout"]->setToolTip("the calibration will be timed out\n"
                                        "if the number of samples was not reached");
    text["degree"]->setToolTip("degree of the polynomial regression");
    text["setpoint_min"]->setToolTip("minimal/maximal setpoint to be sampled from");
    text["setpoint_max"]->setToolTip("minimal/maximal setpoint to be sampled from");

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_calibration_rqt_plugin");
    }

    string configFile;
    nh->getParam("motorConfigFile", configFile);
    if(!readConfig(configFile)){
        motorConfigFile = QFileDialog::getOpenFileName(widget_,
                                                       tr("Select motor config file"), motorConfigFile,
                                                       tr("motor config file (*.yaml)"));
        if(!readConfig(motorConfigFile.toStdString())){
            ROS_FATAL("could not get any config file, i give up!");
        }
        nh->setParam("motorConfigFile",motorConfigFile.toStdString());
    }else{
        motorConfigFile = QString::fromStdString(configFile);
    }

    ui.stop_button_all->setStyleSheet("background-color: green");
    QObject::connect(button["stop_button_all"], SIGNAL(clicked()), this, SLOT(stopButtonAllClicked()));
    QObject::connect(button["calibrate"], SIGNAL(clicked()), this, SLOT(MotorCalibration()));
    QObject::connect(button["load_motor_config"], SIGNAL(clicked()), this, SLOT(loadConfig()));
    QObject::connect(button["fit_curve"], SIGNAL(clicked()), this, SLOT(fitCurve()));

    // muscleMuscle TAB
    ui.load_2->addGraph();
    ui.load_2->graph(0)->setPen(QPen(color_pallette[0])); // measured
    ui.load_2->addGraph();
    ui.load_2->graph(1)->setPen(QPen(color_pallette[3])); // estimated from sensors

    ui.load_2->xAxis->setLabel("x");
    ui.load_2->yAxis->setLabel("Newton");
    ui.load_2->yAxis->setRange(0, 300);
    ui.load_2->replot();

    ui.winchAngle->addGraph();
    ui.winchAngle->graph(0)->setPen(QPen(color_pallette[1]));
    ui.winchAngle->xAxis->setLabel("x");
    ui.winchAngle->yAxis->setLabel("winch [degree]");
    ui.winchAngle->replot();

    ui.motorAngle->addGraph();
    ui.motorAngle->graph(0)->setPen(QPen(color_pallette[1]));
    ui.motorAngle->xAxis->setLabel("x");
    ui.motorAngle->yAxis->setLabel("motor [degree]");
    ui.motorAngle->replot();

    ui.springAngle->addGraph();
    ui.springAngle->graph(0)->setPen(QPen(color_pallette[2]));
    ui.springAngle->xAxis->setLabel("x");
    ui.springAngle->yAxis->setLabel("spring [degree]");
    ui.springAngle->replot();

    // estimated curva
    ui.displacement_force->addGraph();
    ui.displacement_force->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui.displacement_force->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui.displacement_force->graph(0)->setPen(QPen(color_pallette[0]));
    ui.displacement_force->addGraph();
    ui.displacement_force->graph(1)->setPen(QPen(color_pallette[1]));
    ui.displacement_force->yAxis->setLabel("force[N]");

    ui.force_displacement->addGraph();
    ui.force_displacement->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui.force_displacement->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui.force_displacement->graph(0)->setPen(QPen(color_pallette[0]));
    ui.force_displacement->addGraph();
    ui.force_displacement->graph(1)->setPen(QPen(color_pallette[1]));
    ui.force_displacement->xAxis->setLabel("force[N]");


    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));

    loadConfig();

    motorState = nh->subscribe("/roboy/middleware/MotorState", 1, &RoboyMotorCalibration::MotorState, this);
    loadCell = nh->subscribe("/load_cell", 1, &RoboyMotorCalibration::LoadCell, this);
    motorCalibration = nh->serviceClient<roboy_middleware_msgs::MotorCalibrationService>(
            "/roboy/middleware/MotorCalibration");
    motorCommand = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand", 1);

    ui.progressBar->hide();
}

void RoboyMotorCalibration::shutdownPlugin() {
    // unregister all publishers here
}

void RoboyMotorCalibration::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                         qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyMotorCalibration::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                            const qt_gui_cpp::Settings &instance_settings) {

}

void RoboyMotorCalibration::stopButtonAllClicked() {
    std_srvs::SetBool msg;
    if (button["stop_button_all"]->isChecked()) {
        ui.stop_button_all->setStyleSheet("background-color: red");
        msg.request.data = 1;
        emergencyStop.call(msg);
        button["calibrate"]->setEnabled(false);
    } else {
        ui.stop_button_all->setStyleSheet("background-color: green");
        msg.request.data = 0;
        emergencyStop.call(msg);
        button["calibrate"]->setEnabled(true);
    }
}

void RoboyMotorCalibration::MotorCalibration() {
      ROS_INFO("starting motor calibration for myoBrick");
      ui.displacement_force->xAxis->setLabel("displacement[degree]");
      ui.force_displacement->yAxis->setLabel("displacement[degree]");
      calibration_thread.reset(new boost::thread(&RoboyMotorCalibration::estimateMyoBrickSpringParameters, this));
      calibration_thread->detach();
}

void RoboyMotorCalibration::MotorState(const roboy_middleware_msgs::MotorState::ConstPtr &msg) {
      lock_guard<mutex> lock(mux);
      ROS_INFO_THROTTLE(5, "receiving motor status");
      timeMotorData.push_back(counter);
      motorData[ENCODER0_POSITION].push_back(msg->encoder0_pos[ui.motor->value()]);
      // 1024 ticks per turn / gear box ratio * 360 degrees
      if (motorData[ENCODER0_POSITION].size() > samples_per_plot) {
          motorData[ENCODER0_POSITION].pop_front();
      }

      motorData[ENCODER1_POSITION].push_back(msg->encoder1_pos[ui.motor->value()]);
      if (motorData[ENCODER1_POSITION].size() > samples_per_plot) {
          motorData[ENCODER1_POSITION].pop_front();
      }

      motorData[DISPLACEMENT].push_back(msg->displacement[ui.motor->value()]);
      if (motorData[DISPLACEMENT].size() > samples_per_plot) {
          motorData[DISPLACEMENT].pop_front();
      }

      if (timeMotorData.size() > samples_per_plot)
          timeMotorData.pop_front();

      counter++;

      if (counter % 10 == 0)
            Q_EMIT newData();
}

void RoboyMotorCalibration::LoadCell(const std_msgs::Float32::ConstPtr &msg) {
      ROS_DEBUG_THROTTLE(5, "receiving load_cell status");
      lock_guard<mutex> lock(mux);
      time.push_back(counter++);
      loadCellValue.push_back(msg->data);
      if (loadCellValue.size() > samples_per_plot) {
          loadCellValue.pop_front();
      }
      if (time.size() > samples_per_plot)
          time.pop_front();

      if (counter % 10 == 0)
              Q_EMIT newData();
}

void RoboyMotorCalibration::polynomialRegression(int degree, vector<double> &x, vector<double> &y,
                                                 vector<float> &coeffs) {
    int N = x.size(), i, j, k;
    double X[2 * degree +
             1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i = 0; i < 2 * degree + 1; i++) {
        X[i] = 0;
        for (j = 0; j < N; j++)
            X[i] = X[i] + pow(x[j],
                              i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    double B[degree + 1][degree + 2], a[degree +
                                        1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i = 0; i <= degree; i++)
        for (j = 0; j <= degree; j++)
            B[i][j] = X[i +
                        j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double Y[degree +
             1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^degree*yi)
    for (i = 0; i < degree + 1; i++) {
        Y[i] = 0;
        for (j = 0; j < N; j++)
            Y[i] = Y[i] + pow(x[j], i) *
                          y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i = 0; i <= degree; i++)
        B[i][degree +
             1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    degree = degree +
             1;                //degree is made degree+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
    for (i = 0; i <
                degree; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k = i + 1; k < degree; k++)
            if (B[i][i] < B[k][i])
                for (j = 0; j <= degree; j++) {
                    double temp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = temp;
                }

    for (i = 0; i < degree - 1; i++)            //loop to perform the gauss elimination
        for (k = i + 1; k < degree; k++) {
            double t = B[k][i] / B[i][i];
            for (j = 0; j <= degree; j++)
                B[k][j] = B[k][j] - t *
                                    B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
        }
    for (i = degree - 1; i >= 0; i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i] = B[i][degree];                //make the variable to be calculated equal to the rhs of the last equation
        for (j = 0; j < degree; j++)
            if (j !=
                i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i] = a[i] - B[i][j] * a[j];
        a[i] = a[i] /
               B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }
    for (i = 0; i < degree; i++)
        coeffs.push_back(a[i]);    //the values of x^0,x^1,x^2,x^3,....
}

void RoboyMotorCalibration::estimateSpringParameters(vector<double> &force,
                                                     vector<double> &displacement,
                                                     vector<float> &coefficients_displacement_force,
                                                     vector<float> &coefficients_force_displacement) {
    polynomialRegression(ui.degree->text().toInt(), displacement, force, coefficients_displacement_force);

    QVector<double> load = QVector<double>::fromStdVector(force);
    QVector<double> dis = QVector<double>::fromStdVector(displacement);

    ui.displacement_force->graph(0)->setData(dis, load);

    QVector<double> load_graph;
    for (uint i = 0; i < displacement.size(); i++) {
        double val = coefficients_displacement_force[0];
        for (uint j = 1; j < coefficients_displacement_force.size(); j++) {
            val += coefficients_displacement_force[j] * pow(displacement[i], (double) j);
        }
        load_graph.push_back(val);
    }

    ui.displacement_force->graph(1)->setData(dis, load_graph);
    ui.displacement_force->graph(0)->rescaleAxes();
    ui.displacement_force->replot();

    polynomialRegression(ui.degree->text().toInt(), force, displacement, coefficients_force_displacement);

    ui.force_displacement->graph(0)->setData(load, dis);

    QVector<double> displacement_graph;
    for (uint i = 0; i < displacement.size(); i++) {
        double val = coefficients_force_displacement[0];
        for (uint j = 1; j < coefficients_force_displacement.size(); j++) {
            val += coefficients_force_displacement[j] * pow(force[i], (double) j);
        }
        displacement_graph.push_back(val);
    }

    ui.force_displacement->graph(1)->setData(load, displacement_graph);
    ui.force_displacement->graph(0)->rescaleAxes();
    ui.force_displacement->replot();
}

void RoboyMotorCalibration::estimateMyoMuscleSpringParameters() {
    if(!ui.calibrate->isChecked())
        return;
    milliseconds ms_start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()), ms_stop, t0, t1;
    ofstream outfile;
    char str[100];
    sprintf(str, "springParameters_calibration_fpga%d_motor%d.csv", ui.fpga->value(), ui.motor->value());
    outfile.open(str);
    if (!outfile.is_open()) {
        cout << "could not open file " << str << " for writing, aborting!" << endl;
        return;
    }
    outfile << "springAngle[degree], load[N]" << endl;
    float setpoint_max = ui.setpoint_max->text().toFloat(), setpoint_min = ui.setpoint_min->text().toFloat();
    int timeout = ui.timeout->text().toInt(), degree = ui.degree->text().toInt(), numberOfDataPoints = ui.data_points->text().toInt();
    roboy_middleware_msgs::MotorCommand msg;
    msg.motor.push_back(ui.motor->value());
    ui.progressBar->show();
    ui.progressBar->setMaximum(0);
    ui.progressBar->setMinimum(0);

    vector<double> x, y;
    float f = setpoint_min;
    bool up = true;
    do {
//        float f = (rand() / (float) RAND_MAX) * (setpoint_max - setpoint_min) + setpoint_min;
        msg.setpoint.clear();
        msg.setpoint.push_back(f);
        motorCommand.publish(msg);
        t0 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        do {// wait a bit until force is applied
            // update control
            t1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        } while ((t1 - t0).count() < 300);

        x.push_back(motorData[DISPLACEMENT].back());
        y.push_back(loadCellValue.back());

        outfile << x.back() << ", " << y.back() << endl;
        ms_stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        cout << "setPoint: \t" << f << "\tmotorSpring:\t" << x.back() << "\tload:\t" << y.back() << endl;
        if(f>setpoint_max && up)
            up = false;
        if(f<0 && !up)
            up= true;
        f += up?1:-1;

    } while ((ms_stop - ms_start).count() < timeout && x.size() < numberOfDataPoints && ui.calibrate->isChecked());

    ui.progressBar->hide();

    coeffs_displacement2force[ui.motor->value()].clear();
    polynomialRegression(degree, x, y, coeffs_displacement2force[ui.motor->value()]);
    // coefficients_displacement_force
    outfile << "regression coefficients_displacement_force for polynomial of " << degree << " degree:" << endl;
    cout << "regression coefficients_displacement_force for polynomial of " << degree << " degree:" << endl;
    for (float coef:coeffs_displacement2force[ui.motor->value()]) {
        outfile << coef << "\t";
        cout << coef << "\t";
    }
    outfile << endl;
    cout << endl;
    // coefficients_force_displacement
    coeffs_force2displacement.clear();
    polynomialRegression(degree, y, x, coeffs_force2displacement[ui.motor->value()]);
    outfile << "regression coefficients_displacement_force for polynomial of " << degree << " degree:" << endl;
    cout << "regression coefficients_displacement_force for polynomial of " << degree << " degree:" << endl;
    for (float coef:coeffs_force2displacement[ui.motor->value()]) {
        outfile << coef << "\t";
        cout << coef << "\t";
    }
    outfile << endl;
    cout << endl;

//	polyPar[motor] = coeffs;
    outfile.close();

    QVector<double> dis = QVector<double>::fromStdVector(x);
    QVector<double> load = QVector<double>::fromStdVector(y);

    ui.displacement_force->graph(0)->setData(dis, load);

    QVector<double> load_graph;
    for (uint i = 0; i < x.size(); i++) {
        // load_graph.push_back(displacement2force(dis[i],ui.motor->value()));
    }

    ui.displacement_force->graph(1)->setData(dis, load_graph);
    ui.displacement_force->graph(1)->rescaleAxes();
    ui.displacement_force->replot();

    ui.force_displacement->graph(0)->setData(load, dis);

    QVector<double> displacement_graph;
    for (uint i = 0; i < x.size(); i++) {
        // displacement_graph.push_back(force2displacement(y[i],ui.motor->value()));
    }

    ui.force_displacement->graph(1)->setData(load, displacement_graph);
    ui.force_displacement->graph(1)->rescaleAxes();
    ui.force_displacement->replot();

    writeConfig(text["motor_config_path"]->text().toStdString());
}

void RoboyMotorCalibration::estimateMyoBrickSpringParameters() {
    milliseconds ms_start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()), ms_stop, t0, t1;
    ofstream outfile;
    char str[100];
    sprintf(str, "springParameters_calibration_motor%d.csv", ui.motor->value());
    outfile.open(str);
    if (!outfile.is_open()) {
        cout << "could not open file " << str << " for writing, aborting!" << endl;
        return;
    }
    outfile << "springAngle[degree], load[N]" << endl;
    float setpoint_max = ui.setpoint_max->text().toFloat(), setpoint_min = ui.setpoint_min->text().toFloat();
    int timeout = ui.timeout->text().toInt(), degree = ui.degree->text().toInt(), numberOfDataPoints = ui.data_points->text().toInt();
    roboy_middleware_msgs::MotorCommand msg;
    msg.motor.push_back(ui.motor->value());

    vector<double> x, y;

    do {
        float f = (rand() / (float) RAND_MAX) * (setpoint_max - setpoint_min) + setpoint_min;
        msg.setpoint.clear();
        msg.setpoint.push_back(f);
        motorCommand.publish(msg);
        t0 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        do {// wait a bit until force is applied
            // update control
            t1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        } while ((t1 - t0).count() < 1500);

        x.push_back(motorData[DISPLACEMENT].back());
        y.push_back(loadCellValue.back());

        outfile << x.back() << ", " << y.back() << endl;
        ms_stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        cout << "setPoint: \t" << f << "\tmotorSpring:\t" << x.back() << "\tload:\t" << y.back() << endl;
    } while ((ms_stop - ms_start).count() < timeout && x.size() < numberOfDataPoints);

    vector<float> coefficients_displacement_force, coefficients_force_displacement;
    polynomialRegression(degree, x, y, coefficients_displacement_force);
    // coefficients_displacement_force
    outfile << "regression coefficients_displacement_force for polynomial of " << degree << " degree:" << endl;
    cout << "regression coefficients_displacement_force for polynomial of " << degree << " degree:" << endl;
    for (float coef:coefficients_displacement_force) {
        outfile << coef << "\t";
        cout << coef << "\t";
    }
    outfile << endl;
    cout << endl;
    // coefficients_force_displacement
    polynomialRegression(degree, y, x, coefficients_force_displacement);
    outfile << "regression coefficients_displacement_force for polynomial of " << degree << " degree:" << endl;
    cout << "regression coefficients_displacement_force for polynomial of " << degree << " degree:" << endl;
    for (float coef:coefficients_displacement_force) {
        outfile << coef << "\t";
        cout << coef << "\t";
    }
    outfile << endl;
    cout << endl;

//	polyPar[motor] = coeffs;
    outfile.close();

    QVector<double> dis = QVector<double>::fromStdVector(x);
    QVector<double> load = QVector<double>::fromStdVector(y);

    ui.displacement_force->graph(0)->setData(dis, load);

    QVector<double> load_graph;
    for (uint i = 0; i < x.size(); i++) {
        double val = coefficients_displacement_force[0];
        for(uint j = 1; j < coefficients_displacement_force.size(); j++) {
            val += coefficients_displacement_force[j] * pow(dis[i], (double) j);
        }
        load_graph.push_back(val);
    }

    ui.displacement_force->graph(1)->setData(dis, load_graph);
    ui.displacement_force->graph(1)->rescaleAxes();
    ui.displacement_force->replot();

    ui.force_displacement->graph(0)->setData(load, dis);

    QVector<double> displacement_graph;
    for (uint i = 0; i < x.size(); i++) {
        double val = coefficients_force_displacement[0];
        for (uint j = 1; j < coefficients_force_displacement.size(); j++) {
            val += coefficients_force_displacement[j] * pow(y[i], (double) j);
        }
        displacement_graph.push_back(val);
    }

    ui.force_displacement->graph(1)->setData(load, displacement_graph);
    ui.force_displacement->graph(1)->rescaleAxes();
    ui.force_displacement->replot();

    writeConfig(text["motor_config_path"]->text().toStdString());
}

void RoboyMotorCalibration::plotData() {
    lock_guard<mutex> lock(mux);

    ui.load_2->graph(0)->setData(time, loadCellValue);
//            ui.load_2->graph(1)->setData(time, motorDataCalibrated);
    ui.load_2->xAxis->rescale();

    ui.winchAngle->graph(0)->setData(timeMotorData, motorData[ENCODER1_POSITION]);
    ui.winchAngle->graph(0)->rescaleAxes();

    ui.motorAngle->graph(0)->setData(timeMotorData, motorData[ENCODER0_POSITION]);
    ui.motorAngle->graph(0)->rescaleAxes();

    ui.springAngle->graph(0)->setData(timeMotorData, motorData[DISPLACEMENT]);
    ui.springAngle->graph(0)->rescaleAxes();

    ui.load_2->replot();
    ui.winchAngle->replot();
    ui.motorAngle->replot();
    ui.springAngle->replot();
}

void RoboyMotorCalibration::loadConfig() {
    QFileInfo check_file(text["motor_config_path"]->text());
    if (check_file.exists() && check_file.isFile()) {
        ROS_INFO("reading motor config from %s", text["motor_config_path"]->text().toStdString().c_str());
        readConfig(text["motor_config_path"]->text().toStdString());
    } else {
        ROS_ERROR("file %s does not exist, check your path", text["motor_config_path"]->text().toStdString().c_str());
    }

}

void RoboyMotorCalibration::fitCurve() {
    char str[200];
    sprintf(str, "About to overwrite existing spring parameters for fpga %d motor %d", ui.fpga->value(), ui.motor->value());
    QMessageBox::information(widget_, tr(str),
                             "Make sure you have selected the correct motor and fpga in the GUI");

    QString fileName = QFileDialog::getOpenFileName(widget_, tr("Open Calibration CSV file"), "",
                                                    tr("Calibration Data (*.csv);;All Files (*)"));
    if (fileName.isEmpty()) {
        ROS_ERROR("could not open file");
        return;
    } else {
        vector<double> dis, force;

        ifstream infile(fileName.toStdString().c_str());
        string line;
        while (getline(infile, line)) {
            char *pt;
            pt = strtok((char *) line.c_str(), ",");
            bool displacement = true;
            float val0, val1;
            int counter = 0;
            while (pt != NULL) {
                counter++;
                std::istringstream istr(pt);
                istr.imbue(std::locale("C"));
                if (displacement) {
                    istr >> val0;
                    displacement = false;
                } else {
                    istr >> val1;
                }
                pt = strtok(NULL, ",");
            }
            if (counter == 2) {
                dis.push_back(val0);
                force.push_back(val1);
            }
        }
        coeffs_displacement2force[ui.motor->value()].clear();
        coeffs_force2displacement[ui.motor->value()].clear();
        estimateSpringParameters(force, dis,
                                 coeffs_displacement2force[ui.motor->value()],
                                 coeffs_force2displacement[ui.motor->value()]);
        writeConfig(text["motor_config_path"]->text().toStdString());
    }
}

PLUGINLIB_EXPORT_CLASS(RoboyMotorCalibration, rqt_gui_cpp::Plugin)
