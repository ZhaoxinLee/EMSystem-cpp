#include "mainwindow.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CONSTRUCTOR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) , ui(new Ui::MainWindow)
{
    // Create Window from UI
    ui->setupUi(this);

    qInfo() << " ";
    qInfo() << "Setting up the GUI now.";

    // Connect SIGNALS and SLOTS
    // Here is where you should add connections if buttons or checkboxes are added to the UI.

    // CHECKBOXES


    connect(ui->checkBox_udpmode,SIGNAL(clicked()),SLOT(enableUDP()));
    connect(ui->checkBox_sampleBxy,SIGNAL(clicked()),SLOT(enablesampleBxy()));

    connect(ui->checkBox_gradientControlled,SIGNAL(clicked()),SLOT(enableGradientControl()));
    connect(ui->checkBox_controllerEnable,SIGNAL(clicked()),SLOT(enableController()));
    connect(ui->checkBox_startSubthread,SIGNAL(clicked()),SLOT(enableSubroutine()));
    connect(ui->checkBox_angleControlSurgeon,SIGNAL(clicked()),SLOT(enableAngleControl()));
    connect(ui->checkBox_enableMagneticPlot,SIGNAL(clicked()),SLOT(enableMagneticPlot()));
    connect(ui->checkBox_showGradients,SIGNAL(clicked()),SLOT(enableGradientPlot()));
    connect(ui->checkBox_enableMagneticLegend,SIGNAL(clicked()),SLOT(enableMagneticLegend()));
    connect(ui->checkBox_enableInputPlot,SIGNAL(clicked()),SLOT(enableInputPlot()));
    connect(ui->checkBox_showSetpoints,SIGNAL(clicked()),SLOT(enableSetpointPlot()));
    connect(ui->checkBox_enableInputLegend,SIGNAL(clicked()),SLOT(enableInputLegend()));
    connect(ui->checkBox_enableTemperatureLegend,SIGNAL(clicked()),SLOT(enableTemperatureLegend()));
    connect(ui->checkBox_enableTemperaturePlot,SIGNAL(clicked()),SLOT(enableTemperaturePlot()));
//    connect(ui->checkBox_enableAnglesPlot,SIGNAL(clicked()),SLOT(enableAnglesPlot())); //remove
    connect(ui->checkBox_enableDAQPlot,SIGNAL(clicked()),SLOT(enableDAQPlot()));
    connect(ui->checkBox_enableDAQ,SIGNAL(clicked()),SLOT(enableDAQ()));
    connect(ui->checkBox_enableDAQLegend,SIGNAL(clicked()),SLOT(enableDAQLegend()));
    // SCROLL BOXES
    connect(ui->comboBox_controllerMode,SIGNAL(currentIndexChanged (int)),SLOT( controllerModeSlot(int)) );
    connect(ui->comboBox_subThread,SIGNAL(currentIndexChanged (int)),SLOT( subThreadModeSlot(int)) );

    // SPIN BOXES
    // type double
    connect(ui->doubleSpinBox_tilt,SIGNAL(valueChanged(double)),SLOT( setTiltSlot() ) );
    connect(ui->doubleSpinBox_roll,SIGNAL(editingFinished()),SLOT( setRollSlot() ) ); //valueChanged(double)
    connect(ui->doubleSpinBox_factor,SIGNAL(valueChanged(double)),SLOT( setFactorSlot() ) );
    connect(ui->doubleSpinBox_Bx_Global,SIGNAL(editingFinished()),SLOT( setFieldSpinBoxSlot() ) );
    connect(ui->doubleSpinBox_By_Global,SIGNAL(editingFinished()),SLOT( setFieldSpinBoxSlot() ) );
    connect(ui->doubleSpinBox_Bz_Global,SIGNAL(editingFinished()),SLOT( setFieldSpinBoxSlot() ) );
    connect(ui->doubleSpinBox_Bx_Local,SIGNAL(editingFinished()),SLOT( setFieldSpinBoxSlot() ) );
    connect(ui->doubleSpinBox_By_Local,SIGNAL(editingFinished()),SLOT( setFieldSpinBoxSlot() ) );
    connect(ui->doubleSpinBox_Bz_Local,SIGNAL(editingFinished()),SLOT( setFieldSpinBoxSlot() ) );
    connect(ui->dsb_subThreadParam0,SIGNAL(editingFinished()),SLOT( subThreadParameterChangeSlot() ) );
    connect(ui->dsb_subThreadParam1,SIGNAL(editingFinished()),SLOT( subThreadParameterChangeSlot() ) );
    connect(ui->dsb_subThreadParam2,SIGNAL(editingFinished()),SLOT( subThreadParameterChangeSlot() ) );
    connect(ui->dsb_subThreadParam3,SIGNAL(editingFinished()),SLOT( subThreadParameterChangeSlot() ) );
    connect(ui->dsb_subThreadParam4,SIGNAL(editingFinished()),SLOT( subThreadParameterChangeSlot() ) );
    connect(ui->doubleSpinBox_gantryAbsMM_X,SIGNAL(editingFinished()),SLOT( gantryMoveAbsolute() ));
    connect(ui->doubleSpinBox_gantryAbsMM_Y,SIGNAL(editingFinished()),SLOT( gantryMoveAbsolute() ));
    connect(ui->doubleSpinBox_gantryAbsMM_Z,SIGNAL(editingFinished()),SLOT( gantryMoveAbsolute() ));
    // type int
    connect(ui->spinBox_gantryJogSteps,SIGNAL(valueChanged(int)),SLOT( setGantryJogSteps() ) );
    connect(ui->spinBox_gantryAbsSteps_X,SIGNAL(editingFinished()),SLOT( gantryMoveAbsolute() ));
    connect(ui->spinBox_gantryAbsSteps_Y,SIGNAL(editingFinished()),SLOT( gantryMoveAbsolute() ));
    connect(ui->spinBox_gantryAbsSteps_Z,SIGNAL(editingFinished()),SLOT( gantryMoveAbsolute() ));


    // HORIZONTAL SLIDERS
    connect(ui->horizontalSlider_Bx_Global,SIGNAL(sliderMoved(int)), SLOT( sliderMovingSlot(int) ) );
    connect(ui->horizontalSlider_By_Global,SIGNAL(sliderMoved(int)), SLOT( sliderMovingSlot(int) ) );
    connect(ui->horizontalSlider_Bz_Global,SIGNAL(sliderMoved(int)), SLOT( sliderMovingSlot(int) ) );
    connect(ui->horizontalSlider_Bx_Local,SIGNAL(sliderMoved(int)), SLOT( sliderMovingSlot(int) ) );
    connect(ui->horizontalSlider_By_Local,SIGNAL(sliderMoved(int)), SLOT( sliderMovingSlot(int) ) );
    connect(ui->horizontalSlider_Bz_Local,SIGNAL(sliderMoved(int)), SLOT( sliderMovingSlot(int) ) );

    // PUSH BUTTONS
    connect(ui->pushButton_clearCurrents_Global,SIGNAL(clicked()),SLOT(clearCurrentsSlot()));
    connect(ui->pushButton_clearCurrents_Local,SIGNAL(clicked()),SLOT(clearCurrentsSlot()));
    connect(ui->pushButton_clearTime,SIGNAL(clicked()),SLOT(clearTimeSlot()));
    connect(ui->pushButton_ToolIN,SIGNAL(pressed()),SLOT(ToolIN_press_Slot()));
    connect(ui->pushButton_ToolIN,SIGNAL(released()),SLOT(ToolINOUT_rel_Slot()));
    connect(ui->pushButton_ToolOUT,SIGNAL(pressed()),SLOT(ToolOUT_press_Slot()));
    connect(ui->pushButton_ToolOUT,SIGNAL(released()),SLOT(ToolINOUT_rel_Slot()));
    connect(ui->pushButton_reconnectGamepad,SIGNAL(clicked()),SLOT(reconnectGamepadSlot()));
    connect(ui->pushButton_reconnectS826,SIGNAL(clicked()),SLOT(reconnectS826()));
    connect(ui->pushButton_StartRecordDAQ,SIGNAL(released()),SLOT(setDAQRecordingState()));
    connect(ui->pushButton_ClibrationRecord,SIGNAL(released()),SLOT(setDAQRecordingState()));
    connect(ui->pushButton_StopRecordDAQ,SIGNAL(released()),SLOT(stopDAQRecording()));
    connect(ui->pushButton_openDiscreteDataFile,SIGNAL(released()),SLOT(setDAQRecordingState()));
    connect(ui->pushButton_closeDiscreteDataFile,SIGNAL(released()),SLOT(stopDAQRecording()));
    connect(ui->pushButton_recordDiscreteDatapoint,SIGNAL(released()),SLOT(recordDAQDatapoint()));

    connect(ui->pushButton_gantryZero,SIGNAL(clicked()),SLOT(gantryZero()));
    connect(ui->pushButton_gantryCenter,SIGNAL(clicked()),SLOT(gantryCenter()));
    connect(ui->pushButton_gantryNegX,SIGNAL(clicked()),SLOT(gantryJog()));
    connect(ui->pushButton_gantryPosX,SIGNAL(clicked()),SLOT(gantryJog()));
    connect(ui->pushButton_gantryNegY,SIGNAL(clicked()),SLOT(gantryJog()));
    connect(ui->pushButton_gantryPosY,SIGNAL(clicked()),SLOT(gantryJog()));
    connect(ui->pushButton_gantryNegZ,SIGNAL(clicked()),SLOT(gantryJog()));
    connect(ui->pushButton_gantryPosZ,SIGNAL(clicked()),SLOT(gantryJog()));

    connect(ui->pushButton_sampleBxy, SIGNAL(clicked()),SLOT(sampleBxy()));


    // TIMERS
    // Define the timer for updating the graphics and readouts on the GUI
    QTimer *captiontimer = new QTimer(this);
    connect(captiontimer, SIGNAL(timeout()), this, SLOT(updateCaption()));	// show fps,... in timer
    captiontimer->start(captionRefreshPeriod); //Default 200. Set to 20 ms for a faster 50Hz refresh rate
    // Define the timer for executing runtime code know as the callbacks.
    QTimer *callbacktimer = new QTimer(this);
    connect(callbacktimer, SIGNAL(timeout()), this, SLOT(callbacks()));
    callbacktimer->start(callbackRefreshPeriod); //Default 200. Set to 20 ms for a faster 50Hz refresh rate
//    QElapsedTimer currentTime;
    // Start the timer to keep track of the current elapsed time/
    currentTime.start();
    lastTime = currentTime.elapsed();


    // TODO: ensure all booleans and variables are initialized from the GUI on startup

    // Initialize Boolean Variables from GUI defaults
    isGradientControlled = ui->checkBox_gradientControlled->checkState();
    controllerState = ui->checkBox_controllerEnable->checkState();
    // Check plotting Booleans
    magneticPlotState = ui->checkBox_enableMagneticPlot->checkState();
    inputPlotState = ui->checkBox_enableInputPlot->checkState();
    temperaturePlotState = ui->checkBox_enableTemperaturePlot->checkState();
    DAQPlotState = ui->checkBox_enableDAQPlot->checkState();
    // Check subplotting Booleans
    gradientPlotState = ui->checkBox_showGradients->checkState();
    setpointPlotState = ui->checkBox_showSetpoints->checkState();    
    // Check plot legends Booleans
    magneticLegendState = ui->checkBox_enableMagneticLegend->checkState();
    inputLegendState = ui->checkBox_enableInputLegend->checkState();
    temperatureLegendState = ui->checkBox_enableTemperatureLegend->checkState();

    enableAngleControlState = ui->checkBox_angleControlSurgeon->checkState();
    currentControllerMode = ui->comboBox_controllerMode->currentIndex();

    // Initialize Parameter Values from GUI defaults
    tilt = ui->doubleSpinBox_tilt->value();
    roll = ui->doubleSpinBox_roll->value();
    k_factor = ui->doubleSpinBox_factor->value() / 1000;
    gantryJogSteps = ui->spinBox_gantryJogSteps->value();

    // Setup the plots for the fields and actuator inputs
    setupPlots();

    // Initialize Magnetic Properties (Permanent Magnet System):
    // Calculate Control Matrices for both types of control if not using calibrated values
    if (!useCalibratedFieldValues)
    {
        calcIsotropicControlMatrix(pAct_cartesion, mAct_cartesion, N);
        calcAnisotropicControlMatrix(pAct_cartesion, mAct_cartesion, M);
    } // else use preset values in header file
    // Note that the psuedo inverse of these matrices are determined every control loop call
    // which is not necessary for our purposes, but eventually when the control matrix needs
    // to change as a function of the tool position, this will be necessary.


    // Determine the max magnetic fields for these directions
    maxIsotropicField(pAct_cartesion, mAct_cartesion, maxIsoFields);
    maxAnisotropicField(pAct_cartesion, mAct_cartesion, maxAnisoFields);
    // Call function to set all sliders and scrollboxes limits
    // This also initializes maxB variable
    enableGradientControl();

    qDebug() << "The max Theoretical B Field is " << maxB*1000 << "mT";

////    for (int n=3;n<8;n++) // For full 8 magnet system
////    {
////        B_Global_Desired_Input[n] = minAllowableGrad;
////    }



    // temporary
//    maxB = 0.030;


    // These have already been set once above using enableGradientControl()

    // Once the Maximum possible magnetic field is found we need to initialize
    // the scrollboxes' and sliders' max range for the Global Field to not go
    // outside the field limits
    ui->doubleSpinBox_Bx_Global->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->doubleSpinBox_By_Global->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->doubleSpinBox_Bz_Global->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->horizontalSlider_Bx_Global->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps
    ui->horizontalSlider_By_Global->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps
    ui->horizontalSlider_Bz_Global->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps

    // We also need to initialize the scrollboxes' and sliders' max range
    // for the Local Field to not go outside the field limits
    ui->doubleSpinBox_Bx_Local->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->doubleSpinBox_By_Local->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->doubleSpinBox_Bz_Local->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->horizontalSlider_Bx_Local->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps
    ui->horizontalSlider_By_Local->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps
    ui->horizontalSlider_Bz_Local->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps


    QCommonStyle style;
    ui->pushButton_gantryNegX->setIcon(style.standardIcon(QStyle::SP_ArrowUp));
    ui->pushButton_gantryPosX->setIcon(style.standardIcon(QStyle::SP_ArrowDown));
    ui->pushButton_gantryNegY->setIcon(style.standardIcon(QStyle::SP_ArrowLeft));
    ui->pushButton_gantryPosY->setIcon(style.standardIcon(QStyle::SP_ArrowRight));
    ui->pushButton_gantryNegZ->setIcon(style.standardIcon(QStyle::SP_ArrowDown));
    ui->pushButton_gantryPosZ->setIcon(style.standardIcon(QStyle::SP_ArrowUp));

    // I don't know how to set these in the GUI editor so here is the code for it:
    ui->pushButton_StartRecordDAQ->setIcon(style.standardIcon(QStyle::SP_MediaPlay));
    ui->pushButton_StopRecordDAQ->setIcon(style.standardIcon(QStyle::SP_MediaStop));
    ui->label_DAQRecordIcon->setHidden(true);

    // Set default labels to subwindow based on initial control mode
    controllerModeSlot(currentControllerMode);

    // Set up the game controller.
    // This work better here than it does in the objects constructor.
    // Probably due to a timing issue and load on the computer during launch.
    connectedGamepad.reconnectController();

    // Set up the S826 board.
    int errcode = s826.init();
    qInfo() << "S826.init() returned the value: " << errcode;
    qInfo() << " ";

    // Initialize GUI captions and readaouts based on setup labels
    updateCaption();

    /// CONNECT TO ARDUINOS
    qInfo() << "Connecting to Arduinos...";
    /*
     *  Testing code, prints the description, vendor id, and product id of all ports.
     *  Used it to determine the values for the arduino uno.
     *
     */
    qDebug() << "Number of ports: " << QSerialPortInfo::availablePorts().length() << "\n";
    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        qDebug() << "Description: " << serialPortInfo.description() << "\n";
        qDebug() << "Has vendor id?: " << serialPortInfo.hasVendorIdentifier() << "\n";
        qDebug() << "Vendor ID: " << serialPortInfo.vendorIdentifier() << "\n";
        qDebug() << "Has product id?: " << serialPortInfo.hasProductIdentifier() << "\n";
        qDebug() << "Product ID: " << serialPortInfo.productIdentifier() << "\n";
    }

    // Code to automatically detect the connected arduinos (based on vendor and product IDs):

    // Identify the port the arduino uno is on.
    QString arduinoUnoPortName;
    QString arduinoMegaPortName;
    //  For each available serial port
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        //  check if the serialport has both a product identifier and a vendor identifier
        if (serialPortInfo.hasProductIdentifier() && serialPortInfo.hasVendorIdentifier())
        {
            //  check if the product ID and the vendor ID match those of the arduino Uno
            if((serialPortInfo.productIdentifier() == arduinoUnoProductID)
                    && (serialPortInfo.vendorIdentifier() == arduinoUnoVendorID))
            {
                trocarConnected = true; //    arduino uno is available on this port
                arduinoUnoPortName = serialPortInfo.portName();
            }
            //  check if the product ID and the vendor ID match those of the arduino Mega
            if((serialPortInfo.productIdentifier() == arduinoMegaProductID)
                    && (serialPortInfo.vendorIdentifier() == arduinoMegaVendorID))
            {
                gantryConnected = true; //    arduino uno is available on this port
                arduinoMegaPortName = serialPortInfo.portName();
                //qDebug()<<"gantryConnected";
            }
        }
    }

    /*
    Mapping b/w Ubuntu and Windows:
    Ubuntu:  writeport = "/dev/ttyACM"
    Windows: writeport = "COM"
    Ubuntu      |   Windows
    0           |   6
    1           |   3
    2           |   4
    3           |   5
    4 (Trocar)  |   7
                |   8
    */
//    std::string writeportTrocar = "COM6";
    // Open and configure the arduino port if available
    if (trocarConnected)
    {
        qDebug() << "Found the Arduino Uno port for the Trocar...\n";
//        serial[0].setPortName(QString::fromStdString(writeportTrocar));
        serial[0].setPortName(arduinoUnoPortName);
        serial[0].setBaudRate(QSerialPort::Baud115200);
        serial[0].setDataBits(QSerialPort::Data8);
        serial[0].setParity(QSerialPort::NoParity);
        serial[0].setStopBits(QSerialPort::OneStop);
        serial[0].setFlowControl(QSerialPort::NoFlowControl);
//        serial[0].open(QIODevice::ReadWrite);
//        QObject::connect(serial[0], SIGNAL(readyRead()), this, SLOT(readSerial()));
        serial[0].open(QIODevice::WriteOnly);
    }
    else
    {
        qDebug() << "Couldn't find the correct port for the Trocar.\n";
//        QMessageBox::information(this, "Serial Port Error", "Couldn't open serial port to Arduino Uno.");
    }

    // Add a serial port for the Gantry System
    if (gantryConnected)
    {
        qDebug() << "Found the Arduino Mega port for the Gantry...\n";
//        std::string writeportGantry = "COM3";
//        serial[1].setPortName(QString::fromStdString(writeportGantry));
        serial[1].setPortName(arduinoMegaPortName);
        serial[1].setBaudRate(QSerialPort::Baud115200);
        serial[1].setDataBits(QSerialPort::Data8);
        serial[1].setParity(QSerialPort::NoParity);
        serial[1].setStopBits(QSerialPort::OneStop);
        serial[1].setFlowControl(QSerialPort::NoFlowControl);
//        serial[1].open(QIODevice::ReadWrite);
        serial[1].open(QIODevice::WriteOnly);
    }
    else
    {
        qDebug() << "Couldn't find the correct port for the Gantry.\n";
//        QMessageBox::information(this, "Serial Port Error", "Couldn't open serial port to Arduino Mega.");
    }


//    // Connect to motors
//    std::string writeport = "/dev/ttyACM";
//    arduinoMotor0.open( writeport+"0" );
//    arduinoMotor1.open( writeport+"1" );
//    arduinoMotor2.open( writeport+"2" );
//    arduinoMotor3.open( writeport+"3" );



    qInfo() << " ";
    qInfo() << "Program initiation executed. Running program...";
    qInfo() << " ";

    socket = new QUdpSocket(this);
    bool result =  socket->bind(QHostAddress::AnyIPv4, SERVER_PORT);
    qDebug() << result;
    if(result)
    {
        qDebug() << "PASS";
//        UDPflag = true;
    }
    else
    {
        qDebug() << "FAIL";
//        UDPflag = false;
    }

//    processPendingDatagrams();
//    connect(socket, SIGNAL(readyRead()), this, SLOT(processPendingDatagrams()),Qt::QueuedConnection);

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DESTROYER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
MainWindow::~MainWindow()
{
    qInfo() << " ";
    qInfo() << "Application is now shutting down...";
    qInfo() << " ";
    qInfo() << "Disconnecting Arduinos.";

//    arduinoTrocar.close();
//    arduinoGantry.close();

    if(serial[0].isOpen()){
        serial[0].close(); //    Close the serial port if it's open.
    }
    if(serial[1].isOpen()){
        serial[1].close(); //    Close the serial port if it's open.
    }

    // If connected to s826, send clear analog outputs prior to closing window
    if (s826.boardConnected)
    {
        qInfo() << "Clearing S826 Analog Outputs";
        clearCurrentsSlot();
    }


    // STOP recording datastream
    if(isRecordingData)
    {
        qInfo() << "Stoping data recording.";
        stopDAQRecording();
    }


    // STOP DAQ from running
    if ( DAQ.isEnabled() )
    {
        qInfo() << "Disconnecting NI USB-DAQ.";
        DAQ.finishTask();
    }

    delete ui;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SETUP FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void MainWindow::setupPlots(void)
{
    double markerWidth = 1.0; // Marker sizes larger than 1 are more visible, however they bog down the plotting process without OpenGL
    // Define colors to be reused across plots
    // Sorted Chromatically (hover mouse to see color)
    QColor darkOrchid = QColor(153, 50, 204, 255);    // Dark Orchid
    QColor UofTred = QColor(189, 19, 62, 255);        // U of T Highlight Red  // Qt::red;
    QColor pumpkinOrange = QColor(255, 118, 25, 255); // Pumpkin Orange
    QColor gold = QColor(255, 215, 0, 255);           // Gold
    QColor forestGreen = QColor(34, 139, 34, 255);    // Forest Green          // Qt::green;
    QColor darkCyan = QColor(0, 139, 139, 255);       // Dark Cyan
    QColor UofTblue = QColor(6, 41, 88, 255);         // U of T Blue           // Qt::blue;
    QColor slateGray = QColor(112, 128, 144, 255);    // Slate Gray

    // The following plot setup is mostly taken from the plot demos:
    /// Add Magnetic Field Strength Graphs to the plot
    ui->plot_magnetic->addGraph();
    ui->plot_magnetic->graph()->setPen(QPen(UofTred, markerWidth, Qt::SolidLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_magnetic->graph()->setName("Bx");
    ui->plot_magnetic->addGraph();
    ui->plot_magnetic->graph()->setPen(QPen(forestGreen, markerWidth, Qt::SolidLine)); // Forest Green          //Qt::green));
    ui->plot_magnetic->graph()->setName("By");
    ui->plot_magnetic->addGraph();
    ui->plot_magnetic->graph()->setPen(QPen(UofTblue, markerWidth, Qt::SolidLine)); // U of T Blue             //Qt::blue));
    ui->plot_magnetic->graph()->setName("Bz");
    // Add Magnetic Field Gradient Graphs to the plot
    ui->plot_magnetic->addGraph();
    ui->plot_magnetic->graph()->setPen(QPen(pumpkinOrange, markerWidth, Qt::SolidLine));
    ui->plot_magnetic->graph()->setName("dBx/dx");
    ui->plot_magnetic->addGraph();
    ui->plot_magnetic->graph()->setPen(QPen(gold, markerWidth, Qt::SolidLine));
    ui->plot_magnetic->graph()->setName("dBx/dy");
    ui->plot_magnetic->addGraph();
    ui->plot_magnetic->graph()->setPen(QPen(darkOrchid, markerWidth, Qt::SolidLine));
    ui->plot_magnetic->graph()->setName("dBx/dz");
    ui->plot_magnetic->addGraph();
    ui->plot_magnetic->graph()->setPen(QPen(darkCyan, markerWidth, Qt::SolidLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_magnetic->graph()->setName("dBy/dy");
    ui->plot_magnetic->addGraph();
    ui->plot_magnetic->graph()->setPen(QPen(slateGray, markerWidth, Qt::SolidLine));
    ui->plot_magnetic->graph()->setName("dBy/dz");
    // Add labels to the graph
    ui->plot_magnetic->xAxis->setLabel("Time [s]");
    ui->plot_magnetic->yAxis->setLabel("Magnetic Field [mT]");
    ui->plot_magnetic->yAxis->setRange(-maxB, maxB);
    // Add a legend
    ui->plot_magnetic->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop); // 0 bc only item in the plot?
    QFont legendFont = font();
    legendFont.setPointSize(8);
    ui->plot_magnetic->legend->setFont(legendFont);
    ui->plot_magnetic->legend->setSelectedFont(legendFont);
    ui->plot_magnetic->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items
    // Check if legend is enabled by GUI defaults
    if (ui->checkBox_enableMagneticLegend->checkState())
    {
        ui->plot_magnetic->legend->setVisible(true);
    }
    else
    {
        ui->plot_magnetic->legend->setVisible(false);
    }

    // Next Graph is Inputs to the system
    // CURRENTS
    // Set all Setpoints first to be on background layer
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(darkOrchid, markerWidth, Qt::DashLine));
    ui->plot_inputs->graph()->setName("Setpoint 1");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(UofTred, markerWidth, Qt::DashLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_inputs->graph()->setName("Setpoint 2");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(pumpkinOrange, markerWidth, Qt::DashLine));
    ui->plot_inputs->graph()->setName("Setpoint 3");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(gold, markerWidth, Qt::DashLine));
    ui->plot_inputs->graph()->setName("Setpoint 4");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(forestGreen, markerWidth, Qt::DashLine));
    ui->plot_inputs->graph()->setName("Setpoint 5");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(darkCyan, markerWidth, Qt::DashLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_inputs->graph()->setName("Setpoint 6");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(UofTblue, markerWidth, Qt::DashLine));
    ui->plot_inputs->graph()->setName("Setpoint 7");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(slateGray, markerWidth, Qt::DashLine));
    ui->plot_inputs->graph()->setName("Setpoint 8");
    // Now add the Measured Feedback Currents Graphs
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(darkOrchid, markerWidth, Qt::SolidLine));
    ui->plot_inputs->graph()->setName("Actuator 1");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(UofTred, markerWidth, Qt::SolidLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_inputs->graph()->setName("Actuator 2");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(pumpkinOrange, markerWidth, Qt::SolidLine));
    ui->plot_inputs->graph()->setName("Actuator 3");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(gold, markerWidth, Qt::SolidLine));
    ui->plot_inputs->graph()->setName("Actuator 4");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(forestGreen, markerWidth, Qt::SolidLine));
    ui->plot_inputs->graph()->setName("Actuator 5");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(darkCyan, markerWidth, Qt::SolidLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_inputs->graph()->setName("Actuator 6");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(UofTblue, markerWidth, Qt::SolidLine));
    ui->plot_inputs->graph()->setName("Actuator 7");
    ui->plot_inputs->addGraph();
    ui->plot_inputs->graph()->setPen(QPen(slateGray, markerWidth, Qt::SolidLine));
    ui->plot_inputs->graph()->setName("Actuator 8");
    // Setup Legend
    ui->plot_inputs->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop); // 0 bc only item in the plot?
    ui->plot_inputs->legend->setFont(legendFont);
    ui->plot_inputs->legend->setSelectedFont(legendFont);
    ui->plot_inputs->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items
    // Check if legend is enabled by GUI defaults
    if (ui->checkBox_enableInputLegend->checkState())
    {
        ui->plot_inputs->legend->setVisible(true);
    }
    else
    {
        ui->plot_inputs->legend->setVisible(false);
    }
    // Add labels to the graph
    ui->plot_inputs->xAxis->setLabel("Time [s]");
    ui->plot_inputs->yAxis->setLabel("Current [A]");


    /// Add Temperature Graph to the Thermocouple plot
    ui->plot_temperature->addGraph();
    ui->plot_temperature->graph()->setPen(QPen(UofTred, markerWidth, Qt::SolidLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_temperature->graph()->setName("T0");
    ui->plot_temperature->addGraph();
    ui->plot_temperature->graph()->setPen(QPen(forestGreen, markerWidth, Qt::SolidLine)); // Forest Green          //Qt::green));
    ui->plot_temperature->graph()->setName("T1");
    ui->plot_temperature->addGraph();
    ui->plot_temperature->graph()->setPen(QPen(UofTblue, markerWidth, Qt::SolidLine)); // U of T Blue             //Qt::blue));
    ui->plot_temperature->graph()->setName("T2");
    ui->plot_temperature->addGraph();
    ui->plot_temperature->graph()->setPen(QPen(pumpkinOrange, markerWidth, Qt::SolidLine));
    ui->plot_temperature->graph()->setName("T3");
    ui->plot_temperature->addGraph();
    ui->plot_temperature->graph()->setPen(QPen(gold, markerWidth, Qt::SolidLine));
    ui->plot_temperature->graph()->setName("T4");
    ui->plot_temperature->addGraph();
    ui->plot_temperature->graph()->setPen(QPen(darkOrchid, markerWidth, Qt::SolidLine));
    ui->plot_temperature->graph()->setName("T5");
    ui->plot_temperature->addGraph();
    ui->plot_temperature->graph()->setPen(QPen(darkCyan, markerWidth, Qt::SolidLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_temperature->graph()->setName("T6");
    ui->plot_temperature->addGraph();
    ui->plot_temperature->graph()->setPen(QPen(slateGray, markerWidth, Qt::SolidLine));
    ui->plot_temperature->graph()->setName("T7");
    // Add labels to the graph
    ui->plot_temperature->xAxis->setLabel("Time [s]");
    ui->plot_temperature->yAxis->setLabel("Temperature [C]");
    ui->plot_temperature->yAxis->setRange(-maxAllowableTemp, maxAllowableTemp);
    // Add a legend
    ui->plot_temperature->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop); // 0 bc only item in the plot?
    ui->plot_temperature->legend->setFont(legendFont);
    ui->plot_temperature->legend->setSelectedFont(legendFont);
    ui->plot_temperature->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items
    // Check if legend is enabled by GUI defaults
    if (ui->checkBox_enableTemperatureLegend->checkState())
    {
        ui->plot_temperature->legend->setVisible(true);
    }
    else
    {
        ui->plot_temperature->legend->setVisible(false);
    }

    /// Add DAQ Voltage feedback to the plot
    ui->plot_DAQ->addGraph();
    ui->plot_DAQ->graph()->setPen(QPen(UofTred, markerWidth, Qt::SolidLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_DAQ->graph()->setName("AIN0");
    ui->plot_DAQ->addGraph();
    ui->plot_DAQ->graph()->setPen(QPen(forestGreen, markerWidth, Qt::SolidLine)); // Forest Green          //Qt::green));
    ui->plot_DAQ->graph()->setName("AIN1");
    ui->plot_DAQ->addGraph();
    ui->plot_DAQ->graph()->setPen(QPen(UofTblue, markerWidth, Qt::SolidLine)); // U of T Blue             //Qt::blue));
    ui->plot_DAQ->graph()->setName("AIN2");
    ui->plot_DAQ->addGraph();
    ui->plot_DAQ->graph()->setPen(QPen(pumpkinOrange, markerWidth, Qt::SolidLine));
    ui->plot_DAQ->graph()->setName("AIN3");
    ui->plot_DAQ->addGraph();
    ui->plot_DAQ->graph()->setPen(QPen(gold, markerWidth, Qt::SolidLine));
    ui->plot_DAQ->graph()->setName("AIN4");
    ui->plot_DAQ->addGraph();
    ui->plot_DAQ->graph()->setPen(QPen(darkOrchid, markerWidth, Qt::SolidLine));
    ui->plot_DAQ->graph()->setName("AIN5");
    ui->plot_DAQ->addGraph();
    ui->plot_DAQ->graph()->setPen(QPen(darkCyan, markerWidth, Qt::SolidLine)); // U of T Highlight Red  //QPen(Qt::red));
    ui->plot_DAQ->graph()->setName("AIN6");
    ui->plot_DAQ->addGraph();
    ui->plot_DAQ->graph()->setPen(QPen(slateGray, markerWidth, Qt::SolidLine));
    ui->plot_DAQ->graph()->setName("AIN7");
    // Add labels to the graph
    ui->plot_DAQ->xAxis->setLabel("Time [s]");
    ui->plot_DAQ->yAxis->setLabel("Voltage [V]");
    ui->plot_DAQ->yAxis->setRange(-10.0, 10.0);
    // Add a legend
    ui->plot_DAQ->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop); // 0 bc only item in the plot?
    ui->plot_DAQ->legend->setFont(legendFont);
    ui->plot_DAQ->legend->setSelectedFont(legendFont);
    ui->plot_DAQ->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items
    // Check if legend is enabled by GUI defaults
    if (ui->checkBox_enableDAQLegend->checkState())
    {
        ui->plot_DAQ->legend->setVisible(true);
    }
    else
    {
        ui->plot_DAQ->legend->setVisible(false);
    }

    // Able to change the view with the mouse
    ui->plot_magnetic->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->plot_inputs->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->plot_temperature->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->plot_DAQ->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void MainWindow::findB_SurgeonSimulator(double B_desired_local[3])
{
    // closing the gripper requires a B field parallel to the (open-loop) direction of the gripper
    double B_parallel[3] = { B_mag*cos(angle2)*sin(angle1), B_mag*pow(sin(angle2),2), B_mag*cos(angle2)*cos(angle1)};
    double det = sqrt( pow(cos(angle2),2)*pow(sin(angle1),2) + pow(sin(angle2),2) );
    double constantInFront = k_factor*angle3/det;
    double B_perp[3] = {constantInFront*pow(cos(angle2),2)*sin(angle1)*cos(angle1), constantInFront*sin(angle2)*cos(angle1)*cos(angle2), -constantInFront*(pow(cos(angle2),2)*pow(sin(angle1),2) + pow(sin(angle2),2)) };

    // Check that there was not a divide by zero error from using det=0 when angle1=0 and angle2=0
    if ( isnan(B_perp[0]) || isnan(B_perp[1]) || isnan(B_perp[2]) )
    {
        for (int i=0;i<3;i++)
        {
            B_desired_local[i] = B_parallel[i]; // Ignore B_perp as it should be 0 from the calculation of 0/0;
        }
    }
    else
    {
        // Calculate the desired Local B-field from parallel and perpendicular components
        for (int i=0;i<3;i++)
        {
            B_desired_local[i] = B_parallel[i] + B_perp[i];
        }
    }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~ PLOTTING FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void MainWindow::addPoints(double x_Axis, double y_Axis[8], int graphIndex)
{
    // NOTE: this method of appending points gets compuationally expensive as the arrays get larger and larger.
    // The clear time function was implemented to combat this but really an empty time array should be declared
    // initially with values filled in as time progresses. This would mean that recorded plots would be limited in length.
    switch (graphIndex)
    {
    case 0:
        // Magnetic Fields are selected
        qv_t_mag.append(x_Axis);
        qv_Bx.append(y_Axis[0]*1000);
        qv_By.append(y_Axis[1]*1000);
        qv_Bz.append(y_Axis[2]*1000);
        qv_dBxdx.append(y_Axis[3]*1000);
        qv_dBxdy.append(y_Axis[4]*1000);
        qv_dBxdz.append(y_Axis[5]*1000);
        qv_dBydy.append(y_Axis[6]*1000);
        qv_dBydz.append(y_Axis[7]*1000);
        break;
    case 1:
        // Current Measurements from Feedback are selected
        qv_t_input.append(x_Axis);
        qv_current0.append(y_Axis[0]);
        qv_current1.append(y_Axis[1]);
        qv_current2.append(y_Axis[2]);
        qv_current3.append(y_Axis[3]);
        qv_current4.append(y_Axis[4]);
        qv_current5.append(y_Axis[5]);
        qv_current6.append(y_Axis[6]);
        qv_current7.append(y_Axis[7]);
        break;
    case 2:
        // Current Setpoints are selected
        qv_setpoint0.append(y_Axis[0]);
        qv_setpoint1.append(y_Axis[1]);
        qv_setpoint2.append(y_Axis[2]);
        qv_setpoint3.append(y_Axis[3]);
        qv_setpoint4.append(y_Axis[4]);
        qv_setpoint5.append(y_Axis[5]);
        qv_setpoint6.append(y_Axis[6]);
        qv_setpoint7.append(y_Axis[7]);
        break;
    case 3:
        // For Thermocouple temperature Plots
        qv_t_thermocouple.append(x_Axis);
        qv_temp0.append(y_Axis[0]);
        qv_temp1.append(y_Axis[1]);
        qv_temp2.append(y_Axis[2]);
        qv_temp3.append(y_Axis[3]);
        qv_temp4.append(y_Axis[4]);
        qv_temp5.append(y_Axis[5]);
        qv_temp6.append(y_Axis[6]);
        qv_temp7.append(y_Axis[7]);
        break;
    case 4:
        // For DAQ controller Plots
        qv_t_DAQ.append(x_Axis);
        qv_DAQai0.append(y_Axis[0]);
        qv_DAQai1.append(y_Axis[1]);
//        qDebug() << "Datapoint value: " << y_Axis[1];
//        qDebug() << "Datapoint value: " << y_Axis[2];
//        qDebug() << "Datapoint value: " << y_Axis[3];
        qv_DAQai2.append(y_Axis[2]);
        qv_DAQai3.append(y_Axis[3]);
        qv_DAQai4.append(y_Axis[4]);
        qv_DAQai5.append(y_Axis[5]);
        qv_DAQai6.append(y_Axis[6]);
        qv_DAQai7.append(y_Axis[7]);
//        qDebug() << "Added Points to DAQ plot";
        break;

    default:
        qDebug() << "Non-valid graph index used. No points were added to the plot";
        break;
    }

}

void MainWindow::plot()
{

    // Trying to limit the amount of things being plotted
    // especially since the majority of the plot is not seen unless paused.
    int data_Length = plotPeriod * 1000 / captionRefreshPeriod;

    // Check for each enabled plots and update the plot if enabled
    if (magneticPlotState)
    {
        // Magnetic Field components
        ui->plot_magnetic->graph(0)->setData(qv_t_mag.mid(qv_t_mag.size()-data_Length,data_Length), qv_Bx.mid(qv_Bx.size()-data_Length,data_Length));
        ui->plot_magnetic->graph(1)->setData(qv_t_mag.mid(qv_t_mag.size()-data_Length,data_Length), qv_By.mid(qv_By.size()-data_Length,data_Length));
        ui->plot_magnetic->graph(2)->setData(qv_t_mag.mid(qv_t_mag.size()-data_Length,data_Length), qv_Bz.mid(qv_Bz.size()-data_Length,data_Length));
        if (gradientPlotState)
        {
            // Magnetic Gradient Field components
            ui->plot_magnetic->graph(3)->setData(qv_t_mag, qv_dBxdx);
            ui->plot_magnetic->graph(4)->setData(qv_t_mag, qv_dBxdy);
            ui->plot_magnetic->graph(5)->setData(qv_t_mag, qv_dBxdz);
            ui->plot_magnetic->graph(6)->setData(qv_t_mag, qv_dBydy);
            ui->plot_magnetic->graph(7)->setData(qv_t_mag, qv_dBydz);
        }
        ui->plot_magnetic->xAxis->setRange(currentTime.elapsed()/1000.0-plotPeriod, currentTime.elapsed()/1000.0); // units of time are in seconds.
        ui->plot_magnetic->yAxis->setRange(-maxB*1000, maxB*1000); // All calculations are done in Teslas. Plots are in mT.
        ui->plot_magnetic->replot();
        ui->plot_magnetic->update();
    }
    if (inputPlotState)
    {
        // Current Setpoints
        if (setpointPlotState)
        {
            ui->plot_inputs->graph(0)->setData(qv_t_input, qv_setpoint0);
            ui->plot_inputs->graph(1)->setData(qv_t_input, qv_setpoint1);
            ui->plot_inputs->graph(2)->setData(qv_t_input, qv_setpoint2);
            ui->plot_inputs->graph(3)->setData(qv_t_input, qv_setpoint3);
            ui->plot_inputs->graph(4)->setData(qv_t_input, qv_setpoint4);
            ui->plot_inputs->graph(5)->setData(qv_t_input, qv_setpoint5);
            ui->plot_inputs->graph(6)->setData(qv_t_input, qv_setpoint6);
            ui->plot_inputs->graph(7)->setData(qv_t_input, qv_setpoint7);
        }
        // Measured Currents (Motor Amplifiers Feedback)
        ui->plot_inputs->graph(8)->setData(qv_t_input, qv_current0);
        ui->plot_inputs->graph(9)->setData(qv_t_input, qv_current1);
        ui->plot_inputs->graph(10)->setData(qv_t_input, qv_current2);
        ui->plot_inputs->graph(11)->setData(qv_t_input, qv_current3);
        ui->plot_inputs->graph(12)->setData(qv_t_input, qv_current4);
        ui->plot_inputs->graph(13)->setData(qv_t_input, qv_current5);
        ui->plot_inputs->graph(14)->setData(qv_t_input, qv_current6);
        ui->plot_inputs->graph(15)->setData(qv_t_input, qv_current7);
        ui->plot_inputs->xAxis->setRange(currentTime.elapsed()/1000.0-plotPeriod, currentTime.elapsed()/1000.0); // units of time are in seconds.
        ui->plot_inputs->yAxis->setRange(-maxCurrent, maxCurrent); // REMOVE AFTER TESTING.
        ui->plot_inputs->replot();
        ui->plot_inputs->update();
    }
    if (temperaturePlotState)
    {
        // Measured Temperatures (Thermocouples Feedback)
        ui->plot_temperature->graph(0)->setData(qv_t_thermocouple, qv_temp0);
        ui->plot_temperature->graph(1)->setData(qv_t_thermocouple, qv_temp1);
        ui->plot_temperature->graph(2)->setData(qv_t_thermocouple, qv_temp2);
        ui->plot_temperature->graph(3)->setData(qv_t_thermocouple, qv_temp3);
        ui->plot_temperature->graph(4)->setData(qv_t_thermocouple, qv_temp4);
        ui->plot_temperature->graph(5)->setData(qv_t_thermocouple, qv_temp5);
        ui->plot_temperature->graph(6)->setData(qv_t_thermocouple, qv_temp6);
        ui->plot_temperature->graph(7)->setData(qv_t_thermocouple, qv_temp7);
        ui->plot_temperature->xAxis->setRange(currentTime.elapsed()/1000.0-plotPeriod, currentTime.elapsed()/1000.0); // units of time are in seconds.
        ui->plot_temperature->yAxis->setRange(0 , 1.5*maxAllowableTemp);
        ui->plot_temperature->replot();
        ui->plot_temperature->update();
    }
    if (DAQPlotState)
    {
        // Feedback voltages from the DAQ
        if (dataRecordingState == 2)
        {
            double Bx = DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1];
            double By = -DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0];
            double Bz = DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2];
            ui->plot_DAQ-> graph(0)->setData(qv_t_DAQ, qv_DAQai0); //try to plot measured field Bx,By,Bz, but failed to implement that
            ui->plot_DAQ-> graph(1)->setData(qv_t_DAQ, qv_DAQai1); // leave code here for later debug
            ui->plot_DAQ-> graph(2)->setData(qv_t_DAQ, qv_DAQai2);
    //        ui->plot_DAQ-> graph(3)->setData(qv_t_DAQ, qv_DAQai3);
    //        ui->plot_DAQ-> graph(4)->setData(qv_t_DAQ, qv_DAQai4);
    //        ui->plot_DAQ-> graph(5)->setData(qv_t_DAQ, qv_DAQai5);
    //        ui->plot_DAQ-> graph(6)->setData(qv_t_DAQ, qv_DAQai6);
    //        ui->plot_DAQ-> graph(7)->setData(qv_t_DAQ, qv_DAQai7);
            ui->plot_DAQ->xAxis->setRange(currentTime.elapsed()/1000.0-plotPeriod, currentTime.elapsed()/1000.0); // units of time are in seconds.
            ui->plot_DAQ->yAxis->setRange(-0.10 , 0.10); // TODO: make this variable and able to change from GUI
        }
        else
        {
            ui->plot_DAQ-> graph(0)->setData(qv_t_DAQ, qv_DAQai0);
            ui->plot_DAQ-> graph(1)->setData(qv_t_DAQ, qv_DAQai1);
            ui->plot_DAQ-> graph(2)->setData(qv_t_DAQ, qv_DAQai2);
    //        ui->plot_DAQ-> graph(3)->setData(qv_t_DAQ, qv_DAQai3);
    //        ui->plot_DAQ-> graph(4)->setData(qv_t_DAQ, qv_DAQai4);
    //        ui->plot_DAQ-> graph(5)->setData(qv_t_DAQ, qv_DAQai5);
    //        ui->plot_DAQ-> graph(6)->setData(qv_t_DAQ, qv_DAQai6);
    //        ui->plot_DAQ-> graph(7)->setData(qv_t_DAQ, qv_DAQai7);
            ui->plot_DAQ->xAxis->setRange(currentTime.elapsed()/1000.0-plotPeriod, currentTime.elapsed()/1000.0); // units of time are in seconds.
            ui->plot_DAQ->yAxis->setRange(-0.10 , 0.10); // TODO: make this variable and able to change from GUI
        }

//        ui->plot_DAQ->yAxis->setRange(-300 , 300); // TODO: make this variable and able to change from GUI
        ui->plot_DAQ->replot();
        ui->plot_DAQ->update();
    }


}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~ ARDUINO FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void MainWindow::gantryZero(void)
{
    gantryWrite(GANTRY_ZERO);
    for (int i = 0; i <3 ;i++)
    {
        gantryPos[i] = gantryZeros[i];
    }
    // Reset all scrollboxes
    ui->spinBox_gantryAbsSteps_X->setValue(gantryZeros[0]);
    ui->spinBox_gantryAbsSteps_Y->setValue(gantryZeros[1]);
    ui->spinBox_gantryAbsSteps_Z->setValue(gantryZeros[2]);
    ui->doubleSpinBox_gantryAbsMM_X->setValue((double)gantryZeros[0]/gantryStepsPerMM);
    ui->doubleSpinBox_gantryAbsMM_Y->setValue((double)gantryZeros[1]/gantryStepsPerMM);
    ui->doubleSpinBox_gantryAbsMM_Z->setValue((double)gantryZeros[2]/gantryStepsPerMM);
}

void MainWindow::gantryCenter(void)
{
    gantryWrite(GANTRY_CENTER);
    for (int i = 0; i <3 ;i++)
    {
        gantryPos[i] = 0;
    }
    // Reset all scrollboxes
    ui->spinBox_gantryAbsSteps_X->setValue(0);
    ui->spinBox_gantryAbsSteps_Y->setValue(0);
    ui->spinBox_gantryAbsSteps_Z->setValue(0);
    ui->doubleSpinBox_gantryAbsMM_X->setValue(0.0);
    ui->doubleSpinBox_gantryAbsMM_Y->setValue(0.0);
    ui->doubleSpinBox_gantryAbsMM_Z->setValue(0.0);
}

void MainWindow::gantryJog(void)
{
    if (ui->pushButton_gantryNegX->hasFocus())
    {
        qDebug() << "move neg x";
        // Swapped polarity
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_REL, GANTRY_X, GANTRY_POS, gantryJogSteps);

    }
    else if (ui->pushButton_gantryPosX->hasFocus())
    {
        qDebug() << "move pos x";
        // Swapped polarity
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_REL, GANTRY_X, GANTRY_NEG, gantryJogSteps);
    }
    else if (ui->pushButton_gantryNegY->hasFocus())
    {
        qDebug() << "move neg y";
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_REL, GANTRY_Y, GANTRY_NEG, gantryJogSteps);
    }
    else if (ui->pushButton_gantryPosY->hasFocus())
    {
        qDebug() << "move pos y";
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_REL, GANTRY_Y, GANTRY_POS, gantryJogSteps);
    }
    else if (ui->pushButton_gantryNegZ->hasFocus())
    {
        qDebug() << "move neg Z";
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_REL, GANTRY_Z, GANTRY_NEG, gantryJogSteps);
    }
    else if (ui->pushButton_gantryPosZ->hasFocus())
    {
        qDebug() << "move pos Z";
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_REL, GANTRY_Z, GANTRY_POS, gantryJogSteps);
    }

}

void MainWindow::gantryMoveAbsolute(void)
{
    // Check all the step scroll boxes first
    if (ui->spinBox_gantryAbsSteps_X->value() != gantryPos[0])
    {
        qInfo() << "Setting the x abs target command";
        // Record value
        gantryPos[0] = ui->spinBox_gantryAbsSteps_X->value();
        // update mm label box
        ui->doubleSpinBox_gantryAbsMM_X->setValue( (double)gantryPos[0]/gantryStepsPerMM );
        // Send command to the Gantry
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_X, gantryPos[0]-gantryZeros[0]);
    }
    else if (ui->spinBox_gantryAbsSteps_Y->value() != gantryPos[1])
    {
        qInfo() << "Setting the Y abs target command";
        // Record value
        gantryPos[1] = ui->spinBox_gantryAbsSteps_Y->value();
        // update mm label box
        ui->doubleSpinBox_gantryAbsMM_Y->setValue( (double)gantryPos[1]/gantryStepsPerMM );
        // Send command to the Gantry
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Y, gantryPos[1]-gantryZeros[1]);
    }
    else if (ui->spinBox_gantryAbsSteps_Z->value() != gantryPos[2])
    {
        qInfo() << "Setting the Z abs target command";
        // Record value
        gantryPos[2] = ui->spinBox_gantryAbsSteps_Z->value();
        // update mm label box
        ui->doubleSpinBox_gantryAbsMM_Z->setValue( (double)gantryPos[2]/gantryStepsPerMM );
        // Send command to the Gantry
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Z, gantryPos[2]-gantryZeros[2]);
    }

    // Now check all the mm double spin boxes by comparing steps to smallest setable mm size
    else if ( abs( (double)gantryPos[0]/gantryStepsPerMM - ui->doubleSpinBox_gantryAbsMM_X->value() ) > ui->doubleSpinBox_gantryAbsMM_X->singleStep() )
    {
        qInfo() << "Setting the X abs target command in mm";
        // Record value
        gantryPos[0] = (int) (ui->doubleSpinBox_gantryAbsMM_X->value() * gantryStepsPerMM);
        // update steps label box
        ui->spinBox_gantryAbsSteps_X->setValue(gantryPos[0]);
        // Send command to the Gantry
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_X, gantryPos[0]-gantryZeros[0]);
    }
    else if ( abs( (double)gantryPos[1]/gantryStepsPerMM - ui->doubleSpinBox_gantryAbsMM_Y->value() ) > ui->doubleSpinBox_gantryAbsMM_Y->singleStep() )
    {
        qInfo() << "Setting the Y abs target command in mm";
        // Record value
        gantryPos[1] = (int) (ui->doubleSpinBox_gantryAbsMM_Y->value() * gantryStepsPerMM);
        // update steps label box
        ui->spinBox_gantryAbsSteps_Y->setValue(gantryPos[1]);
        // Send command to the Gantry
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Y, gantryPos[1]-gantryZeros[1]);
    }
    else if ( abs( (double)gantryPos[2]/gantryStepsPerMM - ui->doubleSpinBox_gantryAbsMM_Z->value() ) > ui->doubleSpinBox_gantryAbsMM_Z->singleStep() )
    {
        qInfo() << "Setting the Z abs target command in mm";
        // Record value
        gantryPos[2] = (int) (ui->doubleSpinBox_gantryAbsMM_Z->value() * gantryStepsPerMM);
        // update steps label box
        ui->spinBox_gantryAbsSteps_Z->setValue(gantryPos[2]);
        // Send command to the Gantry
        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Z, gantryPos[2]-gantryZeros[2]);
    }
    // TODO Increase steps scroll box maximums?

    // Could speed this up by just writing multistep commands for all 3 simultaneously
}

void MainWindow::gantryWrite(int mode, int reference, int axis, int dir, int steps)
{
//    qDebug() << "made it here";
    // This number of inputs is for SINGLE AXIS RELATIVE motion for the gantry
    std::string outputString;
    if (dir)
    {
        outputString = "<" + std::to_string(mode) + "," + std::to_string(reference) + "," + std::to_string(axis) + "," + std::to_string(steps) + ">";
    }
    else
    {
        outputString = "<" + std::to_string(mode) + "," + std::to_string(reference) + "," + std::to_string(axis) + ",-" + std::to_string(steps) + ">";
    }

    qDebug() << "Relative Single Axis Command: " << outputString.c_str() << " sent to the Gantry.";
    QByteArray outputdata(outputString.c_str(),outputString.length());

    arduinoWrite(SERIAL_GANTRY, outputdata);
}

void MainWindow::gantryWrite(int mode, int reference, int axis, int steps)
{
//    qDebug() << "made it here";
    // This number of inputs is for ABSOLUTE motion for the gantry
    std::string outputString = "<" + std::to_string(mode) + "," + std::to_string(reference) + "," + std::to_string(axis) + "," + std::to_string(steps) + ">";
//    qDebug() << "Absolute Single Axis Command: " << outputString.c_str() << " sent to the Gantry.";

    QByteArray outputdata(outputString.c_str(),outputString.length());

    arduinoWrite(SERIAL_GANTRY, outputdata);
}

void MainWindow::gantryWrite(int mode)
{
    // This only works for Zeroing and Centering commands
    std::string outputString = "<" + std::to_string(mode) + ">";
    qDebug() << outputString.c_str() << " sent to the Gantry.";
    QByteArray outputdata(outputString.c_str(),outputString.length());

    arduinoWrite(SERIAL_GANTRY, outputdata);
}

void MainWindow::gantrySpeed(int speed)
{
    // This only works for changing gantry moving speed
    int mode = 7;
    std::string outputString = "<" + std::to_string(mode) + "," + std::to_string(speed) + ">";
    qDebug() << outputString.c_str() << " sent to the Gantry.";
    QByteArray outputdata(outputString.c_str(),outputString.length());

    arduinoWrite(SERIAL_GANTRY, outputdata);
}

void MainWindow::gantryMoveMulti(int Posxyz[3])
{
    // This only works for Zeroing and Centering commands
    int mode = 1; // 1 for multi move
    int submode = 0; // 0 for absolute move
    std::string outputString = "<" + std::to_string(mode) + "," + std::to_string(submode) + "," + std::to_string(Posxyz[0]) + "," + std::to_string(Posxyz[1]) + "," + std::to_string(Posxyz[2]) + ">";
    qDebug() << outputString.c_str() << " sent to the Gantry.";
    QByteArray outputdata(outputString.c_str(),outputString.length());

    arduinoWrite(SERIAL_GANTRY, outputdata);
}

void MainWindow::setGantryJogSteps()
{
    gantryJogSteps = ui->spinBox_gantryJogSteps->value();
}

void MainWindow::arduinoWrite(int arduinoNum, QByteArray outputdata)
{

    serial[arduinoNum].write(outputdata);
//    serial[arduinoNum].flush();
    serial[arduinoNum].waitForBytesWritten(100);

    qDebug()<<outputdata;

//    std::string writeport = "/dev/ttyACM";
//    writeport = writeport + std::to_string(arduinoNum);

//    std::string nVal = std::to_string( position );
//    nVal = nVal + ',';
    // std::cout << writeport << "  " << nVal;
    //Print to Arduino via serial
//    std::ofstream arduino;
//    arduino.open( writeport );
//    arduino << nVal;
//    arduino.close();
}

void MainWindow::ArduinoWrite4(double motorAngles[4])
{
    int position[4];
    // Takes 4 motor angles in radians and converts to encoder steps and sends
    // the correct command to the respective Arduinos
    for (int n=0; n<4; n++)
    {
        position[n] = (int) (motorAngles[n]*EncoderStepConversion/(2*M_PI));
        // cout << position << endl;
        position[n] = position[n] + EncoderStepConversion; // shift 1 revolution to avoid sending negative numbers
        // cout << position << endl;
//        ArduinoWrite(n, position);
        // Utilize other function to rid unecessary rewritten code
    }

//     qDebug() << "sent to motors" << endl;
    qDebug() << "Sent to motors.";
    arduinoMotor0.open( "/dev/ttyACM0" );
    arduinoMotor0 << std::to_string(position[0]) + ',';
    arduinoMotor0.close();
    arduinoMotor1.open( motor1_port );
    arduinoMotor1 << std::to_string(position[1]) + ',';
    arduinoMotor1.close();
    arduinoMotor2.open( motor2_port );
    arduinoMotor2 << std::to_string(position[2]) + ',';
    arduinoMotor2.close();
    arduinoMotor3.open( motor3_port);
    arduinoMotor3 << std::to_string(position[3]) + ',';
    arduinoMotor3.close();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SLOTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //


void MainWindow::updateCaption(void)
{
    // Update labels on the GUI to reflect parameters saved in code

    // for testing connection
    ui->label_status->setText(tr("%1").arg(connectedGamepad.joystickValues[1]));

//    ui->label_test->setText((QString)controllerState);
    //Temperature sensors
    ui->label_temperature0->setText(tr("%1").arg(round(measuredTemperatures[0]*100)/100.0));
    ui->label_temperature1->setText(tr("%1").arg(round(measuredTemperatures[1]*100)/100.0));
    ui->label_temperature2->setText(tr("%1").arg(round(measuredTemperatures[2]*100)/100.0));
    ui->label_temperature3->setText(tr("%1").arg(round(measuredTemperatures[3]*100)/100.0));
    ui->label_temperature4->setText(tr("%1").arg(round(measuredTemperatures[4]*100)/100.0));
    ui->label_temperature5->setText(tr("%1").arg(round(measuredTemperatures[5]*100)/100.0));
    ui->label_temperature6->setText(tr("%1").arg(round(measuredTemperatures[6]*100)/100.0));
    ui->label_temperature7->setText(tr("%1").arg(round(measuredTemperatures[7]*100)/100.0));

    // Only round the output text, not the actual saved values
    // Update numerical values shown on labels in the GUI
    ui->label_Bx_desired->setText(tr("%1").arg(round(B_Local_Desired[0]*1000000)/1000));
    ui->label_By_desired->setText(tr("%1").arg(round(B_Local_Desired[1]*1000000)/1000));
    ui->label_Bz_desired->setText(tr("%1").arg(round(B_Local_Desired[2]*1000000)/1000));
    ui->label_Bx_desired_global->setText(tr("%1").arg(round(B_Global_Desired[0]*1000000)/1000));
    ui->label_By_desired_global->setText(tr("%1").arg(round(B_Global_Desired[1]*1000000)/1000));
    ui->label_Bz_desired_global->setText(tr("%1").arg(round(B_Global_Desired[2]*1000000)/1000));

    ui->label_V0_setpoint->setText(tr("%1").arg(round(outputAnalogVoltages[0]*1000)/1000));
    ui->label_V1_setpoint->setText(tr("%1").arg(round(outputAnalogVoltages[1]*1000)/1000));
    ui->label_V2_setpoint->setText(tr("%1").arg(round(outputAnalogVoltages[2]*1000)/1000));
    ui->label_V3_setpoint->setText(tr("%1").arg(round(outputAnalogVoltages[3]*1000)/1000));
    ui->label_V4_setpoint->setText(tr("%1").arg(round(outputAnalogVoltages[4]*1000)/1000));
    ui->label_V5_setpoint->setText(tr("%1").arg(round(outputAnalogVoltages[5]*1000)/1000));
    ui->label_V6_setpoint->setText(tr("%1").arg(round(outputAnalogVoltages[6]*1000)/1000));
    ui->label_V7_setpoint->setText(tr("%1").arg(round(outputAnalogVoltages[7]*1000)/1000));

    ui->label_GaussmeterBx->setText(tr("%1").arg(round(gaussmeterCalibratedConstants[0]*DAQ.analogRawInputVoltages[0]*1000)/1000));
    ui->label_GaussmeterBy->setText(tr("%1").arg(round(gaussmeterCalibratedConstants[1]*DAQ.analogRawInputVoltages[1]*1000)/1000));
    ui->label_GaussmeterBz->setText(tr("%1").arg(round(gaussmeterCalibratedConstants[2]*DAQ.analogRawInputVoltages[2]*1000)/1000));
    // Update Sliders based on joystick commands

    // Set Global sliders based on change in Desired Field
    // (NOTE: These changes would be due to the joystick setting the desired values)
    ui->horizontalSlider_Bx_Global->setSliderPosition( round(B_Global_Desired[0]*1000000000)/1000 ) ; // [changes from T -> µT] So that integer values can be used (resolution 1µT)
    ui->horizontalSlider_By_Global->setSliderPosition( round(B_Global_Desired[1]*1000000000)/1000 ) ; // [changes from T -> µT] So that integer values can be used (resolution 1µT)
    ui->horizontalSlider_Bz_Global->setSliderPosition( round(B_Global_Desired[2]*1000000000)/1000 ) ; // [changes from T -> µT] So that integer values can be used (resolution 1µT)
    // Same for corresponding scrollboxes. Check to see if they are being editted by the
    // user before writing to as writing will overwrite the user input faster than they can type.
    if ( !ui->doubleSpinBox_Bx_Global->hasFocus() )
    {
        ui->doubleSpinBox_Bx_Global->setValue( B_Global_Desired[0]*1000000/1000 ); // [changes from T -> mT] For display purposes
    }
    if ( !ui->doubleSpinBox_By_Global->hasFocus() )
    {
        ui->doubleSpinBox_By_Global->setValue( B_Global_Desired[1]*1000000/1000 ); // [changes from T -> mT] For display purposes
    }
    if ( !ui->doubleSpinBox_Bz_Global->hasFocus() )
    {
        ui->doubleSpinBox_Bz_Global->setValue( B_Global_Desired[2]*1000000/1000 ); // [changes from T -> mT] For display purposes
    }
    // Set Local sliders based on change in Desired Local Field
    // (NOTE: These changes would be due to the joystick setting the desired values)
    ui->horizontalSlider_Bx_Local->setSliderPosition( round(B_Local_Desired[0]*1000000000)/1000 ) ; // [changes from T -> µT] So that integer values can be used (resolution 1µT)
    ui->horizontalSlider_By_Local->setSliderPosition( round(B_Local_Desired[1]*1000000000)/1000 ) ; // [changes from T -> µT] So that integer values can be used (resolution 1µT)
    ui->horizontalSlider_Bz_Local->setSliderPosition( round(B_Local_Desired[2]*1000000000)/1000 ) ; // [changes from T -> µT] So that integer values can be used (resolution 1µT)
    // Same for corresponding scrollboxes. Check to see if they are being editted by the
    // user before writing to as writing will overwrite the user input faster than they can type.
    if ( !ui->doubleSpinBox_Bx_Local->hasFocus() )
    {
        ui->doubleSpinBox_Bx_Local->setValue( B_Local_Desired[0]*1000000/1000 ); // [changes from T -> mT] For display purposes
    }
    if ( !ui->doubleSpinBox_By_Local->hasFocus() )
    {
        ui->doubleSpinBox_By_Local->setValue( B_Local_Desired[1]*1000000/1000 ); // [changes from T -> mT] For display purposes
    }
    if ( !ui->doubleSpinBox_Bz_Local->hasFocus() )
    {
        ui->doubleSpinBox_Bz_Local->setValue( B_Local_Desired[2]*1000000/1000 ); // [changes from T -> mT] For display purposes
    }


    ui->label_Bx_output->setText(tr("%1").arg(round(B_Global_Output[0]*1000000)/1000));
    ui->label_By_output->setText(tr("%1").arg(round(B_Global_Output[1]*1000000)/1000));
    ui->label_Bz_output->setText(tr("%1").arg(round(B_Global_Output[2]*1000000)/1000));
//    ui->label_motor0_deg->setText(tr("%1").arg(round(init_Angles[0]*180/M_PI*1000)/1000));
//    ui->label_motor1_deg->setText(tr("%1").arg(round(init_Angles[1]*180/M_PI*1000)/1000));
//    ui->label_motor2_deg->setText(tr("%1").arg(round(init_Angles[2]*180/M_PI*1000)/1000));
//    ui->label_motor3_deg->setText(tr("%1").arg(round(init_Angles[3]*180/M_PI*1000)/1000));
    ui->label_angle1->setText(tr("%1").arg(round(angle1*180/M_PI*1000)/1000));
    ui->label_angle2->setText(tr("%1").arg(round(angle2*180/M_PI*1000)/1000));
    ui->label_B_mag->setText(tr("%1").arg(round(B_mag*1000000)/1000));

    double timeStamp = currentTime.elapsed();
    double fps = 1.0 / (timeStamp/1000.0-lastTime/1000.0);
    ui->label_fps_ext->setText(tr("%1").arg(fps));
//    qDebug() << fps;
    lastTime = timeStamp;

    // Update the Roll of the device
    changeRoll(roll_dir);
    // Advance time by period was inaccurate.
//    currentTime = currentTime + captionRefreshPeriod/1000.0; // Units in seconds. Update the time by adding the time between calls.

    //    double temppp[4] = {0.0};


//    // Update variables based on values read from analog read:
//    for (int i = 0; i < 8; i++)
//    {
//        measuredCurrents[i] = inputAnalogVoltages[i]*currentSenseFactor; // Measured EM Coil currents in A
//        measuredTemperatures[i+8] = inputAnalogVoltages[i+8]*temperatureSenseFactor; // Measured EM core Temperature in Deg C
//    }


    // Add values to the plots.
    // Specify which plot with an index value:
    // 0. Magnetic Plot
    // 1. Currents in the Actuators Plot
    // 2. Setpoints in the Actuators Plot
    // 3. Temperatures in the Thermocouples Plot
    // 4. Preset Values in the DAQ Plot (editable)
    // NOTE: Must add both Currents and Setpoints to the Actuators plot.
    addPoints(currentTime.elapsed()/1000.0, B_Global_Output, 0);  // Magnetic Field plot
    addPoints(currentTime.elapsed()/1000.0, measuredCurrents, 1); // Current feedback plot
    addPoints(currentTime.elapsed()/1000.0, currentSetpoints, 2); // Current setpoint plot
    addPoints(currentTime.elapsed()/1000.0, measuredTemperatures, 3); // Thermocouple Temperatures plot
//    addPoints(currentTime.elapsed()/1000.0, DAQ.analogInputVoltages, 4); // DAQ Voltages plot
    addPoints(currentTime.elapsed()/1000.0, DAQ.analogInputVoltages, 4); // DAQ Voltages plot
    //
    // Update ALL plots simultaneously
    plot();
}


void MainWindow::updateCurrents(void)
{
    // This is the only function that should tell the S826 to send currents to the coils
    // Reads from Variables:
    //      B_Global_Desired : The desire field at the time
    //      isGradientControlled : Boolean to determine if inverse or pseudoInverse is necessary
    // Sets the Variables:
    //      currentSetpoints : The target current in the coils

    // NOTE: This code finds the inverse of the control matrices every single time it is called.
    //      This isn't necessary if the tool is stationary in the workspace, since the control
    //      matrix, as well as its inverse, will be unchanged.

    // Calling updateCurrents will:
    // 1. Check for overheating
    // 2. Determine the currents needed to generate the desired field
    // 3. Calculate the theoretical magnetic field produced by said currents
    // 4. Send analog output command to change the coil currents if the system is not overheating

    if (!overheatingFlag)
    {
        // Calculate the necessary current setpoints based on the desired global field
        if (isGradientControlled)
        {
            // Consider Gradient components here:
            // N is numField+numGrad x numAct
            cv::invert(cv::Mat(numField+numGrad,numAct,CV_64F,N),cv::Mat(numAct,numField+numGrad,CV_64F,invN),cv::DECOMP_SVD); // Singular Value Decomposition is slowest but it can handle nonsquare
            // This would have been made into a simpler function but I suck at pointers and functions of matrices
            // It currently makes 2 matrices from the values of N and invN, takes the inverse of N and stores it in invN

            // Multiply the inverse matrix with the desired field and the resulting vector is the necessary currents
            // If the desired fields result in currents that are larger than what is possible to generate, then
            // all currents will need to be scaled down by the same factor.
            double maxCurrentBuf = 0.0;
            // Explicit matrix multiplication here
            for (int i = 0; i < numAct; i++)
            {
                double sum = 0.0;
                for ( int j = 0; j < numField+numGrad; j++)
                {
                    sum += invN[i][j]*B_Global_Desired[j];
                }
                currentSetpoints[i] = sum;
                // Check that none of the currents are above the max by finding the max and scaling all down accordingly afterwards.
                if ( abs(currentSetpoints[i]) > maxCurrentBuf)
                {
                    maxCurrentBuf = abs(currentSetpoints[i]);
                }
            }
            // Scale down all if necessary
            // Currents are normalized and fall on a scale between 0-1 representing currents
            // from zero to max current (0 A - 25 A)
            if (maxCurrentBuf > 1.0)
            {
                double scalingFactor = 1.0/maxCurrentBuf;
                for (int i = 0; i < numAct; i++)
                {
                    // Multiply all setpoints by (positive and less than unity) scaling factor
                    currentSetpoints[i] *= scalingFactor;
                    // Scale setpoints to their actual value
                    currentSetpoints[i] *= maxCurrent;
                }
            }
            else
            {
                for (int i = 0; i < numAct; i++)
                {
                    // Scale setpoints to their actual value
                    currentSetpoints[i] *= maxCurrent;
                }
            }

        }
        else
        {
            // Don't worry about gradients:
            // M is numField x numAct
            cv::invert(cv::Mat(numField,numAct,CV_64F,M),cv::Mat(numAct,numField,CV_64F,pseudoinvM),cv::DECOMP_SVD); // Singular Value Decomposition is slowest but it can handle nonsquare
            // This would have been made into a simpler function but I suck at pointers and functions of matrices
            // It currently makes 2 matrices from the values of M and pseudoinvM, takes the inverse of M and stores it in pseudoinvM

            // Multiply the inverse matrix with the desired field and the resulting vector is the necessary currents
            double maxCurrentBuf = 0.0;
            for (int i = 0; i < numAct; i++)
            {
                double sum = 0.0;
                for ( int j = 0; j < numField; j++)
                {
                    sum += pseudoinvM[i][j]*B_Global_Desired[j];
                }
                currentSetpoints[i] = sum;
                // Check that none of the currents are above the max by finding the max and scaling all down accordingly afterwards.
                if ( abs(currentSetpoints[i]) > maxCurrentBuf)
                {
                    maxCurrentBuf = abs(currentSetpoints[i]);
                }
            }
            // Scale down all currents if necessary (if max required current is > max allowable current)
            // And scale up normalized currents to theiir actual values.
            if ( maxCurrentBuf > 1.0 )
            {              
                double scalingFactor = 1.0/maxCurrentBuf;
                for (int i = 0; i < numAct; i++)
                {
                    // Multiply all setpoints by (positive and less than unity) scaling factor
                    currentSetpoints[i] *= scalingFactor;
                    // Scale setpoints to their actual value
                    currentSetpoints[i] *= maxCurrent;
                }
            }
            else
            {
                for (int i = 0; i < numAct; i++)
                {
                    // Scale setpoints to their actual value
                    currentSetpoints[i] *= maxCurrent;
                }
            }
        }


        // No matter the state, find the theoretical magnetic field produced by the setpoint currents
        for (int k = 0; k < numField+numGrad; k++)
        {
            double sum = 0.0;
            for (int l = 0; l < numAct; l++)
            {
                sum += N[k][l]*currentSetpoints[l]/maxCurrent;
            }
            B_Global_Output[k] = sum;
        }
        // Send current setpoints as normal
        // Need to convert to voltages first

        // TODO convert amps to volts
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = currentSetpoints[i] * currentControlAdj[i]; // Voltage = (Amps) * (Volts/Amp)
            // TODO limit voltages sent to S826

        }


    }
    else
    {
        // Only send 0s to the currents to turn them off because the system is overheating!
//        clearCurrentsSlot();
        // calling clear currents make an inf loop
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = 0.0; // Voltage = (Amps) * (Volts/Amp)
            std::cerr << "Currents Clearing for coil "<<i+1<<std::endl;
        }


    }

    // TODO send current setpoints to amplifiers
    // Now that the correct currents have been found, write to the s826 board outputs if
    // the board is connected.
    if (s826.boardConnected)
    {
        s826.analogWriteAll(s826.rangeCodesDAC, outputAnalogVoltages);
//        qInfo() << "Wrote values to the S826.";
    }


}


void MainWindow::updateCurrents_CalibrationOnly(double I_command[8])
{
    // This is the only function that should tell the S826 to send currents to the coils
    // Reads from Variables:
    //      B_Global_Desired : The desire field at the time
    //      isGradientControlled : Boolean to determine if inverse or pseudoInverse is necessary
    // Sets the Variables:
    //      currentSetpoints : The target current in the coils

    // NOTE: This code finds the inverse of the control matrices every single time it is called.
    //      This isn't necessary if the tool is stationary in the workspace, since the control
    //      matrix, as well as its inverse, will be unchanged.

    // Calling updateCurrents will:
    // 1. Check for overheating
    // 2. Determine the currents needed to generate the desired field
    // 3. Calculate the theoretical magnetic field produced by said currents
    // 4. Send analog output command to change the coil currents if the system is not overheating

    if (!overheatingFlag)
    {
        // Send current setpoints as normal
        // Need to convert to voltages first

        // TODO convert amps to volts
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = I_command[i] * currentControlAdj[i]; // Voltage = (Amps) * (Volts/Amp)
            // TODO limit voltages sent to S826
        }
    }
    else
    {
        // Only send 0s to the currents to turn them off because the system is overheating!
//        clearCurrentsSlot();
        // calling clear currents make an inf loop
        for (int i = 0; i < numAct; i++)
        {
            outputAnalogVoltages[i] = 0.0; // Voltage = (Amps) * (Volts/Amp)
        }
    }
    // TODO send current setpoints to amplifiers
    // Now that the correct currents have been found, write to the s826 board outputs if
    // the board is connected.
    if (s826.boardConnected)
    {
        s826.analogWriteAll(s826.rangeCodesDAC, outputAnalogVoltages);
        qInfo() << "Wrote values to the S826 in updateCurrents_CalibrationOnly submode";
    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CHECKBOX ENABLES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void MainWindow::enableController(void)
{
    // For Enabling or disabling the gamepad or game controller.
    controllerState = ui->checkBox_controllerEnable->checkState();

    // Enable the controller if the checkbox is checked
    if (controllerState)
    {
        connectedGamepad.enableController();
    }
    else
    {
        // If checkbox is unchecked, disable controller and reset field values
        connectedGamepad.disableController();
        // Set field values to zero for safety
        B_Local_Desired[0] = 0;
        B_Local_Desired[1] = 0;
        B_Local_Desired[2] = 0;
        angle1 = 0;
        angle2 = 0;
        angle3 = 0;
        B_mag = 0;
    }

    local2global(tilt, roll, B_Local_Desired, B_Global_Desired);
    // send zeroing commands to the system (because desired fields are 0)
    updateCurrents();
}

void MainWindow::enableGradientControl(void)
{
    // When checkbox is changed, update the boolean
    isGradientControlled = ui->checkBox_gradientControlled->checkState();
    // update the current method of controlling the fields
    updateCurrents();

    if (isGradientControlled)
    {
        // Z fields are largest
        maxB = maxIsoFields[2];
    }
    else
    {
        // Z fields are largest
        maxB = maxAnisoFields[2];
    }

    // Once the maximum possible magnetic field is set we need to change
    // the scrollboxes' and sliders' max range for the Global Field to not go
    // outside the field limits
    ui->doubleSpinBox_Bx_Global->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->doubleSpinBox_By_Global->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->doubleSpinBox_Bz_Global->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->horizontalSlider_Bx_Global->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps
    ui->horizontalSlider_By_Global->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps
    ui->horizontalSlider_Bz_Global->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps

    // We also need to change the scrollboxes' and sliders' max range
    // for the Local Field to not go outside the field limits
    ui->doubleSpinBox_Bx_Local->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->doubleSpinBox_By_Local->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->doubleSpinBox_Bz_Local->setRange( -maxB*1000.0, maxB*1000.0 ); // [mT]
    ui->horizontalSlider_Bx_Local->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps
    ui->horizontalSlider_By_Local->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps
    ui->horizontalSlider_Bz_Local->setRange( round(-maxB*1000000), round(maxB*1000000) ); // [µT] steps

}

void MainWindow::enableSubroutine(void)
{
    // For enabling (starting) or disabling (stopping) the subroutines

    subroutineState = ui->checkBox_startSubthread->checkState();
    qInfo() << "Subroutine state is: " << subroutineState;

    if (subroutineState)
    {
        // Reset time at the start of each subroutine start
        clearTimeSlot();
    }
    else
    {
        // Clear desired fields if the subroutines are turned off
        clearCurrentsSlot();
    }

}

void MainWindow::enableAngleControl(void)
{
    // For enabling the use of control using angles instead of gripper model
    enableAngleControlState = ui->checkBox_angleControlSurgeon->checkState();
}

void MainWindow::enableDAQ()
{
    DAQ.enableDAQ = ui->checkBox_enableDAQ->checkState();
    if (DAQ.isEnabled())
    {
        DAQ.setupTask();
    }
    else
    {
        DAQ.finishTask();
        for ( int i = 0; i < 8; i++ )
        {
            DAQ.analogInputVoltages[i] = 0.0;
        }
    }
}

// PLOTS

void MainWindow::enableMagneticPlot(void)
{
    magneticPlotState = ui->checkBox_enableMagneticPlot->checkState();
}

void MainWindow::enableInputPlot(void)
{
    inputPlotState = ui->checkBox_enableInputPlot->checkState();
}

void MainWindow::enableTemperaturePlot(void)
{
    temperaturePlotState = ui->checkBox_enableTemperaturePlot->checkState();
}

void MainWindow::enableDAQPlot()
{
    DAQPlotState = ui->checkBox_enableDAQPlot->checkState();
}

// SUB PLOTS

void MainWindow::enableSetpointPlot(void)
{
    setpointPlotState = ui->checkBox_showSetpoints->checkState();

    if(!setpointPlotState)
    {
        // checks if the box is set to hidden.
        // hide the plot by plotting blank overtop of those graphs
        // NOTE: the data for setpoints is still saved and can be replotted by enabling
        QVector<double> zeros = {0.0};
        ui->plot_inputs->graph(0)->setData(zeros, zeros);
        ui->plot_inputs->graph(1)->setData(zeros, zeros);
        ui->plot_inputs->graph(2)->setData(zeros, zeros);
        ui->plot_inputs->graph(3)->setData(zeros, zeros);
        ui->plot_inputs->graph(4)->setData(zeros, zeros);
        ui->plot_inputs->graph(5)->setData(zeros, zeros);
        ui->plot_inputs->graph(6)->setData(zeros, zeros);
        ui->plot_inputs->graph(7)->setData(zeros, zeros);
    }
}

void MainWindow::enableGradientPlot(void)
{
    gradientPlotState = ui->checkBox_showGradients->checkState();

    if(!gradientPlotState)
    {
        // checks if the box is set to hidden.
        // hide the plot by plotting blank overtop of those graphs
        // NOTE: the data for gradients is still saved and can be replotted by enabling
        QVector<double> zeros = {0.0};
        ui->plot_magnetic->graph(3)->setData(zeros, zeros);
        ui->plot_magnetic->graph(4)->setData(zeros, zeros);
        ui->plot_magnetic->graph(5)->setData(zeros, zeros);
        ui->plot_magnetic->graph(6)->setData(zeros, zeros);
        ui->plot_magnetic->graph(7)->setData(zeros, zeros);
    }
}

// PLOT LEGENDS

void MainWindow::enableMagneticLegend(void)
{
    magneticLegendState = ui->checkBox_enableMagneticLegend->checkState();

    if(magneticLegendState)
    {
        ui->plot_magnetic->legend->setVisible(true);
    } else
    {
        ui->plot_magnetic->legend->setVisible(false);
    }
}
void MainWindow::enableInputLegend(void)
{
    inputLegendState = ui->checkBox_enableInputLegend->checkState();

    if(inputLegendState)
    {
        ui->plot_inputs->legend->setVisible(true);
    } else
    {
        ui->plot_inputs->legend->setVisible(false);
    }
}

void MainWindow::enableTemperatureLegend(void)
{
    temperatureLegendState = ui->checkBox_enableTemperatureLegend->checkState();

    if(temperatureLegendState)
    {
        ui->plot_temperature->legend->setVisible(true);
    } else
    {
        ui->plot_temperature->legend->setVisible(false);
    }
}

void MainWindow::enableDAQLegend(void)
{
    DAQLegendState = ui->checkBox_enableDAQLegend->checkState();

    if(DAQLegendState)
    {
        ui->plot_DAQ->legend->setVisible(true);
    } else
    {
        ui->plot_DAQ->legend->setVisible(false);
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ARDUINO SLOTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void MainWindow::ToolIN_press_Slot(void)
{
    std::string writeport = gripperControl_port; // Was hardcoded as ACM4
    // writeport = writeport + std::to_string(motorNum);
    //Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    std::string message = "l1,";
    arduino << message;
    // cout << message;
    arduino.close();
}

void MainWindow::ToolINOUT_rel_Slot(void)
{
    std::string writeport = gripperControl_port;
    // writeport = writeport + std::to_string(motorNum);
    //Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    std::string message = "l0,";
    arduino << message;
    // cout << message;
    // std::cout << writeport << "  " << nVal;
    //Print to Arduino via serial
    arduino.close();
}

void MainWindow::ToolINOUT_speed_Slot(int speed)
{
    std::string writeport = gripperControl_port;
    // writeport = writeport + std::to_string(motorNum);
    //Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    qDebug() << speed;
    std::string message = std::to_string(speed); //10-210
    message = "l" + message + ",";
    arduino << message;
    // cout << message;
    // std::cout << writeport << "  " << nVal;
    //Print to Arduino via serial
    arduino.close();
}

void MainWindow::ToolOUT_press_Slot(void)
{
    std::string writeport = gripperControl_port;
    // writeport = writeport + std::to_string(motorNum);
    // Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    std::string message = "l2,";
    arduino << message;
    // cout << message;
    // std::cout << writeport << "  " << nVal;
    //Print to Arduino via serial
    arduino.close();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~ SCROLLBOX SELECTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void MainWindow::controllerModeSlot(int nModeIndex)
{
    // Only necessary component for function is to record the current mode number
    currentControllerMode = nModeIndex;

    // Change GUI variables to show the user relevant information
    switch (currentControllerMode)
    {
    case(3):
        ui->labelname_Type_of_Control->setText("Polar Field Control Mode");
        ui->labelname_Angle1->setText("θ : " ); // Theta
        ui->labelname_Angle2->setText("ϕ : ");  // Phi
        ui->labelname_B_magnitude->setText("B mag : ");
        ui->labelname_factor->setText("N/A");
        break;
    case(4):
        ui->labelname_Type_of_Control->setText("Surgeon Simulator Control Mode");
        ui->labelname_Angle1->setText("γ : "); // Gamma
        ui->labelname_Angle2->setText("β : "); // Beta
        ui->labelname_B_magnitude->setText("B parallel : ");
        ui->labelname_factor->setText("k factor : ");
        break;
    default:
        ui->labelname_Type_of_Control->setText("Not used for this control mode");
        ui->labelname_Angle1->setText("N/A");
        ui->labelname_Angle2->setText("N/A");
        ui->labelname_B_magnitude->setText("N/A");
        ui->labelname_factor->setText("N/A");
        break;
    }
}

void MainWindow::subThreadModeSlot(int nModeIndex)
{
    // This is where to set the labels for any new subrountines added to the code.

    // Only necessary component for function is to record the current mode number
    currentSubroutineMode = nModeIndex;

    // Since the mode changed, it would be good to reset parameters
    // and/or at least disable the subroutine (for now)
    ui->checkBox_startSubthread->setChecked(false);
    subroutineState = false;
    // Don't need to reset the parameter boxes, but do reset the desired fields and currents to 0
    clearCurrentsSlot();

    // TODO Set max and mins for different functions

    // Change GUI variables to show the user relevant information
    switch (currentSubroutineMode)
    {
    case(0):
        // Set all parameter labels to default param0-param4
        ui->lbl_subThreadParam0->setText("param0");
        ui->lbl_subThreadParam1->setText("param1");
        ui->lbl_subThreadParam2->setText("param2");
        ui->lbl_subThreadParam3->setText("param3");
        ui->lbl_subThreadParam4->setText("param4");
        ui->lbl_subThreadUnits0->setText("N/A");
        ui->lbl_subThreadUnits1->setText("N/A");
        ui->lbl_subThreadUnits2->setText("N/A");
        ui->lbl_subThreadUnits3->setText("N/A");
        ui->lbl_subThreadUnits4->setText("N/A");
        break;

    case(1):
        // Rotating Magnetic Field in the XY plane
        ui->lbl_subThreadParam0->setText("Magnitude [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits1->setText("Hz");
        ui->lbl_subThreadParam2->setText("Phase [rad]");
        ui->lbl_subThreadUnits2->setText("rad");
        ui->lbl_subThreadParam3->setText(" ");
        ui->lbl_subThreadUnits3->setText(" ");
        ui->lbl_subThreadParam4->setText(" ");
        ui->lbl_subThreadUnits4->setText(" ");
        break;
    case(2):
        // Rotating Magnetic Field in the XZ plane
        ui->lbl_subThreadParam0->setText("Magnitude [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits1->setText("Hz");
        ui->lbl_subThreadParam2->setText("Phase [rad]");
        ui->lbl_subThreadUnits2->setText("rad");
        ui->lbl_subThreadParam3->setText(" ");
        ui->lbl_subThreadUnits3->setText(" ");
        ui->lbl_subThreadParam4->setText(" ");
        ui->lbl_subThreadUnits4->setText(" ");
        break;
    case(3):
        // Rotating Magnetic Field in the YZ plane
        ui->lbl_subThreadParam0->setText("Magnitude [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits1->setText("Hz");
        ui->lbl_subThreadParam2->setText("Phase [rad]");
        ui->lbl_subThreadUnits2->setText("rad");
        ui->lbl_subThreadParam3->setText(" ");
        ui->lbl_subThreadUnits3->setText(" ");
        ui->lbl_subThreadParam4->setText(" ");
        ui->lbl_subThreadUnits4->setText(" ");
        break;
    case(4):
        // Oscillating Sawtooth Waveform
        ui->lbl_subThreadParam0->setText("Lower Bound [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Upper Bound [mT]");
        ui->lbl_subThreadUnits1->setText("mT");
        ui->lbl_subThreadParam2->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits2->setText("Hz");
        ui->lbl_subThreadParam3->setText("Azimuthal Angle [deg] ");
        ui->lbl_subThreadUnits3->setText("deg");
        ui->lbl_subThreadParam4->setText("Polar Angle [deg]");
        ui->lbl_subThreadUnits4->setText("deg");
        break;
    case(5):
        // Oscillating Triangular Waveform
        ui->lbl_subThreadParam0->setText("Lower Bound [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Upper Bound [mT]");
        ui->lbl_subThreadUnits1->setText("mT");
        ui->lbl_subThreadParam2->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits2->setText("Hz");
        ui->lbl_subThreadParam3->setText("Azimuthal Angle [deg] ");
        ui->lbl_subThreadUnits3->setText("deg");
        ui->lbl_subThreadParam4->setText("Polar Angle [deg]");
        ui->lbl_subThreadUnits4->setText("deg");
        break;
    case(6):
        // Oscillating Square Waveform
        ui->lbl_subThreadParam0->setText("Lower Bound [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Upper Bound [mT]");
        ui->lbl_subThreadUnits1->setText("mT");
        ui->lbl_subThreadParam2->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits2->setText("Hz");
        ui->lbl_subThreadParam3->setText("Azimuthal Angle [deg] ");
        ui->lbl_subThreadUnits3->setText("deg");
        ui->lbl_subThreadParam4->setText("Polar Angle [deg]");
        ui->lbl_subThreadUnits4->setText("deg");
        break;
    case(7):
        // Oscillating Sinusoidal Waveform
        ui->lbl_subThreadParam0->setText("Lower Bound [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Upper Bound [mT]");
        ui->lbl_subThreadUnits1->setText("mT");
        ui->lbl_subThreadParam2->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits2->setText("Hz");
        ui->lbl_subThreadParam3->setText("Azimuthal Angle [deg] ");
        ui->lbl_subThreadUnits3->setText("deg");
        ui->lbl_subThreadParam4->setText("Polar Angle [deg]");
        ui->lbl_subThreadUnits4->setText("deg");
        break;
    case(8):
        // Spherical Coords Field
        ui->lbl_subThreadParam0->setText("Parallel Field Magnitude [mT] ");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Theta: Polar Angle [deg] ");
        ui->lbl_subThreadUnits1->setText("deg");
        ui->lbl_subThreadParam2->setText("Phi: Azimuthal Angle [deg] ");
        ui->lbl_subThreadUnits2->setText("deg");
        ui->lbl_subThreadParam3->setText(" ");
        ui->lbl_subThreadUnits3->setText(" ");
        ui->lbl_subThreadParam4->setText(" ");
        ui->lbl_subThreadUnits4->setText(" ");
        break;
    case(9):
        // calibration data collection mode NN
        // Set all parameter labels to default param0-param4
        ui->lbl_subThreadParam0->setText("param0");
        ui->lbl_subThreadParam1->setText("param1");
        ui->lbl_subThreadParam2->setText("param2");
        ui->lbl_subThreadParam3->setText("param3");
        ui->lbl_subThreadParam4->setText("param4");
        ui->lbl_subThreadUnits0->setText("N/A");
        ui->lbl_subThreadUnits1->setText("N/A");
        ui->lbl_subThreadUnits2->setText("N/A");
        ui->lbl_subThreadUnits3->setText("N/A");
        ui->lbl_subThreadUnits4->setText("N/A");
        break;
    case(10):
        // calibration data collection mode Model
        // Set all parameter labels to default param0-param4
        ui->lbl_subThreadParam0->setText("param0");
        ui->lbl_subThreadParam1->setText("param1");
        ui->lbl_subThreadParam2->setText("param2");
        ui->lbl_subThreadParam3->setText("param3");
        ui->lbl_subThreadParam4->setText("param4");
        ui->lbl_subThreadUnits0->setText("N/A");
        ui->lbl_subThreadUnits1->setText("N/A");
        ui->lbl_subThreadUnits2->setText("N/A");
        ui->lbl_subThreadUnits3->setText("N/A");
        ui->lbl_subThreadUnits4->setText("N/A");
        break;
    case(11):
        // Linearly increasing field
        ui->lbl_subThreadParam0->setText("Starting Field");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Ramp up slope");
        ui->lbl_subThreadUnits1->setText("mT/s");
        ui->lbl_subThreadParam2->setText("Axis");
        ui->lbl_subThreadUnits2->setText(" ");
        ui->lbl_subThreadParam3->setText("Direction");
        ui->lbl_subThreadUnits3->setText(" ");
        ui->lbl_subThreadParam4->setText("Max Value");
        ui->lbl_subThreadUnits4->setText("mT");
        break;
    case(12):
        // Rotating Magnetic Field in the -X-Y45 plane
        ui->lbl_subThreadParam0->setText("Magnitude [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits1->setText("Hz");
        ui->lbl_subThreadParam2->setText("Phase [rad]");
        ui->lbl_subThreadUnits2->setText("rad");
        ui->lbl_subThreadParam3->setText(" ");
        ui->lbl_subThreadUnits3->setText(" ");
        ui->lbl_subThreadParam4->setText(" ");
        ui->lbl_subThreadUnits4->setText(" ");
        break;
    case(13):
        // Rotating Magnetic Field in the X-Y 45 plane
        ui->lbl_subThreadParam0->setText("Magnitude [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits1->setText("Hz");
        ui->lbl_subThreadParam2->setText("Phase [rad]");
        ui->lbl_subThreadUnits2->setText("rad");
        ui->lbl_subThreadParam3->setText(" ");
        ui->lbl_subThreadUnits3->setText(" ");
        ui->lbl_subThreadParam4->setText(" ");
        ui->lbl_subThreadUnits4->setText(" ");
        break;
    case(14):
        // Rotating Magnetic Field in the +45 Y plane
        ui->lbl_subThreadParam0->setText("Magnitude [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits1->setText("Hz");
        ui->lbl_subThreadParam2->setText("Phase [rad]");
        ui->lbl_subThreadUnits2->setText("rad");
        ui->lbl_subThreadParam3->setText(" ");
        ui->lbl_subThreadUnits3->setText(" ");
        ui->lbl_subThreadParam4->setText(" ");
        ui->lbl_subThreadUnits4->setText(" ");
        break;
    case(15):
        // Rotating Magnetic Field in the -45 Y plane
        ui->lbl_subThreadParam0->setText("Magnitude [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits1->setText("Hz");
        ui->lbl_subThreadParam2->setText("Phase [rad]");
        ui->lbl_subThreadUnits2->setText("rad");
        ui->lbl_subThreadParam3->setText(" ");
        ui->lbl_subThreadUnits3->setText(" ");
        ui->lbl_subThreadParam4->setText(" ");
        ui->lbl_subThreadUnits4->setText(" ");
        break;
    case(16):
        // Rotating Magnetic Field in the plane about Y axis at some degree
        ui->lbl_subThreadParam0->setText("Magnitude [mT]");
        ui->lbl_subThreadUnits0->setText("mT");
        ui->lbl_subThreadParam1->setText("Frequency [Hz]");
        ui->lbl_subThreadUnits1->setText("Hz");
        ui->lbl_subThreadParam2->setText("Phase [rad]");
        ui->lbl_subThreadUnits2->setText("rad");
        ui->lbl_subThreadParam3->setText("Plane tilted [rad] ");
        ui->lbl_subThreadUnits3->setText("rad ");
        ui->lbl_subThreadParam4->setText(" ");
        ui->lbl_subThreadUnits4->setText(" ");
        break;

    default:
        // Set all parameter labels to default param0-param4
        ui->lbl_subThreadParam0->setText("param0");
        ui->lbl_subThreadParam1->setText("param1");
        ui->lbl_subThreadParam2->setText("param2");
        ui->lbl_subThreadParam3->setText("param3");
        ui->lbl_subThreadParam4->setText("param4");
        ui->lbl_subThreadUnits0->setText("N/A");
        ui->lbl_subThreadUnits1->setText("N/A");
        ui->lbl_subThreadUnits2->setText("N/A");
        ui->lbl_subThreadUnits3->setText("N/A");
        ui->lbl_subThreadUnits4->setText("N/A");
        break;
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~ SLIDERS AND SPINBOX ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void MainWindow::sliderMovingSlot(int value)
{
//    qDebug() << "A slider is moving";
//    qInfo() << "Value is: " << value;

    // Recompute new local fields or global fields, depending on what slider is moved
    if ( ui->horizontalSlider_Bx_Global->isSliderDown() )  // Check the GLOBAL frame sliders (edit LOCAL fields accordingly)
    {
        B_Global_Desired[0] = (double) value/1000000.0;
        global2local(tilt,roll,B_Global_Desired,B_Local_Desired);
    }
    else if  ( ui->horizontalSlider_By_Global->isSliderDown() )
    {
        B_Global_Desired[1] = (double) value/1000000.0;
        global2local(tilt,roll,B_Global_Desired,B_Local_Desired);
    }
    else if  ( ui->horizontalSlider_Bz_Global->isSliderDown() )
    {
        B_Global_Desired[2] = (double) value/1000000.0;
        global2local(tilt,roll,B_Global_Desired,B_Local_Desired);
    }
    else if ( ui->horizontalSlider_Bx_Local->isSliderDown() ) // Check the LOCAL frame sliders as well (edit GLOBAL fields accordingly)
    {
        B_Local_Desired[0] = (double) value/1000000.0;
        local2global(tilt,roll,B_Local_Desired,B_Global_Desired);
        // TODO Transform coords
    }
    else if  ( ui->horizontalSlider_By_Local->isSliderDown() )
    {
        B_Local_Desired[1] = (double) value/1000000.0;
        local2global(tilt,roll,B_Local_Desired,B_Global_Desired);
        // TODO Transform coords
    }
    else if  ( ui->horizontalSlider_Bz_Local->isSliderDown() )
    {
        B_Local_Desired[2] = (double) value/1000000.0;
        local2global(tilt,roll,B_Local_Desired,B_Global_Desired);
        // TODO Transform coords
    }

    // Update and send current commands to the system.
    updateCurrents();
}

void MainWindow::setFieldSpinBoxSlot(void)
{
//    qDebug() << "SPINBOX CHANGED";
//    qInfo() << ui->doubleSpinBox_Bx_Global->value();
    // Check what spinbox is in focus
    // edit the right desired field value
    if (ui->doubleSpinBox_Bx_Global->hasFocus()) // GLOBAL FIELD VALUES
    {
            B_Global_Desired[0] = ui->doubleSpinBox_Bx_Global->value()/1000.0; // [T] Change from mT to T
            global2local(tilt,roll,B_Global_Desired,B_Local_Desired);
    }
    else if (ui->doubleSpinBox_By_Global->hasFocus())
    {
            B_Global_Desired[1] = ui->doubleSpinBox_By_Global->value()/1000.0; // [T] Change from mT to T
            global2local(tilt,roll,B_Global_Desired,B_Local_Desired);
    }
    else if (ui->doubleSpinBox_Bz_Global->hasFocus())
    {
            B_Global_Desired[2] = ui->doubleSpinBox_Bz_Global->value()/1000.0; // [T] Change from mT to T
            global2local(tilt,roll,B_Global_Desired,B_Local_Desired);
    }
    else if (ui->doubleSpinBox_Bx_Local->hasFocus()) // LOCAL FIELD VALUES
    {
            B_Local_Desired[0] = ui->doubleSpinBox_Bx_Local->value()/1000.0; // [T] Change from mT to T
            local2global(tilt,roll,B_Local_Desired,B_Global_Desired);
    }
    else if (ui->doubleSpinBox_By_Local->hasFocus())
    {
            B_Local_Desired[1] = ui->doubleSpinBox_By_Local->value()/1000.0; // [T] Change from mT to T
            local2global(tilt,roll,B_Local_Desired,B_Global_Desired);
    }
    else if (ui->doubleSpinBox_Bz_Local->hasFocus())
    {
            B_Local_Desired[2] = ui->doubleSpinBox_Bz_Local->value()/1000.0; // [T] Change from mT to T
            local2global(tilt,roll,B_Local_Desired,B_Global_Desired);
    }

    updateCurrents();
}

//~~~~~~~~~~~~~~~~~~~~ TILT AND ROLL OF LOCAL FRAME ~~~~~~~~~~~~~~~~~~~~~~~~~~/
void MainWindow::setTiltSlot(void)
{
    // GUI in degrees. Computation in radians
    tilt = (ui->doubleSpinBox_tilt->value()) * M_PI / 180.0;
}

void MainWindow::setRollSlot(void)
{
    // GUI in degrees. Computation in radians
    roll = (ui->doubleSpinBox_roll->value()) * M_PI / 180.0;

    // Calculate the new equivalent desired global field
    local2global(tilt, roll, B_Local_Desired, B_Global_Desired);
    // Calling updateCurrents will:
    // 1. Check for overheating
    // 2. Determine the currents needed to generate the desired field
    // 3. Calculate the theoretical magnetic field produced by said currents
    // 4. Send analog output command to change the coil currents if the system is not overheating
    updateCurrents();

    // Send roll value to the arduino to initiate roll
    std::string writeport = gripperControl_port;

    int nVal = (int) ( (ui->doubleSpinBox_roll->value())*(EncoderStepConversionTool/2)/180 + 50000 );
    //Print to Arduino via serial
    std::ofstream arduino;
    arduino.open( writeport );
    std::string position = std::to_string( nVal );
    position = 'r' + position + ',';
    arduino << position;
    qInfo() << "Command: " << QString::fromStdString(position) << " sent to Trocar Arduino.";
    // cout << position;
    arduino.close();

}

void MainWindow::setFactorSlot(void)
{
    k_factor = (ui->doubleSpinBox_factor->value()) / 1000;
}



void MainWindow::subThreadParameterChangeSlot(void)
{
    // Check which spinbox had its value changed and record the value to the array of parameters
    if (ui->dsb_subThreadParam0->hasFocus())
    {
        subThreadParameters[0] = ui->dsb_subThreadParam0->value();
    }
    else if (ui->dsb_subThreadParam1->hasFocus())
    {
        subThreadParameters[1] = ui->dsb_subThreadParam1->value();
    }
    else if (ui->dsb_subThreadParam2->hasFocus())
    {
        subThreadParameters[2] = ui->dsb_subThreadParam2->value();
    }
    else if (ui->dsb_subThreadParam3->hasFocus())
    {
        subThreadParameters[3] = ui->dsb_subThreadParam3->value();
    }
    else if (ui->dsb_subThreadParam4->hasFocus())
    {
        subThreadParameters[4] = ui->dsb_subThreadParam4->value();
    }

}


void MainWindow::clearCurrentsSlot(void)
{
    // Clear setpoints and desired fields to 0
    for (int i = 0; i<8; i++)
    {
        currentSetpoints[i] = 0.0;
        B_Local_Desired[i] = 0.0;
        B_Global_Desired[i] = 0.0;
        B_Global_Output[i] = 0.0;
    }


    // TODO Check that this is up to date

//    if (s826.boardConnected)
//    {
//        // Write zero setpoint values to DAC
////        s826.analogWriteAll(s826.rangeCodesDAC, currentSetpoints);
//    }
    // Regardless of board connectivity the labels should set to zero as well

    updateCurrents();



    // testing print statements. pls delete
//    qInfo() << measuredTemperatures[0];
//    qInfo() << measuredTemperatures[1];
//    qInfo() << measuredTemperatures[2];
//    qInfo() << measuredTemperatures[3];
//    qInfo() << measuredTemperatures[4];

}

void MainWindow::clearTimeSlot(void)
{
//    currentTime = 0.0;
    currentTime.restart();
    // Magnetic Field Plot
    qv_t_mag.clear();
    qv_Bx.clear();
    qv_By.clear();
    qv_Bz.clear();
    qv_dBxdx.clear();
    qv_dBxdy.clear();
    qv_dBxdz.clear();
    qv_dBydy.clear();
    qv_dBydz.clear();
    // Currents Plot
    qv_t_input.clear();
    qv_setpoint0.clear();
    qv_setpoint1.clear();
    qv_setpoint2.clear();
    qv_setpoint3.clear();
    qv_setpoint4.clear();
    qv_setpoint5.clear();
    qv_setpoint6.clear();
    qv_setpoint7.clear();
    qv_current0.clear();
    qv_current1.clear();
    qv_current2.clear();
    qv_current3.clear();
    qv_current4.clear();
    qv_current5.clear();
    qv_current6.clear();
    qv_current7.clear();
    // Thermocouple Plot
    qv_t_thermocouple.clear();
    qv_temp0.clear();
    qv_temp1.clear();
    qv_temp2.clear();
    qv_temp3.clear();
    qv_temp4.clear();
    qv_temp5.clear();
    qv_temp6.clear();
    qv_temp7.clear();
    // DAQ Plot
    qv_t_DAQ.clear();
    qv_DAQai0.clear();
    qv_DAQai1.clear();
    qv_DAQai2.clear();
    qv_DAQai3.clear();
    qv_DAQai4.clear();
    qv_DAQai5.clear();
    qv_DAQai6.clear();
    qv_DAQai7.clear();
}

void MainWindow::startDAQRecording()
{
    // First check that data is not already being recorded
    if(!isRecordingData)
    {
        // TODO: OPEN file based on current time
        // Start adding data points to file every iteration until stop is called.
        isRecordingData = true;
        // Turn ON recording LIGHT
        ui->label_DAQRecordIcon->setHidden(false);

        // Find time to create a unique filename for saving
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

//        strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo); // Filename cannot contain ":" symbol
        strftime(buffer,sizeof(buffer),"%Y-%m-%d_%H-%M-%S",timeinfo);
        std::string temp(buffer);
        filename = QString::fromStdString(temp);
        filename = DATA_SAVE_PATH + filename + ".txt"; // This file will not be changed until a new recording is started

        QFile file(filename);
        if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
            QTextStream stream(&file);

            // Check the type of recording here
            // TODO keep these header labels up to date.
            if (dataRecordingState == 0)
            {
                // Continuous Recording
                stream << "Time\t" << "Bx\t" << "By\t" << "Bz\t" << "\n";
            }
            else if (dataRecordingState == 1)
            {
                // Discrete Recording
                stream << "Time[s]\t" << "Bx [mT]\t" << "By [mT]\t" << "Bz [mT]\t" << "THETA\t"  << "PHI\t" << "Fx\t" << "Fy\t" << "Fz\t" << "Tx\t" << "Ty\t" << "Tz\t" << "V0\t" << "V1\t" << "V2\t" << "V3\t" << "V4\t" << "V5\t" << "V6\t" << "V7\t" << "\n";
//                stream << "Time[s]\t" << "Bx [mT]\t" << "By [mT]\t" << "Bz [mT]\t" << "THETA\t"  << "PHI\t" << "DAQ_v0\t" << "DAQ_v1\t" << "DAQ_v2\t" << "DAQ_v3\t" << "DAQ_v4\t" << "DAQ_v5\t" << "DAQ_v6\t" << "DAQ_v7\t" << "\n";
            }
            else if (dataRecordingState == 2)
            {
                stream << "Time(s)\t " << "ProbeBx(mT)\t" << "ProbeBy(mT)\t" << "ProbeBz(mT)\t" << "ProbePx(mm)\t"  << "ProbePy(mm)\t" << "ProbePz(mm)\t" << "Current_1(A)\t"<< "Current_2(A)\t" <<
                            "Current_3(A)\t" << "Current_4(A)\t" << "Current_5(A)\t" << "Current_6(A)\t" << "Current_7(A)\t" << "Current_8(A)\t" << "Daq_raw1(v)\t" << "Daq_raw2(v)\t" << "Daq_raw3(v)\t" << "\n";

            }


            file.close();
        }
        else
        {
            qWarning() << "Problem occured initializing the datafile.";
        }

    }
    else
    {
        qInfo() << "Data recording has already been started. Stop current recording before attempting to start a new record.";
    }
}

void MainWindow::setDAQRecordingState(void)
{
    // Do not allow a state change if Data is already being recorded
    if (!isRecordingData)
    {
        if (ui->pushButton_StartRecordDAQ->hasFocus())
        {
            dataRecordingState = 0;
        }
        else if (ui->pushButton_openDiscreteDataFile->hasFocus())
        {
            dataRecordingState = 1;
        }
        else if (ui->pushButton_ClibrationRecord->hasFocus())
        {
            dataRecordingState = 2;
        }

        startDAQRecording();
    }


}

void MainWindow::recordDAQDatapoint()
{
    if ( isRecordingData & (dataRecordingState == 0) ) // recording and set to continuous recording
    {
        // TODO: Add the points from this iteration into the data file.
        // TODO: Check if DAQ.isEnabled to write those points as well
        QFile file(filename);
        if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
        {
            QTextStream stream(&file);
//            stream << "Time" << "Bx" << "By" << "Bz" << endl;
            stream << currentTime.elapsed()/1000.0 << "\t" << B_Global_Output[0] << "\t" << B_Global_Output[1] << "\t" << B_Global_Output[2] << "\t" << "\n";
            // TODO add more values to save here
            // TODO add daq check and save daq values as well (at the ends)
            file.close();
        }
        else
        {
            qWarning() << "Problem occured while writing to the datafile.";
        }
    }
    else if ( isRecordingData & (dataRecordingState == 1) ) // recording and set to discrete recording
    {
        // TODO: Add the points from this iteration into the data file.
        // TODO: Check if DAQ.isEnabled to write those points as well
        // currently just save all regardless of state of DAQ
        QFile file(filename);
        if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
        {
            QTextStream stream(&file);
//            stream << "Time " << "Bx" << "By" << "Bz" << "THETA\t"  << "PHI2\t" << "Fx\t" << "Fy\t" << "Fz\t" << "Tx\t" << "Ty\t" << "Tz\t" << "\n" << endl;
            stream << currentTime.elapsed()/1000.0 << "\t" << B_Global_Output[0]*1000.0 << "\t" << B_Global_Output[1]*1000.0 << "\t" << B_Global_Output[2]*1000.0 << "\t" <<
                      subThreadParameters[1] << "\t" << subThreadParameters[2]<< "\t" <<
                      ATINanoForceTorque[0] << "\t" << ATINanoForceTorque[1] << "\t" << ATINanoForceTorque[2] << "\t" <<
                      ATINanoForceTorque[3] << "\t" << ATINanoForceTorque[4] << "\t" << ATINanoForceTorque[5] << "\t" <<
                      outputAnalogVoltages[0] << "\t" << outputAnalogVoltages[1] << "\t" << outputAnalogVoltages[2] << "\t" << outputAnalogVoltages[3] << "\t" <<
                      outputAnalogVoltages[4] << "\t" << outputAnalogVoltages[5] << "\t" << outputAnalogVoltages[6] << "\t" << outputAnalogVoltages[7] << "\t" << "\n";
//            stream << currentTime.elapsed()/1000.0 << "\t" << B_Global_Output[0]*1000.0 << "\t" << B_Global_Output[1]*1000.0 << "\t" << B_Global_Output[2]*1000.0 << "\t" <<
//                      subThreadParameters[1] << "\t" << subThreadParameters[2]<< "\t" <<
//                      DAQ.analogInputVoltages[0] << "\t" << DAQ.analogInputVoltages[1] << "\t" << DAQ.analogInputVoltages[2] << "\t" << DAQ.analogInputVoltages[3] << "\t" <<
//                      DAQ.analogInputVoltages[4] << "\t" << DAQ.analogInputVoltages[5] << "\t" << DAQ.analogInputVoltages[6] << "\t" << DAQ.analogInputVoltages[7] << "\t" << "\n";
//            stream << currentTime.elapsed()/1000.0 << "\t" << DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0] << "\t" << DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1] << "\t" << DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2] << "\t" <<
//                      DAQ.analogRawInputVoltages[0] << "\t" << DAQ.analogRawInputVoltages[1] << "\t" << DAQ.analogRawInputVoltages[2] << "\t" << "\n";

            // TODO add more values to save here
            // TODO add daq check and save daq values as well (at the ends)
            qInfo() << "Recorded Datapoint for Theta = " << subThreadParameters[1] << " deg, Phi = " << subThreadParameters[2] << " deg.";
            file.close();
        }
        else
        {
            qWarning() << "Problem occured while writing to the datafile.";
        }
    }
    else if ( isRecordingData & (dataRecordingState == 2) ) // recording for field calibration
    {
        // TODO: Add the points from this iteration into the data file.
        // TODO: Check if DAQ.isEnabled to write those points as well
        // currently just save all regardless of state of DAQ
        if (gantryinitflag == true && Datacollectdoneflag == false)
        {
            QFile file(filename);
            if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
            {
                QTextStream stream(&file);
    //            stream << "Time " << "ProbeBx(mT)\t" << "ProbeBy(mT)\t" << "ProbeBz(mT)\t" << "ProbePx(mm)\t"  << "ProbePy(mm)\t" << "ProbePz(mm)\t" << "Current_1(A)\t"<< "Current_2(A)\t"
    //            << "Current_3(A)\t" << "Current_4(A)\t" << "Current_5(A)\t" << "Current_6(A)\t" << "Current_7(A)\t" << << "Current_8(A)\t" << "Daq_raw1(v)\t" << "Daq_raw2(v)\t" << "Daq_raw3(v)\t" << "\n" << endl;
                //convert the gantry abs position to coil system coordinates
                double Px_coilframe = gantryorigin[1] - gantryPos[1]/gantryStepsPerMM;
                double Py_coilframe = gantryorigin[0] - gantryPos[0]/gantryStepsPerMM;
                double Pz_coilframe = gantryPos[2]/gantryStepsPerMM - gantryorigin[2];

                //covert probe reading to filed at coil system frame
                // Bx = ProbeBy
                // By = -ProbeBx
                // Bz = ProbeBz
                double Bx = DAQ.analogRawInputVoltages[1]*gaussCalibCons_new[1];
                double By = -DAQ.analogRawInputVoltages[0]*gaussCalibCons_new[0];
                double Bz = DAQ.analogRawInputVoltages[2]*gaussCalibCons_new[2];

                stream << currentTime.elapsed()/1000.0 << "\t" << Bx << "\t" << By << "\t" << Bz << "\t" <<
                          Px_coilframe << "\t" <<  Py_coilframe << "\t" <<  Pz_coilframe<< "\t" <<
                          measuredCurrents[0] << "\t" << measuredCurrents[1] << "\t" <<measuredCurrents[2] << "\t" <<measuredCurrents[3] << "\t" <<measuredCurrents[4] << "\t" <<measuredCurrents[5] << "\t" <<measuredCurrents[6] << "\t" <<measuredCurrents[7] << "\t"  <<
                          DAQ.analogRawInputVoltages[0] << "\t" << DAQ.analogRawInputVoltages[1] << "\t" << DAQ.analogRawInputVoltages[2] << "\t" << "\n";
               // currentSetpoints[8] is the output current to S826 board
                // measuredCurrents[8] is the feedback current read from S826
                // TODO add more values to save here: gantry xyz coordinates,
                // TODO add daq check and save daq values as well (at the ends)
                file.close();
            }
            else
            {
                qWarning() << "Problem occured while writing to the datafile.";
            }
        }


    }

}

void MainWindow::stopDAQRecording()
{
    // First check that data is currently being recorded
    if(isRecordingData)
    {


        // TODO: CLOSE and safe file
        // TODO Start adding data points to file every iteration until stop is called.
        isRecordingData = false;
        // Turn OFF recording LIGHT
        ui->label_DAQRecordIcon->setHidden(true);
        qInfo() << "Data recording stopped. Saved under <data> with filename: " << filename;
    }
    else
    {
        qInfo() << "No data recording is currently ongoing.";
    }
}

void MainWindow::changeRoll(int dir)
{
    double temp_val = (ui->doubleSpinBox_roll->value());
//    if ( abs(temp_val) > ui->doubleSpinBox_roll->maximum() ) not necessary
//    {
//        dir = 0;
//        ui->doubleSpinBox_roll->setValue(temp_val/abs(temp_val) * ui->doubleSpinBox_roll->maximum() );
//    }
//    else
//    {
//        ui->doubleSpinBox_roll->setValue(temp_val+0.5*dir);
//    }
    ui->doubleSpinBox_roll->setValue(temp_val+0.5*dir);

    if (dir!=0)
    {
        setRollSlot();
    }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~ RECONNECT FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void MainWindow::reconnectGamepadSlot(void)
{
    // Only allow if gamepad is not enabled (for safety reasons)
    if (!controllerState)
    {
        connectedGamepad.reconnectController();
    }
    else
    {
        qInfo() << "Ensure the controller is not enabled before attempting to reconnect.";
    }
}

void MainWindow::reconnectS826(void)
{
    // Only allow if S826 is not already connected
    if (!s826.boardConnected)
    {
        s826.init();
    }
    else
    {
        qInfo() << "The S826 board is already connected.";


        // TODO: Clear these comments

        // this was a test function
//        uint counter    = 0;            // counter channel to use as a periodic timer (0 to 5)
//        uint period     = 500000;       // timer interval in microseconds (500000 = 0.5 seconds)

//        int i;
//        uint tstamp;    // counter snapshot's timestamp
//        uint counts;

//        s826.periodicTimerStart(counter, period);     // configure counter as periodic timer and start it running.

//        for (i = 0; i < 10; i++)                                // repeat a few times ...
//        {
//            s826.periodicTimerWait(counter, &tstamp);     // wait for timer snapshot
//            S826_CounterRead(BOARD, counter, &counts);      // get counts - just to exercise CounterRead
//            qInfo() << "timestamp: " << (uint) tstamp << " counts: " << counts; // report timestamp
//        }

//        qInfo() << s826.periodicTimerStop(counter);              // halt timer

//        s826.periodicTimerClear(counter);


    }
}

void MainWindow::udpintialize()
{

    // Initialize Winsock
  /*  printf("Initialising Winsock...\n");
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        printf("Failed. Error code: %d\n", WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");

    // Create a SOCKET for the server to listen for client connections
    ConnectSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ConnectSocket == INVALID_SOCKET)
    {
        printf("Error at socket(): %d\n", WSAGetLastError());
        WSACleanup();
        exit(EXIT_FAILURE);
    }
    printf("Socket created.\n");

    // Setup address structure
    serverSin.sin_family = AF_INET;
    serverSin.sin_addr.s_addr = INADDR_ANY;
    serverSin.sin_port = htons(SERVER_PORT);

    // Bind
    if (bind(ConnectSocket, (struct sockaddr*)&serverSin, serverSinSize) == SOCKET_ERROR)
    {
        printf("Bind failed with error code: %d", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        exit(EXIT_FAILURE);
    }
    printf("Binding done.\n");*/

 /*   int sockfd;
        char buffer[MAXLINE];
        char hello = 'S';
        struct sockaddr_in servaddr, cliaddr;

        // Creating socket file descriptor
        if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }

        memset(&servaddr, 0, sizeof(servaddr));
        memset(&cliaddr, 0, sizeof(cliaddr));

        // Filling server information
        servaddr.sin_family    = AF_INET; // IPv4
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(PORT);

        // Bind the socket with the server address
        if ( bind(sockfd, (const struct sockaddr *)&servaddr,
                sizeof(servaddr)) < 0 )
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        int len, n;
        bool MSG_WAITALL = true;

        len = sizeof(cliaddr);  //len is value/result

        n = recvfrom(sockfd, (char *)buffer, MAXLINE,
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                    &len);
        buffer[n] = '\0';
        printf("Client : %s\n", buffer);
//        sendto(sockfd, (const char *)hello, strlen(hello),
//            MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
//                len);
        printf("Hello message sent.\n");


//    printf("\n");*/
}

void MainWindow::processPendingDatagrams()
 {
    QHostAddress sender;
    uint16_t port;
    if (socket->hasPendingDatagrams())
    {
         QByteArray datagram;
         datagram.resize(socket->pendingDatagramSize());
         socket->readDatagram(datagram.data(),datagram.size(),&sender,&port);

//        qDebug() <<"Message From :: " << sender.toString();
//        qDebug() <<"Port From :: "<< port;
//        qDebug() <<"Message :: " << datagram;

        PBYTE pPacketByte = reinterpret_cast<byte*>(datagram.data());

            // Left Device
            for (int i = 0; i < 3; i++)
            {
//                m_Outputs.LeftDesiredPosition[i] = datagram.toFloat();

//                m_Outputs.LeftDesiredPosition[i] = qToLittleEndian(*((unsigned __int64*)pPacketByte));
                m_Outputs.LeftDesiredPosition[i] = ntohd(*((unsigned __int64*)pPacketByte));
                pPacketByte += sizeof(unsigned __int64);
            }
            for (int i = 0; i < 3; i++)
            {
                m_Outputs.LeftHaptic.Gimbal[i] = ntohd(*((unsigned __int64*)pPacketByte));
                pPacketByte += sizeof(unsigned __int64);
            }

            m_Outputs.LeftHaptic.GreyButton = *((unsigned char*)pPacketByte);
            pPacketByte += sizeof(unsigned char);
            m_Outputs.LeftHaptic.WhiteButton = *((unsigned char*)pPacketByte);
//        qDebug() <<"Message Desired Position :: " << m_Outputs.LeftDesiredPosition[0] << m_Outputs.LeftDesiredPosition[1]<< m_Outputs.LeftDesiredPosition[2];
            std::cout <<"Message Desired Gimbal :: " << m_Outputs.LeftHaptic.Gimbal[0]*180/M_PI <<std::endl; // << m_Outputs.LeftHaptic.Gimbal[1]*180/M_PI<< m_Outputs.LeftHaptic.Gimbal[2]*180/M_PI;
            std::cout <<"Message Grey button:: " << m_Outputs.LeftHaptic.GreyButton <<std::endl;
//        qDebug() <<"Message white button:: " << m_Outputs.LeftHaptic.WhiteButton;

    }
}

void MainWindow::enableUDP(void)
{
    // When checkbox is changed, update the boolean
    UDPflag = ui->checkBox_udpmode->checkState();
    std::cout<< "UPD mode:: "<<UDPflag<<std::endl;
    for (int k=0; k<numField+numGrad; k++) {
        B_Global_Desired[k] = 0.0;
    }
    updateCurrents();
}

void MainWindow::enablesampleBxy()
{
    xysampleflag = ui->checkBox_sampleBxy->checkState();
    std::cout<< "xysampleflag:: "<<xysampleflag<<std::endl;
    samplecount = 0;
}

void MainWindow::sampleBxy()
{
    samplecount++;
    std::cout<< "samplecount:: "<<samplecount<<std::endl;
}
