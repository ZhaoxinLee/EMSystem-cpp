#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <fstream>
#include <QtSerialPort/QSerialPort>
#include <QSerialPortInfo>
#include <QFile>

// all important math functions and libraries are contained in "magneticmathfunctions.h"

#include "magneticmathfunctions.h"
//#include "callbacks.h"   // Does this need to be called?

#include "s826.h"               // The necessary API is included within this include.

#include "daq.h"

#include "ui_mainwindow.h"
#include "gamepadmonitor.h"
#include <QtGamepad/QGamepad>
#include "qcustomplot.h"

#include <QDebug>
#include <QTimer>
#include <ctime>



//#include <winsock.h>

#include <WinSock2.h>
#include <QUdpSocket>
#include  <QtEndian>


// TODO: REMOVE
//// MOVED THESE TO MAGNETICMATHFUNCTIONS.H
//const double mu_0 = 3.14159265359 * 4e-7;
//const double Br = 1.457; // For Gripper experiments: 1.47; // [T]
//const double M = 1/mu_0*Br;
//const double vAct = 0.0508*0.0508*0.0508; // [m^3]

#define SERIAL_TROCAR  0
#define SERIAL_GANTRY  1

#define GANTRY_MOVE_SINGLE  0
#define GANTRY_MOVE_MULTI   1
#define GANTRY_ZERO         2
#define GANTRY_CENTER       3

#define GANTRY_MOVE_REL  0
#define GANTRY_MOVE_ABS  1

#define GANTRY_X 0
#define GANTRY_Y 1
#define GANTRY_Z 2

#define GANTRY_POS 1
#define GANTRY_NEG 0

// These are defined in the S826api header internally. Left here for reference
#define AOUT0_PIN   42
#define AOUT1_PIN   44
#define AOUT2_PIN   46
#define AOUT3_PIN   48
#define AOUT4_PIN   41
#define AOUT5_PIN   43
#define AOUT6_PIN   45
#define AOUT7_PIN   47

// AIN0 - AIN7  ARE CURRENT SENSE FROM EM0-EM7, respectively
#define NEG_AIN0_PIN    3
#define POS_AIN0_PIN    4
#define NEG_AIN1_PIN    5
#define POS_AIN1_PIN    6
#define NEG_AIN2_PIN    7
#define POS_AIN2_PIN    8
#define NEG_AIN3_PIN    9
#define POS_AIN3_PIN    10
#define NEG_AIN4_PIN    11
#define POS_AIN4_PIN    12
#define NEG_AIN5_PIN    13
#define POS_AIN5_PIN    14
#define NEG_AIN6_PIN    15
#define POS_AIN6_PIN    16
#define NEG_AIN7_PIN    17
#define POS_AIN7_PIN    18
// AIN8 - AIN15 ARE THERMOCOUPLE SENSE FROM EM0 - EM7, respectively
#define NEG_AIN8_PIN    21
#define POS_AIN8_PIN    22
#define NEG_AIN9_PIN    23
#define POS_AIN9_PIN    24
#define NEG_AIN10_PIN   25
#define POS_AIN10_PIN   26
#define NEG_AIN11_PIN   27
#define POS_AIN11_PIN   28
#define NEG_AIN12_PIN   29
#define POS_AIN12_PIN   30
#define NEG_AIN13_PIN   31
#define POS_AIN13_PIN   32
#define NEG_AIN14_PIN   33
#define POS_AIN14_PIN   34
#define NEG_AIN15_PIN   35
#define POS_AIN15_PIN   36

#define BUFLEN (512)				// Max length of UDP messages
#define SERVER_PORT 45454			// The port on which the server listens for incoming data
//#define PORT     8080
//#define MAXLINE 1024

//~~~~~ INITIALIZE CLASS AND FUNCTIONS ~~~~~//
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    //~~~~~ INITIALIZE MAGNETIC SYSTEM PARAMETERS ~~~~~//

    //~~~~~~~~~~~~ PERMANENT MAGNET SYSTEM ~~~~~~~~~~~~//
    // Constant regardless of setup
    const int EncoderStepConversion = 3000; // Number of encoder steps per revolution
    const int EncoderStepConversionTool = 360; //12000; // Number of encoder steps per revolution
    // Changes if setup changes or based on tuning/calibration
    const double r_mag = 0.156; //0.1259; // [m]
    const double minAllowableGrad = 0.001; // minimum allowable gradient of field
    // Considering the 45 deg orientation of the system:

    //~~~~~~~~~~~~ ELECTROMAGNETIC ACTUATION SYSTEM ~~~~~~~~~~~~//
    const static int numAct = 8; // Shouldn't ever change
    const static int numField = 3;
    const static int numGrad = 5;

    constexpr static double maxAllowableTemp  = 90.0;  // [C] Maximum temperature that the system is allowed to operate at. Kill currents if temperature exceeds
    constexpr static double maxCurrent = 24.0; // [A]
    // MAX COMMAND SIGNALS Found through current experimentation with clamp meter -Adam 2021/09/13
//    constexpr static double maxCurrentCommand[numAct] = { 3.5555, 3.5976, 3.7431, 6.1806, 3.5449, 3.5449, 3.5764, 3.5036 }; // [V] For max 24 A
    constexpr static double maxCurrentCommand[numAct] = { 3.5555/2.0, 3.5976/2.0, 3.7431/2.0, 3.5389/2.0, 3.5449/2.0, 3.5449/2.0, 3.5764/2.0, 3.5036/2.0 }; // [V] For max 12 A (old EM7 6.1806/2.0)

    constexpr static double deg2rad = M_PI/180.0; // factor for changing degrees to radians

//    const double d1 =  0.287; // [m] position in meters
//    const double d2 =  0.093; // [m] position in meters
//    const double h1 = -0.300; // [m] position in meters
//    // Shorten to pAct once pAct is removed
//    double pAct_cartesion[numField][numAct] = {
//        { -d1, 0.0,  d1, 0.0, -d2,  d2,  d2, -d2 },
//        { 0.0, -d1, 0.0,  d1, -d2, -d2,  d2,  d2 },
//        {  h1,  h1,  h1,  h1,  h1,  h1,  h1,  h1 }
//        };

    // Considering the system as built:
    // Declare the Electromagnets (EM1, EM2, EM3, EM4, EM5, EM6, EM7, EM8)
    // in the order that they are wired. See comment below
    const double d1 =  0.287/sqrt(2.0);
    const double d2 =  0.093*sqrt(2.0);
    const double h1 = -0.300;
    double pAct_cartesion[numField][numAct] = {
    //    EM1, EM4, EM6, EM7, EM2, EM8, EM5, EM3  (As wired)
        { -d1, 0.0,  d1,  d2, -d2,  d1, 0.0, -d1 },
        { -d1, -d2, -d1, 0.0, 0.0,  d1,  d2,  d1 },
        {  h1,  h1,  h1,  h1,  h1,  h1,  h1,  h1 }
        };

//    const double m = 6324.48; // [Am^2] this is the theoretical magnetic moment on each EM based on simulation
    // 2*pi*(0.12+0.360/2)^3*(0.035366483)/mu_0 = 4.7745e+03 [A m^2] From 2021/01/13 COMSOL Simulations
    const double m = 4774.5; // [Am^2] this is most recent number. It is also similar to a volume-scaled permanent magnet
    double mAct_cartesion[numField][numAct] = { // All are in positive z direction
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {   m,   m,   m,   m,   m,   m,   m,   m}
        };

    // comes from 2*pi*(0.12+0.360/2)^3*(0.046848)/mu_0;
    // Should there be a calibrated m for each core?


    // Define ports for motors 0 - 3 and IN/OUT ROLL Arduino
    // Motor Arduinos 0-3
    const std::string motor0_port = "/dev/ttyACM0"; // by default: ACM0
    const std::string motor1_port = "/dev/ttyACM2"; // by default: ACM1
    const std::string motor2_port = "/dev/ttyACM1"; // by default: ACM2
    const std::string motor3_port = "/dev/ttyACM4"; // by default: ACM3
    // IN/OUT ROLL Arduino
    const std::string gripperControl_port = "/dev/ttyACM3";  // by default: ACM4
    // Add another for the gantry control

    // Arduinos for control
    std::ofstream arduinoTrocar;
    std::ofstream arduinoGantry;
    QSerialPort serial[2];

    QString filename;
//    QString DATA_SAVE_PATH = "D:/Users/Adam/Documents/001_Work/001_MASc/013-C++/Thesis/Clinical-Electromagnetic-Actuation-System-Core/data";
    QString DATA_SAVE_PATH = "../../data/";

    // Untethered magnetic tool parameters
    double mTool_dir[3] = {0, 0, -1}; // tool orientation
    const double mTool_mag = 8.4e-3;  // tool dipole moment magnetiude [Am^2]
    double maxB, maxF, maxG;
    double maxAnisoFields[3];
    double maxIsoFields[3];

    //Field Calibration Parameters
    int count_a = 0;
    int count_i = 0;
//    double currentpool[10] ={-23,-18,-13,-8,-3,2,7,12,17,22}; //[A]
    double currentpool[2] ={-10,10}; //[A]
    double poolsize = 10;
    double range_x = 10; //[mm] *half range*
    double range_y = 10; //[mm] *half range*
    double range_z = 10; //[mm] *half range*
    double gantry_x = -range_x; //[mm]
    double gantry_y = -range_y; //[mm]
    double gantry_z = -range_z; //[mm]
    double pos_inc = 20; //[mm]
//    double gantry_posmm[3] = {0,0,0};

    // define gantry moving range for calibration
    // calibration cube is 100*160*50mm for grantry's x, y,z ,  which are y,x,z in the coil system frame
    // need to convert gantry's abs position to coordinates in coil system frame when recording
    const double gantryrange_x[2] = {20, 120}; //unit: mm, absolute position for gantry
    const double gantryrange_y[2] = {-80, 80}; //unit: mm, absolute position for gantry
    const double gantryrange_z[2] = {-40, 10}; //unit: mm, absolute position for gantry
//    const double gantryrange_z[2] = {-50, 0}; //unit: mm, absolute position for gantry
    const double gantryinitcorner[3] = {120, 80, 10};//unit: mm, absolute position for gantry at initial corner
    const double gantryendcorner[3] = {20, -80, -40};//unit: mm, absolute position for gantry at end corner
    const double gantryorigin[3] = {70, 0, -15}; //unit: mm, absolute position for gantry at center
//    const double gantryinitcorner[3] = {120, 80, 0};//unit: mm, absolute position for gantry at initial corner
//    const double gantryendcorner[3] = {20, -80, -50};//unit: mm, absolute position for gantry at end corner
//    const double gantryorigin[3] = {70, 0, -25}; //unit: mm, absolute position for gantry at center
    int loopcount = 0;
    int loop = 1;
    bool gantryinitflag = false; //moniter if gantry goes to the initial corner: lower bound of each range
    bool singleloopdone = true;
    bool Datacollectdoneflag = false;
    bool gantryZupwardFlag = false;
    bool gantryZdownwardFlag = true;
    bool gantryYnegtivewardFlag = true;
    bool gantryYpositivewardFlag = false;

    /// GAME CONTROLLER OBJECT
    GamepadMonitor connectedGamepad; // Gamepad monitor for the connected game controller (Use Xbox)

    /// PLOTTING AND CALLBACKS TIMERS
    const int captionRefreshPeriod = 10; // 20 ms correlates to 50 Hz
    const int callbackRefreshPeriod = 10; // 20 ms correlates to 50 Hz

    /// S826 BOARD FOR CONTROL
    S826 s826;


    /// DAQ FOR FEEDBACK
    daq DAQ;
    double gaussmeterCalibratedConstants[3] = {102.67, 103.25, 104.66}; // mT/V
    double gaussCalibCons_new[3] = {102.6250, 102.9547, 104.3281}; // mT/V
    std::ofstream arduinoMotor0;
    std::ofstream arduinoMotor1;
    std::ofstream arduinoMotor2;
    std::ofstream arduinoMotor3;

    /// CONTROL BOOLEANS and STATE VALUES
    bool overheatingFlag = false;
    bool isGradientControlled = true;
    bool controllerState; // State for enabling or disabling the game controller.
    bool useCalibratedFieldValues = true;
    // Enable/Disable Plots State
    bool magneticPlotState; // State for enabling realtime plotting of the magnetic field.
    bool gradientPlotState; // State for enabling realtime plotting of field gradients on the magnetic field plot.
    bool magneticLegendState;
    bool inputPlotState; // State for enabling the realtime plotting of amplifier currents in the EMs.
    bool setpointPlotState; // State for enabling realtime plotting on current setpoints on the input plot.
    bool inputLegendState;
    bool temperaturePlotState;
    bool temperatureLegendState;

    bool isRecordingData = false;
    // Data recording states:
    // 0: Continuous Recording
    // 1: Discrete Sampling
    int dataRecordingState = 0;
    bool DAQPlotState;
    bool DAQLegendState;

    int currentControllerMode; // Specifies the state of the game controller. This dictates how the joystick inputs are used.
    int currentSubroutineMode; // Specifies the state of the Subroutines.
    bool subroutineState;
    bool enableAngleControlState; // REMOVE? Is this control via angle?

    // Actuation System Variables
//    double mAct[4] = {M*vAct, M*vAct, M*vAct, M*vAct}; //actuator magnet dipole moments (scalar) [Am^2] constant

    //    double mAct[4] = {0.0};

    // Change mAct to 8 indices

    double pTool[3] = {0, 0, 0}; // tool position centered [m]

    double angle1 = 0.0; // Theta or Gamma
    double angle2 = 0.0; // Phi or Beta
    double B_mag = 0.0;
    // k factor is ( stiffness k / magnetic moment )
    double k_factor = 0.0; // mT/rad : use 7.4-8.1 from experiments
    double angle3 = 0.0; // N/A or theta

//    const double currentControlFactor = 5.0;    // [Amps/Volt] Assuming that the +-10V input is including the limited peak current
//    const double currentSenseFactor = 8.05;     // [Amps/Volt] signal from current amplifiers
//    const double temperatureSenseFactor = 20.0; // [deg C/Volt] signal from thermocouples after instrumental amplifier gain applied

    int roll_dir = 0;    

    /// CONTROL MATRICES
    // Declare the control matrix for relating the currents to the magnetic field and gradients
    // This matrix considers the gradients as well as the field components
    double N[numAct][numField+numGrad] = { {  0.0036336,	-0.00078,	-0.004188,	-0.0172944,	 0.01758,	-0.0034944,	 0.0017976,	 0.0040032 },
                                           {  0.0037728,	 0.0181776,	 0.003588,	-0.0010728,	 0.0008232,	-0.0040848,	-0.0170712,	-0.0036168 },
                                           { -0.0007968,	 0.0125472,	-0.0012336,	 0.012252,	 0.01212,	-0.0011952,	 0.0121992,	-0.0011472 },
                                           { -0.0153312,	 0.15354,	-0.0191832,	-0.0795432,	-0.0935904,	-0.012216,	 0.1548408,	-0.0230928 },
                                           { -0.0383184,	 0.0036216,	 0.0412104,	-0.0063,	-0.0008544,	-0.0371352,	 0.015732,	 0.0364968 },
                                           { -0.0083544,	 0.0153336,	 0.0099864,	 0.2313216,	-0.2274,	 0.0070104,	-0.011592,	-0.01098   },
                                           { -0.0187776,	-0.0901176,	-0.0144816,	 0.1492368,	 0.1640112,	-0.0203568,	-0.0964416,	-0.0139104 },
                                           { -0.0108264,	-0.247728,	-0.0094248,	 0.0090024,	-0.0205296,	 0.0097824,	 0.2304216,	 0.0087936 } };
    // Declare the inverse of the control matrix relating the magnetic field components to the currents
    double invN[numField+numGrad][numAct] =  {0.0};
    // Declare the control matrix for relating the currents to the magnetic field
    // This matrix does not consider the gradients, only the field components
    // Uses these calibrated field values unless useCalibratedFieldValues is set to false
    double M[numField][numAct] =   { { 0.0036336,	-0.00078,	-0.004188,	-0.0172944,	0.01758,	-0.0034944,	 0.0017976,	 0.0040032},
                                     { 0.0037728,	 0.0181776,	 0.003588,	-0.0010728,	0.0008232,  -0.0040848,	-0.0170712,	-0.0036168},
                                     {-0.0007968,	 0.0125472,	-0.0012336,	 0.012252,	0.01212,	-0.0011952,	 0.0121992,	-0.0011472} };
    // Declare the psuedo-inverse of the non-square control matrix relating the currents to the magnetic field components
    double pseudoinvM[numAct][numField] =  {0.0};

    /// CALIBRATION CONSTANTS
    // These are the calibration constants for each electromagnet current signals
//    const double currentControlAdj[numAct] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; // [T/V]
//    const double currentControlAdj[numAct] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2}; // [V/A] Max 5V for 25 A Output (OLD ESTIMATES)
    const double currentControlAdj[numAct] = {1.0/6.7501, 1.0/6.6705, 1.0/6.4118, 1.0/6.7818, 1.0/6.7703, 1.0/6.7703, 1.0/6.7107, 1.0/6.8500}; // [V/A] Max Command signal for 24 A Output shown above
//    const double currentSenseAdj[numAct] = {8.05, 8.05, 8.05, 8.05, 8.05, 8.05, 8.05, 8.05}; // [A/V] (OLD ESTIMATES)
    const double currentSenseAdj[numAct] = {6.7501, 6.6705, 6.4118, 3.8831, 6.7703, 6.7703, 6.7107, 6.8500}; // [A/V] (OLD ESTIMATES)
//    const double currentSenseAdj[numAct] = {1.0/0.6338, 1.0/0.6516, 1.0/0.6791, 1.0/0.64, 1.0/0.6422, 1.0/0.6395, 1.0/0.6341, 1.0/0.6392}; // [A/V] (NEW ESTIMATES) (NOTE EM7 is not calibrated)
    const double temperatureSenseAdj[numAct] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}; // [deg C/V]
//    const double ATINanoVoltageOffsets[6] = {-1.26987250,	3.78201850,	2.1132725,	2.020594,	-0.4165031,	-0.75069465}; // just magnet experiments
    const double ATINanoVoltageOffsets[6] = {-1.3171104,	3.8164696,	2.2166968,	1.9402872,	-0.44156372,	-1.0281404}; // Gripper experiments

    double ATINanoCalibrationMatrix[6][6] =       {{  0.00749,	 0.02250,	 0.01747,	-0.80775,	-0.02452,	 0.81951},
                                                   { -0.03933,	 0.97698,	 0.01759,	-0.43942,	 0.00040,	-0.49738},
                                                   {  0.92086,	 0.03400,	 0.93714,	-0.00522,	 0.97298,	 0.01115},
                                                   { -0.55209,	 5.99193,	 5.23393,	-2.69998,	-5.44013,	-3.14306},
                                                   { -5.88807,	-0.35924,	 2.76672,	 4.91801,	 3.46558,	-4.95869},
                                                   { -0.17835,	 3.57785,	-0.02007,	 3.30164,	-0.08660,	 3.66835}};

    /// CURRENTS AND FIELD VALUES
    // should there be gradients considered for the local field? if so, how do you transform them to find the global frame equivalent?
    double B_Local_Desired[numField] = {0.0};      // [T?]
    double B_Global_Desired[numField+numGrad] = {0.0};     // [T?]
//    double B_Global_Desired_Input[8] = {0.0};
    double B_Global_Output[numField+numGrad]  = {0.0}; // [T] theoretical field values based on measured currents
    double measuredCurrents[numAct]     = {0.0}; // [A] read from amplifiers
    double measuredTemperatures[numAct] = {0.0}; // [C] thermocouple feedback readings converted to deg C
    double inputAnalogVoltages[16] = {0.0}; // [V] feedback voltage values. Contains both coil current monitor and thermocouples voltages
    double ATINanoForceTorque[6] = {0.0}; //
    // Input Analog Voltages are as follows:
    // AIN0 - AIN7  ARE CURRENT SENSE FROM EM0-EM7, respectively
    // AIN8 - AIN15 ARE THERMOCOUPLE SENSE FROM EM0 - EM7, respectively
    double currentSetpoints[numAct]     = {0.0}; // [A] out to amplifiers
    double outputAnalogVoltages[numAct] = {0.0}; // [V] Voltage values to go out to the current amplifiers
    // Output Analog Voltages are as follows:
    // AOUT0 - AOUT7 ARE CURRENT REFERENCES FOR EM0 - EM7, respectively

    int gantryJogSteps;
    const int gantryStepsPerMM = 538;
    // The Absolute X, Y, and Z positions in # of Steps
    int gantryPos[3] = {0};
    // The Absolute X, Y, and Z positions in Steps when zeroed. Also the inverse of the midpoint values
    const int gantryZeros[3] = {65000,58756,8888};
    double tilt = 0.0; // [rad]?
    double roll = 0.0; // [rad]?
    // Array to store the parameters used in the subroutines section.
    double subThreadParameters[5] = {0.0};

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    /// PUBLIC FUNCTIONS WRITING TO ARDUINOS
//    void ArduinoWrite(int motorNum, int position);
    void arduinoWrite(int arduinoNum, QByteArray outputdata);

    void ArduinoWrite4(double motorAngles[4]);

    /// PUBLIC OPTIMIZATION FUNCTIONS
    void findB_SurgeonSimulator(double B_desired_local[3]);
    bool UDPflag = false;


    //UPD package receiving
//    WSADATA wsaData;
//    SOCKET ConnectSocket = INVALID_SOCKET;
//    SOCKADDR_IN serverSin;
//    int serverSinSize = sizeof(serverSin);
//    SOCKADDR_IN clientSin;
//    int clientSinSize = sizeof(clientSin);

//    char buf[BUFLEN];
//    int recvLen;
//    char address_string[INET_ADDRSTRLEN];
    void udpreceive(void);
    void udpintialize(void);

    struct udp_data{
        double LeftDesiredPosition[3];
        struct Stylus{
            double Gimbal[3];
            bool GreyButton;
            bool WhiteButton;
        }LeftHaptic;
    }m_Outputs;

    bool xysampleflag;
    int  samplecount = 0;



private:
    Ui::MainWindow *ui;
    // The arduino Uno corresponds to the arduino controlling the trocar
    static const quint16 arduinoUnoVendorID = 10755;
    static const quint16 arduinoUnoProductID = 67;
    // The arduino MEGA corresponds to the arduino controlling the gantry
    static const quint16 arduinoMegaVendorID = 9025;
    static const quint16 arduinoMegaProductID = 66;
    //TODO set these to correct values

    bool gantryConnected = false;
    bool trocarConnected = false;

    /// VECTORS FOR RECORDING DATA and PLOTTING
    // Declare vectors for recording the Magnetic Fields and Gradients for plotting:
    QVector<double> qv_t_mag, qv_Bx, qv_By, qv_Bz, qv_dBxdx, qv_dBxdy, qv_dBxdz, qv_dBydy, qv_dBydz;
    // Old vectors for recording the DAQ input voltages
    QVector<double> qv_t_DAQ, qv_DAQai0, qv_DAQai1, qv_DAQai2, qv_DAQai3, qv_DAQai4, qv_DAQai5, qv_DAQai6, qv_DAQai7;
    // Declare vectors for recording the setpoint currents and the measured feedback currents in all 8 EMs:
    QVector<double> qv_t_input, qv_setpoint0, qv_setpoint1, qv_setpoint2, qv_setpoint3, qv_setpoint4, qv_setpoint5, qv_setpoint6, qv_setpoint7;
    QVector<double> qv_current0, qv_current1, qv_current2, qv_current3, qv_current4, qv_current5, qv_current6, qv_current7;
    // Declare vectors for recording the measured feedback temperatures from the thermocouples in all 8 EMs:
    QVector<double> qv_t_thermocouple, qv_temp0, qv_temp1, qv_temp2, qv_temp3, qv_temp4, qv_temp5, qv_temp6, qv_temp7;

    // Set the time for all plots and the periodicity of recording/sampling and plotting
//    double currentTime = 0.0;

    QElapsedTimer currentTime;
    double lastTime;
    double plotPeriod = 4.0; // in seconds. 10 s for slow scrolling, 5 s for faster.

    /// PRIVATE SETUP FUNCTIONS
    void setupPlots(void);
    void addPoints(double x_Axis, double y_Axis[8], int graphIndex);
    void plot(void);
    QUdpSocket *socket = nullptr;

public slots:
    void enableUDP(void);
    void enableController(void);
    void enableGradientControl(void);
    void enableSubroutine(void);
    void enableAngleControl(void);
    void enableDAQ(void);
    // Plots and Subplots Enable/Disable
    void enableMagneticPlot(void);
    void enableGradientPlot(void);
    void enableInputPlot(void);
    void enableSetpointPlot(void);
    void enableTemperaturePlot(void);
    void enableDAQPlot(void);
    // Legends Enable/Disable
    void enableMagneticLegend(void);
    void enableInputLegend(void);
    void enableTemperatureLegend(void);
    void enableDAQLegend(void);
    // remove angles
    void gantryZero(void);
    void gantryCenter(void);
    void gantryJog(void);
    void gantryMoveAbsolute(void);
    void gantryWrite(int mode, int reference, int axis, int dir, int steps); // Relative motion
    void gantryWrite(int mode, int reference, int axis, int steps); // Absolute motion
    void gantryWrite(int mode);
    void gantrySpeed(int speed); //update gantry moving speed
    void gantryMoveMulti(int Posxyz[3]);
    void setGantryJogSteps(void);
    void ToolIN_press_Slot(void);
    void ToolOUT_press_Slot(void);
    void ToolINOUT_rel_Slot(void);
    void ToolINOUT_speed_Slot(int speed);
    void changeRoll(int dir);
    void enablesampleBxy();


private slots:
    void callbacks(void);
    void updateCaption(void);
    void updateCurrents(void);
    void updateCurrents_CalibrationOnly(double I_command[8]);
    void controllerModeSlot(int nModeIndex); // DONE
    void subThreadModeSlot(int nModeIndex);
    void subThreadParameterChangeSlot(void);
    void sliderMovingSlot(int value);
    void setTiltSlot(void);
    void setRollSlot(void);
    void setFactorSlot(void);
    void setFieldSpinBoxSlot(void); //double value);
    void clearCurrentsSlot(void);
    void clearTimeSlot(void);
    void startDAQRecording(void);
    void setDAQRecordingState(void);
    void recordDAQDatapoint(void);
    void stopDAQRecording(void);
    void reconnectGamepadSlot(void);
    void reconnectS826(void);
    void processPendingDatagrams();
    void sampleBxy();

signals:
    void controllerStateSignal(bool state);
};


#endif // MAINWINDOW_H
