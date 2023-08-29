#include "callbacks.h"

//void callback(void)
//{
//    qDebug() << "Callback executed.";

//}

void MainWindow::callbacks(void)
{
    // Execute code here
//    qInfo() << connectedGamepad.joystickValues[0];
//    qDebug() << "Code callback executing...";


    // ~~~~~~~~~~~~~~~~~~~~~~~~ GENERAL CALLBACKS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
    // These are commands that are executed every iteration no matter the settings
    // to ensure that the system is running properly. As such, they should not be
    // modified unless you know what you are doing.

    // Read Thermocouples and Current Monitors
    if (s826.boardConnected)
    {
        // Read all 16 input channels and store in pass-by-reference array:
        int err = s826.analogReadAll(inputAnalogVoltages);
//        qInfo() << err;
    }

    for (int t = 0; t < 8; t++)
    {
        // Record new temperatures and currents from conversions
        // Input Analog Voltages are as follows:
        // AIN0 - AIN7  CURRENT SENSE FROM EM0-EM7, respectively
        // AIN8 - AIN15 THERMOCOUPLE SENSE FROM EM0 - EM7, respectively
        measuredCurrents[t] = inputAnalogVoltages[t]*currentSenseAdj[t]; // [A] read from amplifiers
        measuredTemperatures[t] = inputAnalogVoltages[t+8]*temperatureSenseAdj[t]; // [deg C] read from thermocouples
        // Check that the temperature in any core is not above the max value
        if (measuredTemperatures[t] > maxAllowableTemp)
        {
 //!!!!!!!!!! NEVER change or comment below code!!!!!!!!!!!!!!!!!!!!!!!
 //!!!!!!!!!!! This is the only place to moniter overheating of the system!!!
//            overheatingFlag = true;
            // set all currents to 0 and reset all desired field strengths
//            std::cerr << "System is Overheating!!!"<<std::endl ;
//            updateCurrents();
//            std::cerr << "Currents Cleared"<<std::endl;
            // at the end of callbacks, re-evaluate the temperatures.
        }
    }

    // READ FROM DAQ
    if (DAQ.isEnabled())
    {
        // Read analog inputs from the DAQ by reading values and passing by ref.
        DAQ.dataAcquisition8(DAQ.analogInputVoltages);
        DAQ.dataAcquisition8(DAQ.analogRawInputVoltages); //record the raw data without any change
        std::cout<<std::endl<<"Daq reading is: "<<DAQ.analogRawInputVoltages[0]<<" "<<DAQ.analogRawInputVoltages[1]<<" "<<DAQ.analogRawInputVoltages[2]<<std::endl;
        std::cout<<"Field calculation is: "<<DAQ.analogRawInputVoltages[0]*gaussmeterCalibratedConstants[0]<<" "<<DAQ.analogRawInputVoltages[1]*gaussmeterCalibratedConstants[1]<<" "<<DAQ.analogRawInputVoltages[2]*gaussmeterCalibratedConstants[2]<<std::endl;
//        DAQ.dataAcquisition();
        //Get Forces and torques from values
        double tempVoltages[6];
        double originalDAQVol[6];
        for (int v = 0; v<6; v++)
        {
            tempVoltages[v] = DAQ.analogInputVoltages[v]-ATINanoVoltageOffsets[v];
            originalDAQVol[v] = DAQ.analogInputVoltages[v];
        }

        double A[6][6] = {0.0};
        double J[6] = {0.0};
//        MatrixMultVect6(A, tempVoltages, J);
        MatrixMultVect6(ATINanoCalibrationMatrix, tempVoltages, ATINanoForceTorque);
        //this is for ATI force/torque output
        for (int v = 0; v<6; v++)
        {
            DAQ.analogInputVoltages[v] = ATINanoForceTorque[v];
        }
        //this is for gaussmeter output
//        for (int v = 0; v<3; v++)
//        {
//            DAQ.analogInputVoltages[v] = originalDAQVol[v]*gaussmeterCalibratedConstants[v];
//        }
    }

    // RECORD DATA STREAM
    if ( isRecordingData & (dataRecordingState == 0 || dataRecordingState == 2) )
    {
        recordDAQDatapoint();
    }
//    if ( isRecordingData & (dataRecordingState == 0) ) // recording and set to continuous recording
//    {
//        // TODO: Add the points from this iteration into the data file.
//        // TODO: Check if DAQ.isEnabled to write those points as well
//        QFile file(filename);
//        if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
//            QTextStream stream(&file);
////            stream << "Time" << "Bx" << "By" << "Bz" << endl;
//            stream << currentTime.elapsed()/1000.0 << "\t" << B_Global_Output[0] << "\t" << B_Global_Output[1] << "\t" << B_Global_Output[2] << "\t" << "\n";
//            // TODO add more values to save here
//            // TODO add daq check and save daq values as well (at the ends)
//            file.close();

//        }
//        else
//        {
//            qWarning() << "Problem occured while writing to the datafile.";
//        }
//    }



    // TODO:
        // CHECK TEMPERATURES ARE < MAX VAL
    // Cut power if thermocouples are above a threshold!!


    // this was an attempt at making a shortcut to open the windows gaming bar with a ps3 controller
//    if (connectedGamepad.openGameBar)
//    {
//        QKeyEvent *event_Meta = new QKeyEvent( QEvent::KeyPress, Qt::Key_Meta, Qt::NoModifier);
//        QCoreApplication::postEvent(this, event_Meta);
//        QKeyEvent *event_G = new QKeyEvent( QEvent::KeyPress, Qt::Key_G, Qt::NoModifier);
//        QCoreApplication::postEvent(this, event_G);
//        QKeyEvent *event_Enter = new QKeyEvent( QEvent::KeyPress, Qt::Key_Enter, Qt::NoModifier);
//        QCoreApplication::postEvent(this, event_Enter);
//        connectedGamepad.openGameBar = false;
//    }


    // TODO:
        // CAMERA UPDATES IN THIS PART




    // NEED A WAY TO DISTINGUISH BETWEEN CONTROLLER CONTROL AND SUBROUTINE CONTROL




    // ~~~~~~~~~~~~~~~~~~~~~ CONTROLLER INPUT MODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
    // The following is a list of all the control modes and their state values.
    // More info on each mode can be found before in the state check of each below.
    // 0. <BLANK>
    // 1. DIRECT LOCAL B-FIELD CONTROL
    // 2. CHANGE IN LOCAL B-FIELD CONTROL
    // 3. CHANGE IN POLAR ANGLE CONTROL
    // 4. SURGEON SIMULATOR
    // 5. SUBROUTINE
    // 6. ROLLING

    if (currentControllerMode == 0 && connectedGamepad.enabled) // Update <Nothing>
    {
        /// 0. <BLANK>
        //
        // This function does nothing. It serves as a place where the the controller
        // can be enabled but does not control anything, for safety.


        // Code here should be deleted and is only intended for testing purposes:
//        B_Global_Desired[0] = connectedGamepad.joystickValues[0]*connectedGamepad.joystickValues[0]*connectedGamepad.joystickValues[0]*maxB/4;
//        B_Global_Desired[1] = connectedGamepad.joystickValues[2]*connectedGamepad.joystickValues[2]*connectedGamepad.joystickValues[2]*maxB/4;
//        B_Global_Desired[2] = connectedGamepad.joystickValues[4]*maxB/4;
//        B_Global_Desired[3] = connectedGamepad.joystickValues[1]*maxB/2;
//        B_Global_Desired[4] = connectedGamepad.joystickValues[3]*maxB/2;
//        B_Global_Desired[5] = connectedGamepad.joystickValues[5]*maxB/2;

        updateCurrents();

        // Go from Desired Global to Actual Global:


    }
    else if (currentControllerMode == 1 && connectedGamepad.enabled) // Update Direct Local B-field
    {
        //// 1. DIRECT LOCAL B-FIELD CONTROL:
        //
        // Here the joystick directly controls the desired Bx, By, and Bz field components
        // in the tool frame. This assumes that the desired fields are relative to the
        // tool which, when at rest, points in the z-direction. The code takes the
        // desired local frame magnetic field values and computes the necessary field in
        // the global frame to be sent to the actuators.
        //
        // The assigned joystick axes are as follows:
        // Left Y-axis:     Bx field
        // Right X-axis:    By field
        // L2 and R2:       Bz field

//        qDebug() << "Working";

        // Set desired B local field
        B_Local_Desired[0] = -maxB/2.0*connectedGamepad.joystickValues[1] ; // Increase nonlinearly to add sensitivity and control
        B_Local_Desired[1] =  maxB/2.0*connectedGamepad.joystickValues[2] ; // Increase nonlinearly to add sensitivity and control
        B_Local_Desired[2] = -maxB/2.0*connectedGamepad.joystickValues[4] + maxB/2.0*connectedGamepad.joystickValues[5]; // Increase nonlinearly to add sensitivity and control
        // Calculate the equivalent desired global field
        local2global(tilt, roll, B_Local_Desired, B_Global_Desired);

        // Calling updateCurrents will:
        // 1. Check for overheating
        // 2. Determine the currents needed to generate the desired field
        // 3. Calculate the theoretical magnetic field produced by said currents
        // 4. Send analog output command to change the coil currents if the system is not overheating
        updateCurrents();

    }
    else if (currentControllerMode == 2 && connectedGamepad.enabled) // Update Change in Local B-field
    {
        //// 2. CHANGE IN LOCAL B-FIELD CONTROL:
        //
        // Here the joystick controls the amount added to the desired Bx, By, and Bz field
        // components in the tool frame over time. This can be thought of as 'velocity'
        // control. This assumes that the desired fields are relative to the tool
        // which, when at rest, points in the z-direction. The code takes the desired
        // local frame magnetic field values and computes the necessary field in the
        // global frame to be sent to the actuators.
        //
        // The assigned joystick axes are as follows:
        // Left Y-axis:     dBx/dt field
        // Right X-axis:    dBy/dt field
        // L2 and R2:       dBz/dt field

        // Set desired Bz local field
        if (abs(connectedGamepad.joystickValues[1]) > 0.075) // threshold because of the noisy controller in x
        {
            B_Local_Desired[0] = B_Local_Desired[0] + -maxB*connectedGamepad.joystickValues[1]/100 ; // Increase nonlinearly to add sensitivity and control
        }
        B_Local_Desired[1] = B_Local_Desired[1] + maxB*connectedGamepad.joystickValues[2]/100 ; // Increase nonlinearly to add sensitivity and control
        B_Local_Desired[2] = B_Local_Desired[2] - maxB*connectedGamepad.joystickValues[4]/100 + maxB*connectedGamepad.joystickValues[5]/100; // Increase nonlinearly to add sensitivity and control

        // Threshold the max Bfield so it doesn't increase to an absurd number
        for (int i=0;i<3;i++)
        {
            if (abs(B_Local_Desired[i]) > maxB)
            {
                B_Local_Desired[i] = maxB * B_Local_Desired[i]/abs(B_Local_Desired[i]); // max field times direction of joystick.
            }
        }
        // Set desired global field if there has been a change
        if ( (abs(connectedGamepad.joystickValues[1]) > 0.075) || (abs(connectedGamepad.joystickValues[2]) > 0.075) || (abs(connectedGamepad.joystickValues[4]) > 0.075) || (abs(connectedGamepad.joystickValues[5]) > 0.075)  )
        {
            // Calculate the equivalent desired global field
            local2global(tilt, roll, B_Local_Desired, B_Global_Desired);
            // Calling updateCurrents will:
            // 1. Check for overheating
            // 2. Determine the currents needed to generate the desired field
            // 3. Calculate the theoretical magnetic field produced by said currents
            // 4. Send analog output command to change the coil currents if the system is not overheating
            updateCurrents();
        }
    }
    else if (currentControllerMode == 3 && connectedGamepad.enabled) // Update Change in Polar Direction
    {
        //// 3. CHANGE IN POLAR DIRECTION CONTROL:
        //
        // Here the joystick controls the Bx, By, and Bz field by changing its polar counterparts.
        // Moving the joystick increases or decreases the polar angles that desribe the field
        // in polar (spherical?) coordinates in the tool frame. This assumes that the desired fields
        // are relative to the tool which, when at rest, points in the z-direction. The joystick
        // also has control over the magnitude of the field. The code takes the desired local frame
        // magnetic field values and computes the necessary field in the global frame to be sent to
        // the actuators.
        //
        // The assigned joystick axes are as follows:
        // Left Y-axis:     Angle of Phi from the z-axis about the y-axis, Field Unit Vector
        // Right X-axis:    Angle of Theta about the z-axis from the (new?) x-axis, Field Unit Vector.
        // L2 and R2:       Magnitude of Magnetic Field Vector

        if (abs(connectedGamepad.joystickValues[1]) > 0.075) // threshold because of the noisy controller in x
        {
            angle2   = angle2 + connectedGamepad.joystickValues[1] / 25; // phi
        }
        angle1 = angle1 + connectedGamepad.joystickValues[2] / 25; // theta
        B_mag = B_mag - connectedGamepad.joystickValues[4] / 5000 + connectedGamepad.joystickValues[5] / 5000;
        // Threshold the max Bfield so it doesn't increase to an absurd number
        if ( abs(B_mag) > maxB)
        {
            B_mag = maxB * B_mag/abs(B_mag); // x/abs(x) = sign(x);
        }
        B_Local_Desired[0] = B_mag*sin(angle2)*cos(angle1); // use phi and theta, angles 1 and 2
        B_Local_Desired[1] = B_mag*sin(angle2)*sin(angle1); // use phi and theta, angles 1 and 2
        B_Local_Desired[2] = B_mag*cos(angle2); //

        // Calculate the equivalent desired global field
        local2global(tilt, roll, B_Local_Desired, B_Global_Desired);
        // Calling updateCurrents will:
        // 1. Check for overheating
        // 2. Determine the currents needed to generate the desired field
        // 3. Calculate the theoretical magnetic field produced by said currents
        // 4. Send analog output command to change the coil currents if the system is not overheating
        updateCurrents();
    }
    else if (currentControllerMode == 4 && connectedGamepad.enabled) // Surgeon Simulator
    {
        //// 4. SURGEON SIMULATOR:
        //
        // Here the joystick controls the Bx, By, and Bz field in the tool frame by using an
        // elastic model for the gripper bending and 'velocity' control of the pitch and yaw
        // angles. This mode also has improved control of the speed of the in and out tool
        // motions as well as roll values. The joystick controls increase or decrease the
        // desired pitch and yaw angles of the gripper and use a bending model to determine
        // the perpendicular and parallel field components that describe the field in the
        // tool frame. This assumes that the desired fields are relative to the tool
        // which, when at rest, points in the z-direction. The code takes the computed
        // fields in local frame and computes the necessary fields in the global frame
        // to be sent to the actuators.
        //
        // The assigned joystick axes are as follows:
        // Left Y-axis:     Speed of the Tool in and out motions
        // Right X-axis:    Yaw of the tool in degrees
        // Right Y-axis:    Pitch of the tool in degrees
        // L2 and R2:       Magnitude of the parallel Magnetic Field Vector for grasping.
        //
        // Optionally, the joystick can control the field directly instead of pitch and yaw:
        // Left Y-axis:     Speed of the Tool in and out motions (unchanged)
        // Right X-axis:    dBx/dt field
        // Right Y-axis:    dBy/dt field
        // L2 and R2:       dBz/dt field
        // NOTE: that the bending model is not used in this second case.


        // Set desired In/Out Speed of tool
        if (abs(connectedGamepad.joystickValues[1]) > 0.075) // threshold because of the noisy controller in x
        {
            ToolINOUT_speed_Slot(int(-100*sqrt(abs(connectedGamepad.joystickValues[1])) * (connectedGamepad.joystickValues[1]/abs(connectedGamepad.joystickValues[1]))) + 110); // Value -1 to 1 shifted to 10 to 210
        }
        else
        {
            ToolINOUT_rel_Slot(); // sends a stop command
            // This doesn't allow the a faster in/out motion with the buttons
        }
        // Check if Field control or Angle control is Enabled
        if (enableAngleControlState)
        {
            angle1   = angle1 - connectedGamepad.joystickValues[2] / 60; // change - to + to invert direction  of joystick
            angle2   = angle2 + connectedGamepad.joystickValues[3] / 60;
            B_mag = B_mag - maxB*connectedGamepad.joystickValues[4]/100 + maxB*connectedGamepad.joystickValues[5]/100; // Increase nonlinearly to add sensitivity and control
            // Check bounds
            if (abs(angle1) > M_PI/2)
            {
                angle1 = M_PI/2 * angle1/abs(angle1); // (keeps the polarity of the angle)
            }
            if (abs(angle2) > M_PI/2)
            {
                angle2 = M_PI/2 * angle2/abs(angle2); // (keeps the polarity of the angle)
            }
            // Calculate angle Theta from other angles, Gamma and Phi
            angle3 = calculateTheta(angle1, angle2);
            findB_SurgeonSimulator(B_Local_Desired);
        }
        else
        {
            angle1 = 0.0;
            angle2 = 0.0;
            B_mag = 0.0;
            B_Local_Desired[0] = B_Local_Desired[0] - maxB*connectedGamepad.joystickValues[2]/100 ; // Increase nonlinearly to add sensitivity and control
            B_Local_Desired[1] = B_Local_Desired[1] + maxB*connectedGamepad.joystickValues[3]/100 ; // Increase nonlinearly to add sensitivity and control
            B_Local_Desired[2] = B_Local_Desired[2] - maxB*connectedGamepad.joystickValues[4]/100 + maxB*connectedGamepad.joystickValues[5]/100; // Increase nonlinearly to add sensitivity and control
        }

        // Threshold the max Bfield so it doesn't increase to an absurd number
        for (int i=0;i<3;i++)
        {
            if (abs(B_Local_Desired[i]) > maxB)
            {
                B_Local_Desired[i] = maxB * B_Local_Desired[i]/abs(B_Local_Desired[i]);
            }
        }


        // Calculate the equivalent desired global field
        local2global(tilt, roll, B_Local_Desired, B_Global_Desired);
        // Calling updateCurrents will:
        // 1. Check for overheating
        // 2. Determine the currents needed to generate the desired field
        // 3. Calculate the theoretical magnetic field produced by said currents
        // 4. Send analog output command to change the coil currents if the system is not overheating
        updateCurrents();
    }
    else if (currentControllerMode == 5 && connectedGamepad.enabled)
    {
        //// 5. SUBROUTINES:
        // For if the gamepad needs to control something in the subroutines slot
    }


    else if (currentControllerMode == 6 && connectedGamepad.enabled) // Jason's part to controll untethered robots for rolling motion
    {
        //// 6. ROLLING:
        // Left X-axis:    Bx field, default 1 mT
        // Left Y-axis:    By field, default 1 mT
        // L2:             Magnitude
        // Right X-axis:   Rolling along x axis
        // Right Y-axis:   Rolling along y axis
        // R2:             Rolling frequency
        double Bx = connectedGamepad.joystickValues[0];
        double By = -connectedGamepad.joystickValues[1];
        double Mag = connectedGamepad.joystickValues[4];
        double XRoll = connectedGamepad.joystickValues[2];
        double YRoll = -connectedGamepad.joystickValues[3];
        double freq = connectedGamepad.joystickValues[5];
        double t = currentTime.elapsed()/1000.0; // time in seconds
        B_Global_Desired[0] =  (maxB*Mag/4.0+0.001)*Bx ; //field range: 0 ~ 22.29 mT
        B_Global_Desired[1] =  (maxB*Mag/4.0+0.001)*By ; //field range: 0 ~ 22.29 mT
        B_Global_Desired[2] = 0;
//        qDebug() << XRoll;
//        qDebug() << YRoll;
        double inplaneMag = sqrt(pow(XRoll,2)+pow(YRoll,2));// in-plane component of field magnitude
        if (inplaneMag > 0.2) //radius greater than value of 0.2
        {
            B_Global_Desired[0] = (maxB*Mag/4.0+0.001)*sin(2*M_PI*(1+round(2*freq))*t)*XRoll/inplaneMag;
            B_Global_Desired[1] = (maxB*Mag/4.0+0.001)*sin(2*M_PI*(1+round(2*freq))*t)*YRoll/inplaneMag;
            B_Global_Desired[2] = (maxB*Mag/4.0+0.001)*cos(2*M_PI*(1+round(2*freq))*t);
        }

        // Calculate the equivalent desired local field
        global2local(tilt, roll, B_Global_Desired, B_Local_Desired);

        // Calling updateCurrents will:
        // 1. Check for overheating
        // 2. Determine the currents needed to generate the desired field
        // 3. Calculate the theoretical magnetic field produced by said currents
        // 4. Send analog output command to change the coil currents if the system is not overheating
        updateCurrents();
    }



    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~ SUBROUTINES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
    // The following is a list of all the subroutines and their state values.
    // More info on each mode can be found before in the state check of each below.
    // 0. <BLANK>
    // 1. ROTATING FIELD IN THE XY PLANE
    // 2. ROTATING FIELD IN THE XZ PLANE
    // 3. ROTATING FIELD IN THE YZ PLANE
    // 4. OSCILLATING SAWTOOTH WAVEFORM
    // 5. OSCILLATING TRIANGLE WAVEFORM
    // 6. OSCILLATING SQUARE WAVEFORM
    // 7. OSCILLATING SINUSOIDAL WAVEFORM
    // 8. GLOBAL FIELD IN SPHERICAL COORDINATES
    // 9. Collect data for calibration with Neural newtork
    // 10.Collect data for calibration with Model

    if (currentControllerMode == 0 || currentControllerMode == 5)
    {
        // Check that the gamepad controller mode will not interfere with subroutines first.
        if (currentSubroutineMode == 0)
        {
            /// 0. <BLANK>
            //
            // This function does nothing. It serves as a place where the the subthread
            // can be enabled but does not control anything, for safety.
        }
        else if (currentSubroutineMode == 1 && subroutineState)
        {
            /// 1. ROTATING FIELD IN THE XY PLANE
            //
            // This function creates a rotating field in the global XY plane
            // The parameters allow for control over the amplitude, phase, and
            // frequence of the rotating field.
            // PARAMETERS:
            // Magnitude
            // Frequency
            // Phase

            double t = currentTime.elapsed()/1000.0; // time in seconds

            if (subThreadParameters[1] >= 0) //CW if frequency is positive, CCW if frequency is negative
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }

                    /*
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/

                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*cos(theta);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);

            }
            else
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/
                B_Global_Desired[0] = -subThreadParameters[0]/1000.0*cos(theta);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);

            }
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }
        else if (currentSubroutineMode == 2 && subroutineState)
        {
            /// 2. ROTATING FIELD IN THE XZ PLANE
            //
            // This function creates a rotating field in the global XZ plane
            // The parameters allow for control over the amplitude, phase, and
            // frequence of the rotating field.
            // PARAMETERS:
            // Magnitude
            // Frequency
            // Phase

            double t = currentTime.elapsed()/1000.0; // time in seconds
            if (subThreadParameters[1] >= 0) //rolling towards right if frequency is positive, rolling towards left if frequency is negative
            {
                B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
            }
            else
            {
                B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
            }
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }
        else if (currentSubroutineMode == 3 && subroutineState)
        {
            /// 3. ROTATING FIELD IN THE YZ PLANE
            //
            // This function creates a rotating field in the global YZ plane
            // The parameters allow for control over the amplitude, phase, and
            // frequence of the rotating field.
            // PARAMETERS:
            // Magnitude
            // Frequency
            // Phase

            double t = currentTime.elapsed()/1000.0; // time in seconds
            if (subThreadParameters[1] >= 0) //rolling upwards right if frequency is positive, rolling downwards left if frequency is negative
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);
                B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(theta);


            }
            else
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);
                B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(theta);

            }
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }
        else if (currentSubroutineMode == 4 && subroutineState)
        {
            /// 4. OSCILLATING SAWTOOTH WAVEFORM
            //
            // This function creates an oscillating sawtooth waveform in the
            // direction of a vector described in polar coordinates from the x-axis.
            // The parameters allow for control over the lower and upper bounds,
            // frequency of the oscillation, and the direction of the vector using polar coords.
            // PARAMETERS:
            // Lower Bound [mT]
            // Upper Bound [mT]
            // Frequency [Hz]
            // Azimuthal Angle [deg]
            // Polar Angle [deg]

            double t = currentTime.elapsed()/1000.0; // time in seconds

            double slope = (subThreadParameters[1]/1000-subThreadParameters[0]/1000)*subThreadParameters[2]; // this is slope = dist/period = dist*freq
            // Make triangular waveform using absolute value and shift vertically as needed
            double magnitude = subThreadParameters[0]/1000 + fmod((slope*t),(subThreadParameters[1]/1000-subThreadParameters[0]/1000));
            // Set the magnitudes for each field component based on the polar vector
            B_Global_Desired[0] = magnitude * cos(subThreadParameters[4]*deg2rad) * cos(subThreadParameters[3]*deg2rad);
            B_Global_Desired[1] = magnitude * cos(subThreadParameters[4]*deg2rad) * sin(subThreadParameters[3]*deg2rad);
            B_Global_Desired[2] = magnitude * sin(subThreadParameters[4]*deg2rad);
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }
        else if (currentSubroutineMode == 5 && subroutineState)
        {
            /// 5. OSCILLATING TRIANGLE WAVEFORM
            //
            // This function creates an oscillating triangular waveform in the
            // direction of a vector described in polar coordinates from the x-axis.
            // The parameters allow for control over the lower and upper bounds,
            // frequency of the oscillation, and the direction of the vector using polar coords.
            // PARAMETERS:
            // Lower Bound [mT]
            // Upper Bound [mT]
            // Frequency [Hz]
            // Azimuthal Angle [deg]
            // Polar Angle [deg]

            double t = currentTime.elapsed()/1000.0; // time in seconds
            // Assumes triangular slope is symmetric
            double slope = (subThreadParameters[1]/1000-subThreadParameters[0]/1000)*subThreadParameters[2]*2; // this is slope = dist/(period/2) = dist*freq*2
            double magnitude;
            if (subThreadParameters[1] > subThreadParameters[0])
            {
                // if upper > lower
                magnitude = subThreadParameters[1]/1000 - abs(slope*(fmod(t,1/subThreadParameters[2])-0.5/subThreadParameters[2]));
            }
            else
            {
                // if lower > upper
                magnitude = subThreadParameters[1]/1000 + abs(slope*(fmod(t,1/subThreadParameters[2])-0.5/subThreadParameters[2]));
            }
            // Set the magnitudes for each field component based on the polar vector
            B_Global_Desired[0] = magnitude * cos(subThreadParameters[4]*deg2rad) * cos(subThreadParameters[3]*deg2rad);
            B_Global_Desired[1] = magnitude * cos(subThreadParameters[4]*deg2rad) * sin(subThreadParameters[3]*deg2rad);
            B_Global_Desired[2] = magnitude * sin(subThreadParameters[4]*deg2rad);
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }
        else if (currentSubroutineMode == 6 && subroutineState)
        {
            /// 6. OSCILLATING SQUARE WAVEFORM
            //
            // This function creates an oscillating square waveform in the
            // direction of a vector described in polar coordinates from the x-axis.
            // The parameters allow for control over the lower and upper bounds,
            // frequency of the oscillation, and the direction of the vector using polar coords.
            // PARAMETERS:
            // Lower Bound [mT]
            // Upper Bound [mT]
            // Frequency [Hz]
            // Azimuthal Angle [deg]
            // Polar Angle [deg]

            double t = currentTime.elapsed()/1000.0; // time in seconds
            double magnitude;
            // set magnitude to lower bound if time < 1/2 period, else set to upper bound
            if (fmod(t,1/subThreadParameters[2]) < 0.5/subThreadParameters[2] )
            {
                magnitude = subThreadParameters[0]/1000;
            }
            else
            {
                magnitude = subThreadParameters[1]/1000;
            }

            // Set the magnitudes for each field component based on the polar vector
            B_Global_Desired[0] = magnitude * cos(subThreadParameters[4]*deg2rad) * cos(subThreadParameters[3]*deg2rad);
            B_Global_Desired[1] = magnitude * cos(subThreadParameters[4]*deg2rad) * sin(subThreadParameters[3]*deg2rad);
            B_Global_Desired[2] = magnitude * sin(subThreadParameters[4]*deg2rad);
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }
        else if (currentSubroutineMode == 7 && subroutineState)
        {
            /// 7. OSCILLATING SINUSOIDAL WAVEFORM
            //
            // This function creates an oscillating sinusoidal waveform in the
            // direction of a vector described in polar coordinates from the x-axis.
            // The parameters allow for control over the lower and upper bounds,
            // frequency of the oscillation, and the direction of the vector using polar coords.
            // PARAMETERS:
            // Lower Bound [mT]
            // Upper Bound [mT]
            // Frequency [Hz]
            // Azimuthal Angle [deg]
            // Polar Angle [deg]

            double t = currentTime.elapsed()/1000.0; // time in seconds

            double amplitude = (subThreadParameters[1]/1000-subThreadParameters[0]/1000)/2;
            double mean = (subThreadParameters[1]/1000+subThreadParameters[0]/1000)/2;
            // Make sinusoidal waveform using amplitude and shift vertically as needed
            double magnitude = amplitude*sin(2*M_PI*subThreadParameters[2]*t) + mean;
            // Set the magnitudes for each field component based on the polar vector
            B_Global_Desired[0] = magnitude * cos(subThreadParameters[4]*deg2rad) * cos(subThreadParameters[3]*deg2rad);
            B_Global_Desired[1] = magnitude * cos(subThreadParameters[4]*deg2rad) * sin(subThreadParameters[3]*deg2rad);
            B_Global_Desired[2] = magnitude * sin(subThreadParameters[4]*deg2rad);
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }
        else if (currentSubroutineMode == 8 && subroutineState)
        {
            /// 8. GLOBAL FIELD IN SPHERICAL COORDINATES
            //
            // This function creates an oscillating sinusoidal waveform in the
            // direction of a vector described in polar coordinates from the x-axis.
            // The parameters allow for control over the lower and upper bounds,
            // frequency of the oscillation, and the direction of the vector using polar coords.
            // PARAMETERS:
            // Field Magnitude [mT]
            // Polar Angle [deg]
            // Azimuthal Angle [deg]

            double B_temp[3] = {subThreadParameters[0]/1000.0, 0.0, 0.0};
            double B_sph[3];
            calcFieldInSphericalCoords(subThreadParameters[1]*deg2rad, subThreadParameters[2]*deg2rad, B_temp, B_sph);
            // Set the magnitudes for each field component based on the polar vector
            bool writeNewCurrentCommands = false;
            for (int i = 0; i < 3; i++)
            {
                // Check if any setpoints change by 0.1 mT, if so write to S826
                // this is prevent the S826 from running new values every iteration.
                if ( abs(B_Global_Desired[i]-B_sph[i]) > 0.0001 )
                {
                    writeNewCurrentCommands = true;
                }
                B_Global_Desired[i] = B_sph[i];
            }
            if (writeNewCurrentCommands)
            {
                global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
                updateCurrents();
            }
        }
        else if (currentSubroutineMode == 9 && subroutineState)
        {
            /// 9. DATA COLLECTION FOR FIELD CALIBRATION WITH NEURAL NETWORK
            //
            // This function collects/records field [Bx,By,Bz] (fetched from gaussmeter/DAQ),
            // gaussmeter probe position [Px,Py,Pz] (fetched from gantry)
            // and coil currents [I1, I2,...,I8] (measured from S826).
            // The function also sends commands to S826 with desired currents and gantry positions
            // The currents for 8 coils are generated randomly, for each set of currents, gantry moves a full workspace
            // ---
            // --
            // -
            double I_command[8] = {0,0,0,0,0,0,0,0};
            int step_x = 20;
            int step_y = 20;
            int step_z = 10;
//            int step_x = 25;
//            int step_y = 25;
//            int step_z = 15;

            if (gantryinitflag == false)
            {
                int speed = 4000;
                gantrySpeed(speed);
                gantry_x = gantryinitcorner[0];
                gantry_y = gantryinitcorner[1];
                gantry_z = gantryinitcorner[2];
                // move gantry to desired position
                gantryPos[0] = (int) (gantry_x * gantryStepsPerMM);
                gantryPos[1] = (int) (gantry_y * gantryStepsPerMM);
                gantryPos[2] = (int) (gantry_z * gantryStepsPerMM);
                // update steps label box
                ui->spinBox_gantryAbsSteps_X->setValue(gantryPos[0]);
                ui->spinBox_gantryAbsSteps_Y->setValue(gantryPos[1]);
                ui->spinBox_gantryAbsSteps_Z->setValue(gantryPos[2]);
                // update mm label box
                ui->doubleSpinBox_gantryAbsMM_X->setValue( gantry_x );
                ui->doubleSpinBox_gantryAbsMM_Y->setValue( gantry_y );
                ui->doubleSpinBox_gantryAbsMM_Z->setValue( gantry_z );
                // Send command to the Gantry
//                gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_X, gantryPos[0]-gantryZeros[0]);
//                gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Y, gantryPos[1]-gantryZeros[1]);
//                gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Z, gantryPos[2]-gantryZeros[2]);
                int posxyz[3] = {gantryPos[0]-gantryZeros[0], gantryPos[1]-gantryZeros[1], gantryPos[2]-gantryZeros[2]};
                gantryMoveMulti(posxyz);
                Sleep(8000); //wait 10 seconds to make sure gantry has enough time to move to the initial position
                qInfo() << "GantryP is: "<<gantry_x<<gantry_y<<gantry_z;
                qInfo() << "Gantry is initilized!!!";
                gantryinitflag = true;
            }
            else //gantry is initilized
            {
                if (loopcount<=loop)
                {
                    if (singleloopdone == false)
                    {
                        if (gantry_x >= gantryendcorner[0])
                        {
                            if (gantry_y >= gantryrange_y[0]&& gantry_y <= gantryrange_y[1])
                            {
                                if (gantry_z >= gantryrange_z[0]&& gantry_z <= gantryrange_z[1])
                                {
                                    // move gantry to desired position
                                    gantryPos[0] = (int) (gantry_x * gantryStepsPerMM);
                                    gantryPos[1] = (int) (gantry_y * gantryStepsPerMM);
                                    gantryPos[2] = (int) (gantry_z * gantryStepsPerMM);
                                    // update steps label box
                                    ui->spinBox_gantryAbsSteps_X->setValue(gantryPos[0]);
                                    ui->spinBox_gantryAbsSteps_Y->setValue(gantryPos[1]);
                                    ui->spinBox_gantryAbsSteps_Z->setValue(gantryPos[2]);
                                    // update mm label box
                                    ui->doubleSpinBox_gantryAbsMM_X->setValue( gantry_x );
                                    ui->doubleSpinBox_gantryAbsMM_Y->setValue( gantry_y );
                                    ui->doubleSpinBox_gantryAbsMM_Z->setValue( gantry_z );
                                    // Send command to the Gantry
//                                    gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_X, gantryPos[0]-gantryZeros[0]);
//                                    gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Y, gantryPos[1]-gantryZeros[1]);
//                                    gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Z, gantryPos[2]-gantryZeros[2]);
                                    int posxyz[3] = {gantryPos[0]-gantryZeros[0], gantryPos[1]-gantryZeros[1], gantryPos[2]-gantryZeros[2]};
//                                    int posxyz[3] = {gantryPos[0], gantryPos[1], gantryPos[2]};
//                                    qDebug()<<posxyz;
                                    gantryMoveMulti(posxyz);


                                    Sleep(6000);
                                    qInfo() << "GantryP is: "<<gantry_x<<gantry_y<<gantry_z;
                                    if (gantryZupwardFlag == true)
                                    {
                                        gantry_z = gantry_z + step_z;
                                    }
                                    if (gantryZdownwardFlag == true)
                                    {
                                        gantry_z = gantry_z - step_z;
                                    }
                                }
                                else
                                {
                                    if (gantryYpositivewardFlag == true)
                                    {
                                        gantry_y = gantry_y + step_y;
                                    }
                                    if (gantryYnegtivewardFlag == true)
                                    {
                                        gantry_y = gantry_y - step_y;
                                    }

                                    if (gantry_z < gantryrange_z[0])
                                    {
                                        gantry_z = gantryrange_z[0];
                                        gantryZupwardFlag = true;
                                        gantryZdownwardFlag = false;
                                    }
                                    if (gantry_z > gantryrange_z[1])
                                    {
                                        gantry_z = gantryrange_z[1];
                                        gantryZupwardFlag = false;
                                        gantryZdownwardFlag = true;
                                    }

                                    if(gantry_y >= gantryrange_y[0]&& gantry_y <= gantryrange_y[1])
                                    {
                                        // move gantry to desired position
                                        gantryPos[0] = (int) (gantry_x * gantryStepsPerMM);
                                        gantryPos[1] = (int) (gantry_y * gantryStepsPerMM);
                                        gantryPos[2] = (int) (gantry_z * gantryStepsPerMM);
                                        // update steps label box
                                        ui->spinBox_gantryAbsSteps_X->setValue(gantryPos[0]);
                                        ui->spinBox_gantryAbsSteps_Y->setValue(gantryPos[1]);
                                        ui->spinBox_gantryAbsSteps_Z->setValue(gantryPos[2]);
                                        // update mm label box
                                        ui->doubleSpinBox_gantryAbsMM_X->setValue( gantry_x );
                                        ui->doubleSpinBox_gantryAbsMM_Y->setValue( gantry_y );
                                        ui->doubleSpinBox_gantryAbsMM_Z->setValue( gantry_z );
                                        // Send command to the Gantry
//                                        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_X, gantryPos[0]-gantryZeros[0]);
//                                        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Y, gantryPos[1]-gantryZeros[1]);
//                                        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Z, gantryPos[2]-gantryZeros[2]);
                                        int posxyz[3] = {gantryPos[0]-gantryZeros[0], gantryPos[1]-gantryZeros[1], gantryPos[2]-gantryZeros[2]};
//                                        int posxyz[3] = {gantryPos[0], gantryPos[1], gantryPos[2]};
                                        gantryMoveMulti(posxyz);
                                        Sleep(6000);
                                        qInfo() << "GantryP is: "<<gantry_x<<gantry_y<<gantry_z;
//                                        gantry_z = gantry_z - step_z;
                                        if (gantryZupwardFlag == true)
                                        {
                                            gantry_z = gantry_z + step_z;
                                        }
                                        if (gantryZdownwardFlag == true)
                                        {
                                            gantry_z = gantry_z - step_z;
                                        }
                                    }
                                }
                            }
                            else
                            {
                                gantry_x = gantry_x - step_x;
//                                gantry_y = gantryinitcorner[1];
//                                gantry_z = gantryinitcorner[2];

                                if (gantry_z < gantryrange_z[0])
                                {
                                    gantry_z = gantryrange_z[0];
                                    gantryZupwardFlag = true;
                                    gantryZdownwardFlag = false;
                                }
                                if (gantry_z > gantryrange_z[1])
                                {
                                    gantry_z = gantryrange_z[1];
                                    gantryZupwardFlag = false;
                                    gantryZdownwardFlag = true;
                                }

                                if (gantry_y < gantryrange_y[0])
                                {
                                    gantry_y = gantryrange_y[0];
                                    gantryYpositivewardFlag = true;
                                    gantryYnegtivewardFlag = false;
                                }
                                if (gantry_y > gantryrange_y[1])
                                {
                                    gantry_y = gantryrange_y[1];
                                    gantryYpositivewardFlag = false;
                                    gantryYnegtivewardFlag = true;
                                }
                                if(gantry_x >= gantryendcorner[0])
                                {
                                    // move gantry to desired position
                                    gantryPos[0] = (int) (gantry_x * gantryStepsPerMM);
                                    gantryPos[1] = (int) (gantry_y * gantryStepsPerMM);
                                    gantryPos[2] = (int) (gantry_z * gantryStepsPerMM);
                                    // update steps label box
                                    ui->spinBox_gantryAbsSteps_X->setValue(gantryPos[0]);
                                    ui->spinBox_gantryAbsSteps_Y->setValue(gantryPos[1]);
                                    ui->spinBox_gantryAbsSteps_Z->setValue(gantryPos[2]);
                                    // update mm label box
                                    ui->doubleSpinBox_gantryAbsMM_X->setValue( gantry_x );
                                    ui->doubleSpinBox_gantryAbsMM_Y->setValue( gantry_y );
                                    ui->doubleSpinBox_gantryAbsMM_Z->setValue( gantry_z );
                                    // Send command to the Gantry
//                                    gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_X, gantryPos[0]-gantryZeros[0]);
//                                    gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Y, gantryPos[1]-gantryZeros[1]);
//                                    gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Z, gantryPos[2]-gantryZeros[2]);
                                    int posxyz[3] = {gantryPos[0]-gantryZeros[0], gantryPos[1]-gantryZeros[1], gantryPos[2]-gantryZeros[2]};
//                                    int posxyz[3] = {gantryPos[0], gantryPos[1], gantryPos[2]};
                                    gantryMoveMulti(posxyz);
                                    Sleep(8000);
                                    qInfo() << "GantryP is: "<<gantry_x<<gantry_y<<gantry_z;
//                                    gantry_z = gantry_z - step_z;
                                    if (gantryZupwardFlag == true)
                                    {
                                        gantry_z = gantry_z + step_z;
                                    }
                                    if (gantryZdownwardFlag == true)
                                    {
                                        gantry_z = gantry_z - step_z;
                                    }
                                }
                            }
                        }
                        else // single loop data collection is done!
                        {
                            qInfo()<<"Loop " << loopcount <<" is done! Now re-zero gantry!!!";
                            singleloopdone = true;
                            // call Gantry Zero twice to dimish the accumulated position error
                            gantryZero();
                            Sleep(15000);
                            gantryZero();
                            Sleep(10000);

                            //initialize gantry each time at start, to avoid gantry position error accumulated, as gantry runs in a open loop
                            gantry_x = gantryinitcorner[0];
                            gantry_y = gantryinitcorner[1];
                            gantry_z = gantryinitcorner[2];
                            // move gantry to desired position
                            gantryPos[0] = (int) (gantry_x * gantryStepsPerMM);
                            gantryPos[1] = (int) (gantry_y * gantryStepsPerMM);
                            gantryPos[2] = (int) (gantry_z * gantryStepsPerMM);
                            // update steps label box
                            ui->spinBox_gantryAbsSteps_X->setValue(gantryPos[0]);
                            ui->spinBox_gantryAbsSteps_Y->setValue(gantryPos[1]);
                            ui->spinBox_gantryAbsSteps_Z->setValue(gantryPos[2]);
                            // update mm label box
                            ui->doubleSpinBox_gantryAbsMM_X->setValue( gantry_x );
                            ui->doubleSpinBox_gantryAbsMM_Y->setValue( gantry_y );
                            ui->doubleSpinBox_gantryAbsMM_Z->setValue( gantry_z );
                            // Send command to the Gantry
//                            gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_X, gantryPos[0]-gantryZeros[0]);
//                            gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Y, gantryPos[1]-gantryZeros[1]);
//                            gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Z, gantryPos[2]-gantryZeros[2]);
                            int posxyz[3] = {gantryPos[0]-gantryZeros[0], gantryPos[1]-gantryZeros[1], gantryPos[2]-gantryZeros[2]};
//                            int posxyz[3] = {gantryPos[0], gantryPos[1], gantryPos[2]};
                            gantryMoveMulti(posxyz);
                            Sleep(8000); //wait 5 seconds to make sure gantry has enough time to move to the initial position
                            qInfo() << "GantryP is: "<<gantry_x<<gantry_y<<gantry_z;
                            qInfo() << "Gantry is re-initilized!!!";

                        }
                    }
                    else if (singleloopdone == true)
                    {
                        if (loopcount+1<=loop) //avoid update current at the last loop
                        {
                            // those code is copied from https://stackoverflow.com/questions/13445688/how-to-generate-a-random-number-in-c
                            // and is used to generate a random number, which is placed here for local use
                            std::random_device dev;
                            std::mt19937 rng(dev());
                            std::uniform_int_distribution<std::mt19937::result_type> dist6(0,maxCurrent*2); // distribution in range [a, b]
                            //

                            for (int i = 0; i < 8; i++)
                            {
                              I_command[i] =  dist6(rng)-maxCurrent; //generate random current in the range of [-maxCurrent, maxcurrent]
                            }
                            qInfo() << "Current is: ............................."<<I_command[0]<<", "<<I_command[1]<<", "<<I_command[2]<<", "<<I_command[3]<<", "<<I_command[4]<<", "<<I_command[5]<<", "<<I_command[6]<<", "<<I_command[7];
                            updateCurrents_CalibrationOnly(I_command);
                            qInfo() << "loop is " <<loopcount;

                        }
                        singleloopdone = false;
                        loopcount++;

                    }


                }
                else // loop reach max count, data collection finished!
                {
                    qInfo() << "!!!!!!!!!!!Data loop is finished, send 0 to S826!!!";
                    updateCurrents_CalibrationOnly(I_command); //send 0 to S826
                    Datacollectdoneflag = true;
                }
        }


//            qInfo() << "now we start to build the great wall with Neural Networks!!!";
        }
        else if (currentSubroutineMode ==10 && subroutineState)
        {
            /// 10. DATA COLLECTION FOR FIELD CALIBRATION WITH MODEL METHOD
            //
            // This function collects/records field [Bx,By,Bz] (fetched from gaussmeter/DAQ),
            // gaussmeter probe position [Px,Py,Pz] (fetched from gantry)
            // and coil currents [I1, I2,...,I8] (measured from S826).
            // The function also sends commands to S826 with different desired currents and gantry positions
            // PARAMETERS:
            // ---
            // --
            // -
            double I_command[8] = {0,0,0,0,0,0,0,0};
            if (gantry_x <= range_x)
            {
                if (gantry_y <= range_y)
                {
                    if (gantry_z <= range_z)
                    {
                        // move gantry to desired position
                        gantryPos[0] = (int) (gantry_x * gantryStepsPerMM);
                        gantryPos[1] = (int) (gantry_y * gantryStepsPerMM);
                        gantryPos[2] = (int) (gantry_z * gantryStepsPerMM);
                        // update steps label box
                        ui->spinBox_gantryAbsSteps_X->setValue(gantryPos[0]);
                        ui->spinBox_gantryAbsSteps_Y->setValue(gantryPos[1]);
                        ui->spinBox_gantryAbsSteps_Z->setValue(gantryPos[2]);
                        // update mm label box
                        ui->doubleSpinBox_gantryAbsMM_X->setValue( gantry_x );
                        ui->doubleSpinBox_gantryAbsMM_Y->setValue( gantry_y );
                        ui->doubleSpinBox_gantryAbsMM_Z->setValue( gantry_z );
                        // Send command to the Gantry
                        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_X, gantryPos[0]-gantryZeros[0]);
                        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Y, gantryPos[1]-gantryZeros[1]);
                        gantryWrite(GANTRY_MOVE_SINGLE, GANTRY_MOVE_ABS, GANTRY_Z, gantryPos[2]-gantryZeros[2]);
                        Sleep(100);
                         qInfo() << "GantryP is: "<<gantry_x<<gantry_y<<gantry_z;
                        if (count_a < numAct)
                        {
                            if (count_i < poolsize)
                            {
                                I_command[count_a] = currentpool[count_i];
                                qInfo() << "Current is: ............................."<<I_command[0]<<", "<<I_command[1]<<", "<<I_command[2]<<", "<<I_command[3]<<", "<<I_command[4]<<", "<<I_command[5]<<", "<<I_command[6]<<", "<<I_command[7];
                                updateCurrents_CalibrationOnly(I_command);

                                count_i++;
                                if (count_i>=poolsize)
                                {
                                    count_i = 0;
                                    count_a ++;
                                }
                            }
//                            else
//                            {
//                                count_i = 0;
//                                count_a ++;
//                            }
                            if (count_a >= numAct)
                            {
                                gantry_z = gantry_z + pos_inc;
                                count_a = 0;
                            }
                        }
//                        else
//                        {
//                            gantry_z = gantry_z + pos_inc;
//                            count_a = 0;
//                        }
                    }
                    else
                    {
                        gantry_z = -range_z;
                        gantry_y = gantry_y + pos_inc;
                    }
                }
                else
                {
                    gantry_y = -range_y;
                    gantry_z = -range_z;
                    gantry_x = gantry_x + pos_inc;
                }
            }
            else
            {
                qInfo() << "Data collection is completed!!!";
                updateCurrents_CalibrationOnly(I_command); //send 0 currents to S826
            }


        }
        else if (currentSubroutineMode == 11 && subroutineState)
        {
            /// 11. Linearly increasing field
            //
            // This function creates a linearly increasing field.
            // PARAMETERS:
            // Starting Field [mT]
            // Slope [mT/s]
            // Axis -- to be implemented
            // Direction -- to be implemented
            // Maximum Cap [mT]

            qint64 t = currentTime.elapsed()/1000.0; // convert current time to seconds

            B_Global_Desired[0] = (subThreadParameters[1] * t + subThreadParameters[0])/1000;
            qDebug() << B_Global_Desired[0];
            if (abs(B_Global_Desired[0]) <= abs(subThreadParameters[4]/1000))
            {
                updateCurrents();
            }
        }
        else if (currentSubroutineMode == 12 && subroutineState)
        {
            /// 12. ROTATING FIELD IN THE -X-Y45 and Z PLANE
            //
            // This function creates a rotating field in the global YZ plane
            // The parameters allow for control over the amplitude, phase, and
            // frequence of the rotating field.
            // PARAMETERS:
            // Magnitude
            // Frequency
            // Phase

            double t = currentTime.elapsed()/1000.0; // time in seconds
            if (subThreadParameters[1] >= 0) //rolling upwards right if frequency is positive, rolling downwards left if frequency is negative
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/
                B_Global_Desired[0] = -subThreadParameters[0]/1000.0*sin(theta)*sin(M_PI_4);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta)*cos(M_PI_4);
                B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(theta);

            }
            else
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/

                B_Global_Desired[0] = -subThreadParameters[0]/1000.0*sin(theta)*sin(M_PI_4);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta)*cos(M_PI_4);
                B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(theta);

            }
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }
        else if (currentSubroutineMode == 13 && subroutineState)
        {
            /// 13. ROTATING FIELD IN THE X-Y45 and Z PLANE
            //
            // This function creates a rotating field in the global YZ plane
            // The parameters allow for control over the amplitude, phase, and
            // frequence of the rotating field.
            // PARAMETERS:
            // Magnitude
            // Frequency
            // Phase

            double t = currentTime.elapsed()/1000.0; // time in seconds
            if (subThreadParameters[1] >= 0) //rolling upwards right if frequency is positive, rolling downwards left if frequency is negative
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/

                B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(theta)*sin(M_PI_4);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta)*cos(M_PI_4);
                B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(theta);
            }
            else
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/
                B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(theta)*sin(M_PI_4);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta)*cos(M_PI_4);
                B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(theta);

            }
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }

        else if (currentSubroutineMode == 14 && subroutineState)
        {
            /// 14. ROTATING FIELD IN THE 45 ROT_Y PLANE
            //
            // This function creates a rotating field in the global YZ plane
            // The parameters allow for control over the amplitude, phase, and
            // frequence of the rotating field.
            // PARAMETERS:
            // Magnitude
            // Frequency
            // Phase

            double t = currentTime.elapsed()/1000.0; // time in seconds
            if (subThreadParameters[1] >= 0) //rolling upwards right if frequency is positive, rolling downwards left if frequency is negative
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/

                B_Global_Desired[0] = subThreadParameters[0]/1000.0*cos(theta)*sin(M_PI_4);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);
                B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(theta)*cos(M_PI_4);
            }
            else
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/
                B_Global_Desired[0] = subThreadParameters[0]/1000.0*cos(theta)*sin(M_PI_4);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);
                B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(theta)*cos(M_PI_4);

            }
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }

        else if (currentSubroutineMode == 15 && subroutineState)
        {
            /// 15. ROTATING FIELD IN THE -45 ROT_Y PLANE
            //
            // This function creates a rotating field in the global YZ plane
            // The parameters allow for control over the amplitude, phase, and
            // frequence of the rotating field.
            // PARAMETERS:
            // Magnitude
            // Frequency
            // Phase

            double t = currentTime.elapsed()/1000.0; // time in seconds
            if (subThreadParameters[1] >= 0) //rolling upwards right if frequency is positive, rolling downwards left if frequency is negative
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/

                B_Global_Desired[0] = -subThreadParameters[0]/1000.0*cos(theta)*sin(M_PI_4);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);
                B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(theta)*cos(M_PI_4);
            }
            else
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/
                B_Global_Desired[0] = -subThreadParameters[0]/1000.0*cos(theta)*sin(M_PI_4);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);
                B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(theta)*cos(M_PI_4);

            }
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }

        else if (currentSubroutineMode == 16 && subroutineState)
        {
            /// 16. ROTATING FIELD IN THE PLANE at RotY at some degree
            //
            // This function creates a rotating field in the global YZ plane
            // The parameters allow for control over the amplitude, phase, and
            // frequence of the rotating field.
            // PARAMETERS:
            // Magnitude
            // Frequency
            // Phase
            // plane tilted degree

            double t = currentTime.elapsed()/1000.0; // time in seconds
            if (subThreadParameters[1] >= 0) //rolling upwards right if frequency is positive, rolling downwards left if frequency is negative
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/

                B_Global_Desired[0] = -subThreadParameters[0]/1000.0*cos(theta)*sin(subThreadParameters[3]);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);
                B_Global_Desired[2] = subThreadParameters[0]/1000.0*cos(theta)*cos(subThreadParameters[3]);
            }
            else
            {
                double theta = 2*M_PI*subThreadParameters[1]*t;
//                double mod = fmod(theta,2*M_PI);

//                if(mod<M_PI*2 && mod>M_PI)
//                {
//                    theta = -theta;
//                }
                    /*
                    B_Global_Desired[1] = subThreadParameters[0]/1000.0*sin(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(-2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);
                    B_Global_Desired[0] = subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*sin(M_PI_4);
                    B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2])*cos(M_PI_4);
                    B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(2*M_PI*subThreadParameters[1]*t+subThreadParameters[2]);*/
                B_Global_Desired[0] = -subThreadParameters[0]/1000.0*cos(theta)*sin(subThreadParameters[3]);
                B_Global_Desired[1] = -subThreadParameters[0]/1000.0*sin(theta);
                B_Global_Desired[2] = -subThreadParameters[0]/1000.0*cos(theta)*cos(subThreadParameters[3]);

            }
            global2local(tilt, roll, B_Global_Desired, B_Local_Desired);
            updateCurrents();
        }

    }




    // Re-evaluate if the overheating flag is warranted
    if (overheatingFlag)
    {
        bool stillOverheating = false;
        // Check all temperatures and re-evaluate if the system is still overheating
        for (int t = 0; t < 8; t++)
        {
            // If any temperatures are still greater than the max allowable temperature
            // then keep the overheatingFlag true.
            if ( measuredTemperatures[t] > maxAllowableTemp )
            {
                stillOverheating = true;
                break;
            }
        }

        // Only if the system is not still overheating will the flag be lifted.
        if ( !stillOverheating )
        {
            overheatingFlag = false;
        }
    }


    //Receiving UDP package from CTR-- once udp received, active field control!
    if(UDPflag == true)
    {
        qDebug() << "udp mode !";
        isGradientControlled = false; //set default as false
        processPendingDatagrams();
        connect(socket, SIGNAL(readyRead()), this, SLOT(processPendingDatagrams()),Qt::QueuedConnection);


        // first use two buttons to activiate filed, if (greybutton==true&&whitebutton==true)
        // then map gimbal[1] to field direction, set field magnitude as a constant value
        // use grey button to apply inverse field to open forceps

        //control filed direction for wrist orient control
        double b_amount = 16.0/1000.0; //T, default unit is Tesla, and T/m
        double b_z = -8.0;
        double theta = m_Outputs.LeftHaptic.Gimbal[0]; //unit: rad, positive means in +x-y quad, negtive means in -x-y quad

        //!for----- xy plane wrist rotate
//        double sub_b[3] = {b_amount*sin(theta), -b_amount*cos(theta), b_z/1000.0};
        //!for---- yz plane wrist rotate
        double sub_b[3] = {0.0, -b_amount*cos(theta), b_amount*sin(theta)};

        //this set could open jaws only in -x-y quad
//        double sub_Gb[5] = {0.0, 0.0, -0.2, 0.0, -0.2}; //sub_Gbzz = -(sub_Gbxx+sub_Gbyy) = 10, Fz = sub_Gbzz*m_z, unit: T/m

        double b_hold = 10.0/1000.0; //used for add friction to the wrist, and apply torque for forceps

        //!-----for yz plane forceps open
//        double sub_b_hold[3] = {b_hold*sin(theta), -b_hold*cos(theta), b_z/1000.0};
//        double sub_b_hold[3] = {0.0, 0.0, b_hold};
//        opposite field to open forceps and z field to add friciton
//        double sub_b_hold_oppo[3] = {-b_hold*sin(theta), b_hold*cos(theta), 1.7*b_hold};
        double sub_Gb[5] = {-0.05, 0.0, -0.1, -0.05, -0.1}; //sub_Gbzz = -(sub_Gbxx+sub_Gbyy) = 2.4, Fz = sub_Gbzz*m_z, unit: T/m

        //!-----for xy plane forceps open
//        double sub_b_hold[3] = {b_hold*sin(theta), -b_hold*cos(theta), b_z/1000.0};
//        double sub_b_hold[3] = {0.0, 0.0, b_hold};
        //opposite field to open forceps and z field to add friciton
        double sub_b_hold_oppo[3] = {-1.4*b_hold,  b_hold*cos(theta), -b_hold*sin(theta)};
//        double sub_Gb[5] = {-0.05, 0.0, -0.1, -0.05, -0.1}; //sub_Gbzz = -(sub_Gbxx+sub_Gbyy) = 2.4, Fz = sub_Gbzz*m_z, unit: T/m



        if(m_Outputs.LeftHaptic.GreyButton == false)
        {
            isGradientControlled = false;
            for (int k=0; k<numField; k++)
                B_Global_Desired[k] = sub_b[k];
            for (int k=0; k<numGrad; k++)
                B_Global_Desired[k+numField] = 0.0;

        }
        else if (m_Outputs.LeftHaptic.GreyButton == true)
        {
            //apply opposite field to open the jaws

            isGradientControlled = false;
            for (int k=0; k<numField; k++)
                B_Global_Desired[k] = sub_b_hold_oppo[k];
            for (int k=0; k<numGrad; k++)
                B_Global_Desired[k+numField] = 0.0;

            //apply gradient to open the jaws

           /* isGradientControlled = true;
            for (int k=0; k<numField; k++)
                B_Global_Desired[k] = sub_b_hold[k];
            for (int k=0; k<numGrad; k++)
                B_Global_Desired[k+numField] = sub_Gb[k];*/

//            //directly assign currents
//            double I_goal[8] = {24.0, 1.77, 22.72, 2.08, 1.89, 22.32, 2.27, 21.5};
//            updateCurrents_CalibrationOnly(I_goal);
        }
        else
        {
            for (int k=0; k<numField+numGrad; k++)
                B_Global_Desired[k] = 0.0;
        }

        std::cout <<"B_Global Desired:: ";
        for (int k=0; k<numField+numGrad; k++)
            std::cout <<B_Global_Desired[k] <<" ";
        std::cout<<std::endl;
        updateCurrents();

    }
//    else
//    {
//        //set field and gradient to 0
//        for (int k=0; k<numField+numGrad; k++) {
//            B_Global_Desired[k] = 0.0;
//        }
////        updateCurrents();
//    }

    if(xysampleflag == true)
    {
        double b_amount = 32.0/1000.0; //unit: T
        double inc = 1.0/36.0*M_PI;  //5 degree interval
        double initial = M_PI/10.0; //18 degree
        double mintheta = -M_PI/10.0;
        double theta = initial-inc*samplecount;
//        std::cout<<"theta = "<<theta<<std::endl;
        if(theta>=mintheta)
        {
            //when dipole toward to -y
            /*
            //at xy plane
            double sub_b_xy[3] = {b_amount*sin(theta), -b_amount*cos(theta), 0.0};
            //at yz plane
            double sub_b_yz[3] = {0, -b_amount*cos(theta), b_amount*sin(theta)};
            //at roty45 plane
            double sub_b_y45[3] = {b_amount*cos(theta)*sin(M_PI_4), -b_amount*sin(theta), b_amount*cos(theta)*cos(M_PI_4)};
            // at roty-45 plane
            double sub_b_y_45[3] = {-b_amount*cos(theta)*sin(M_PI_4), -b_amount*sin(theta), b_amount*cos(theta)*cos(M_PI_4)};
            */

            //when dipole toward to x

            //at xy plane
            double sub_b_xy[3] = {b_amount*cos(theta), b_amount*sin(theta), 0.0};
            //at xz plane
            double sub_b_xz[3] = {b_amount*cos(theta), 0.0, b_amount*sin(theta)};
            //at rotx45 plane
            double sub_b_x45[3] = {b_amount*cos(theta), -b_amount*sin(theta)*sin(M_PI_4), b_amount*sin(theta)*cos(M_PI_4)};
            // at roty-45 plane
            double sub_b_x_45[3] ={b_amount*cos(theta), b_amount*sin(theta)*sin(M_PI_4), b_amount*sin(theta)*cos(M_PI_4)};


            //when dipole toward to -z, downwards
/*
            //at yz plane
            double sub_b_yz[3] = {0.0, b_amount*sin(theta), -b_amount*cos(theta)};
            //at xz plane
            double sub_b_xz[3] = {b_amount*sin(theta), 0.0, -b_amount*cos(theta)};
            //at rotx45 plane
            double sub_b_x45[3] = {b_amount*sin(theta)*cos(M_PI_4), b_amount*sin(theta)*sin(M_PI_4), -b_amount*cos(theta)};
            // at roty-45 plane
            double sub_b_x_45[3] ={-b_amount*sin(theta)*cos(M_PI_4), b_amount*sin(theta)*sin(M_PI_4), -b_amount*cos(theta)};
*/

            isGradientControlled = false;
            for (int k=0; k<numField; k++)
                B_Global_Desired[k] = sub_b_xy[k];

            for (int k=0; k<numGrad; k++)
                B_Global_Desired[k+numGrad] = 0.0;
            updateCurrents();
        }
        else
            std::cout<<"Reach the boundary!!!"<<std::endl;

    }



}

