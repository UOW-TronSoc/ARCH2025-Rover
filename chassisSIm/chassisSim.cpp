// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "matplotlibcpp.h"
#include <stdio.h>
#include <fstream>
#include <memory>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <csignal>
#include "../modules/localController/getController.h"
#include "../modules/utilities/utils.h"

using namespace std;
namespace plt = matplotlibcpp;

#define HeightMap true
#define rsStep 0.001

// Variables to share between threads
atomic<bool> running(true);
atomic<float> inputAxisX(0.0f);
atomic<float> inputAxisY(0.0f);
atomic<float> inputAxisW(0.0f);
atomic<float> inputAxisZ(0.0f);
atomic<bool> buttonY(false);
atomic<bool> wasButtonYPressed(false);

// Interrupt handler
void signalHandler(int signum) {
    running = false; // Stop the loops
}

// Function for reading from controller in new thread
void readController() {
    // Controller connection status
    int fd;

    // Loop until the Controller is successfully opened
    while (true) {

        // Input event stream for contorller (might need to adjust for when event changes )
        const string device = SteelSeriesDeviceFinder::get_device_path("SteelSeries Stratus XL");

        // Access controller
        fd = open(device.c_str(), O_RDONLY);

        if (fd == -1) {
            cerr << "Failed to open input device "<<device<<". Retrying..." << endl;
            sleep(1);  // Wait for 1 second before trying again
        } else {
            cout << "Connected to device: " << device << endl;
            break;  // Exit loop when successfully opened
        }
    }

    struct input_event ev;

    // Counter for low velocity to allow joystick parsing quickly through zero zone
    static int lowVelCounter = 0;

    // Controlle reading cycle
    while (running) {
        // Read event
        ssize_t n = read(fd, &ev, sizeof(ev));

        // Check if event read failes
        if (n == (ssize_t)-1) {
            cerr << "Failed to read input event." << endl;
            break;
        }

        // Only read button press (key) and joystick (abs) readings
        if (ev.type == EV_ABS || ev.type == EV_KEY) {
            // Handle deadzone for Right Trigger (code 9) and Left Trigger (code 10)
            if ((ev.code == 9 || ev.code == 10) && abs(ev.value) < 2048) {
                continue; // Skip further processing if within the deadzone
            }

            // Switch case for each event handling
            switch (ev.code) {
                case 0: // Left Joystick X
                    // cout << "Left Joystick X: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;
                    inputAxisX = ev.value;
                    break;
                case 1: // Left Joystick Y
                    // cout << "Left Joystick Y: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Up" : "Down") << ")" << endl;

                    inputAxisY = ev.value;
                    break;
                case 2: // Right Joystick X
                    // cout << "Right Joystick X: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;
					inputAxisW = ev.value;
                    break;
                case 5: // Right Joystick Y
                    // cout << "Right Joystick Y: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Up" : "Down") << ")" << endl;
					inputAxisZ = ev.value;
                    break;
                case 16: // Dpad X
                    // cout << "Dpad X: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;
                    // dPadX = ev.value;
                    break;
                case 17: // DPad Y
                    // cout << "DPad Y: " << ev.value << " (" 
                    //           << (ev.value > 0 ? "Down" : "Up") << ")" << endl;
                    // dPadY = ev.value;
                    break;
                // case 9: // Right Trigger
                //     // cout << "Right Trigger: " << ev.value << endl;
                //     break;
                // case 10: // Left Trigger
                //     // cout << "Left Trigger: " << ev.value << endl;
                //     break;
                // case 317: // Left Joystick Click
                //     // cout << "Left Joystick Click: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                case 318: // Right Joystick Click
                    // cout << "Right Joystick Click: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    // running = false;
                    break;
                case 304: // Button A
                    // cout << "Button A: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    // buttonA = ev.value;
                    break;
                case 307: // Button X
                    // cout << "Button X: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    // buttonX = ev.value;
                    break;
                case 305: // Button B
                    // static bool wasButtonBPressed = false; // Track the previous state of Button Y
                    // // cout<<"ButtonB Pressed: " <<endl;
                    // if (ev.value == 1 && !wasButtonBPressed) {
                    //     // Button B is pressed and was not pressed before (i.e., transition from unpressed to pressed)
                    //     cout<<"ButtonB Pressed: " <<endl;
                        
                    //     buttonB = true;
                        
                    //     hexapod.operationDuration = 0;
                        
                    //     wasButtonBPressed = true; // Update the state to pressed
                    // } else {
                    //     // Button B is released, reset the state
                    //     buttonB = false;
                    //     wasButtonBPressed = false;
                    // }
                    // break;
                case 308: // Button Y
					static bool wasButtonYPressed = false; // Track the previous state of Button Y
					
					if (ev.value == 1 && !wasButtonYPressed) {
						// Button Y is pressed and was not pressed before (rising edge)
						buttonY = true;  // Signal the button press
						wasButtonYPressed = true; // Update the state to pressed
					} else if (ev.value == 0) {
						// Button Y is released, reset the press state
						wasButtonYPressed = false;
						buttonY = false;  // Clear buttonY to prevent continuous activation
					}
					break;
                case 311: // Right Bumper
                    // cout << "Right Bumper: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    // rightBumper = ev.value;
                    break;
                case 310: // Left Bumper
                    // cout << "Left Bumper: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    // leftBumper = ev.value;
                    break;
                default:
                    // cout << "Unknown event. Code: " << ev.code << " Value: " << ev.value << endl;
                    break;
            }
        }
    }

    // close the device connection
    close(fd);
}


int main(int argc, char* argv[]) {
	auto binaryPath = raisim::Path::setFromArgv(argv[0]);

	raisim::World world;
	world.setTimeStep(rsStep);

	if (HeightMap) {
		/// create objects
		raisim::TerrainProperties terrainProperties;
		terrainProperties.frequency = 0.2;
		terrainProperties.zScale = 2.0;
		terrainProperties.xSize = 70.0;
		terrainProperties.ySize = 70.0;
		terrainProperties.xSamples = 70;
		terrainProperties.ySamples = 70;
		terrainProperties.fractalOctaves = 3;
		terrainProperties.fractalLacunarity = 2.0;
		terrainProperties.fractalGain = 0.25;

		auto hm = world.addHeightMap(0.0, 0.0, terrainProperties);
		hm->setAppearance("soil2");
	} else {
		auto ground = world.addGround(0);
	}

	auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/huskyDemo/husky.urdf");
	robot->setName("smb");
	Eigen::VectorXd gc(robot->getGeneralizedCoordinateDim()), gv(robot->getDOF()), ga(robot->getDOF()), gf(robot->getDOF()), damping(robot->getDOF());
	gc.setZero(); gv.setZero();
	gc.segment<7>(0) << 0, 0, 2, 1, 0, 0, 0;
	robot->setGeneralizedCoordinate(gc);
	robot->setGeneralizedVelocity(gv);
	damping.setConstant(0);
	damping.tail(4).setConstant(1.);
	robot->setJointDamping(damping);

	/// launch raisim server
	raisim::RaisimServer server(&world);

	// scans->resize(scanSize1*scanSize2);
	server.launchServer();

	// Start the controller input reading in a separate thread
    thread input_thread(readController);
    // Register signal handler
    signal(SIGINT, signalHandler);

	bool active = false;

	// Wait for server connection
    cout << "Awaiting Connection to raisim server" << endl;
    while (!server.isConnected())
        ;
    cout << "Server Connected" << endl;

	server.focusOn(robot);

	int motorV = 24;
	int motorKv = 100;
	double motorTorque = 1;
	double ESCeff = 0.78;
	double maxWheelRPM = motorV * motorKv * ESCeff;
	int reduction = 50;
	double maxWheelRads = maxWheelRPM * M_PI/30 / 50;
	double maxwheelTorque = motorTorque * reduction;
	double maxWheelTorqueDelta = 5;

	double wheelVel[4] = {0.0, 0.0, 0.0, 0.0};
	double controlForce[4] = {0.0, 0.0, 0.0, 0.0};

	double kp = 10.0;
	double kd = 0.00;

	double Tp;
	double Td;

	int dur = 5;

	// Create vector variables for plotting
    // vector<double> t(dur / rsStep);
    // vector<vector<double>> frontLeftAngDesired(4, vector<double>(dur / rsStep));
    // vector<vector<double>> frontLeftAngActual(4, vector<double>(dur / rsStep));
	// vector<vector<double>> frontLeftTorqueActual(4, vector<double>(dur / rsStep));
	// vector<vector<double>> frontLeftTorqueDesired(4, vector<double>(dur / rsStep));

	int time = 0;

	// Main Loop
	while(running) {
		// Real Time delay and physics integration
		RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
		server.integrateWorldThreadSafe();

		// Get Robot State
		gc = robot->getGeneralizedCoordinate().e();
		gv = robot->getGeneralizedVelocity().e();
		ga = robot->getGeneralizedAcceleration().e();
		gf = robot->getGeneralizedForce().e();

		if (active) {
			wheelVel[0] = (inputAxisZ / 2047) * maxWheelRads;
			wheelVel[1] = (inputAxisY / 2047) * maxWheelRads;
			wheelVel[2] = (inputAxisZ / 2047) * maxWheelRads;
			wheelVel[3] = (inputAxisY / 2047) * maxWheelRads;
		} else {
			wheelVel[0] = 0.0;
			wheelVel[1] = 0.0;
			wheelVel[2] = 0.0;
			wheelVel[3] = 0.0;
		}

			// PID Controller wit velocity input and torque output
		for (size_t wheel = 0; wheel < 4; wheel++) {

			Tp = kp * (wheelVel[wheel] - gv[6 + wheel]);
			Td = kd * (wheelVel[wheel] - gv[6 + wheel]) / rsStep;

			controlForce[wheel] = wheelVel[wheel] + Tp + Td;

			controlForce[wheel] = max(min(controlForce[wheel], gf[6 + wheel] + maxWheelTorqueDelta), gf[6 + wheel] - maxWheelTorqueDelta);

			controlForce[wheel] = max(min(controlForce[wheel], maxwheelTorque), -maxwheelTorque);

			// For Plots
			// t[time] = time;
			// frontLeftAngDesired[wheel][time] = wheelVel[wheel];
			// frontLeftAngActual[wheel][time] = gv[6 + wheel];
			// frontLeftTorqueDesired[wheel][time] = controlForce[wheel];
			// frontLeftTorqueActual[wheel][time] = gf[6 + wheel];
		}

		robot->setGeneralizedForce({0, 0, 0, 0, 0, 0, controlForce[0], controlForce[1], controlForce[2], controlForce[3]});

		if (buttonY) {  // Only runs on a true "buttonY" from rising edge detection
			if (!active) {
				// Activation code
				cout << "Turning on motors" << endl;
				active = true;
			} else {
				// Deactivation code
				cout << "Turning off motors" << endl;
				active = false;
			}
			buttonY = false;  // Reset buttonY after toggling to await the next rising edge
		}

		// If fallen off edge send back to centre
		if(fabs(gc[0])>35. || fabs(gc[1])>35.) {
		  gc.segment<7>(0) << 0, 0, 2, 1, 0, 0, 0;
		  gv.setZero();
		  robot->setState(gc, gv);
		}

		time++;
	}

	server.killServer();

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftAngDesired[0], "r-");
	// plt::plot(t, frontLeftAngActual[0], "b-");
	// plt::title("Front Left Wheel Angular Velocity");
	// plt::ylabel("Angular Velocity (rad/s)");
	// plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftTorqueDesired[0], "r-");
	// plt::plot(t, frontLeftTorqueActual[0], "b-");
	// plt::title("Front Left Wheel Torque");
	// plt::ylabel("Torque (Nm)");
	// plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftAngDesired[1], "r-");
	// plt::plot(t, frontLeftAngActual[1], "b-");
	// plt::title("Front Right Wheel Angular Velocity");
	// plt::ylabel("Angular Velocity (rad/s)");
	// plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftTorqueDesired[0], "r-");
	// plt::plot(t, frontLeftTorqueActual[0], "b-");
	// plt::title("Front Right Wheel Torque");
	// plt::ylabel("Torque (Nm)");
	// plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftAngDesired[2], "r-");
	// plt::plot(t, frontLeftAngActual[2], "b-");
	// plt::title("Rear Left Wheel Angular Velocity");
	// plt::ylabel("Angular Velocity (rad/s)");
	// plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftTorqueDesired[2], "r-");
	// plt::plot(t, frontLeftTorqueActual[2], "b-");
	// plt::title("Rear Left Wheel Torque");
	// plt::ylabel("Torque (Nm)");
	// plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftAngDesired[3], "r-");
	// plt::plot(t, frontLeftAngActual[3], "b-");
	// plt::title("Rear Right Wheel Angular Velocity");
	// plt::ylabel("Angular Velocity (rad/s)");
	// plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftTorqueDesired[3], "r-");
	// plt::plot(t, frontLeftTorqueActual[3], "b-");
	// plt::title("Rear Right Wheel Torque");
	// plt::ylabel("Torque (Nm)");
	// plt::xlabel("Time (ms)");

	plt::show();
}
