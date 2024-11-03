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

#include <opencv2/opencv.hpp>

using namespace std;
namespace plt = matplotlibcpp;

#define FlatMap false
#define PlotMotors false
#define RSStep 0.005

/// Variables to share between threads

// Data from Base Station
atomic<bool> running(true);		// Toggles wheel activity
atomic<float> inputAxisX(0.0f); // Left Joystick X axis
atomic<float> inputAxisY(0.0f); // Left Joystick Y axis
atomic<float> inputAxisW(0.0f); // Right Joystick X axis
atomic<float> inputAxisZ(0.0f); // Right Joystick Y axis
atomic<bool> buttonY(false);	// Trigger for toggling running
atomic<bool> buttonB(false);	// Trigger for reseting rover position and velocity to lander

// Data t0 base station
atomic<char *> frontCameraData(nullptr); // Front Camera Data
atomic<char *> rearCameraData(nullptr);	 // Rear Camera Data
atomic<char *> belowCameraData(nullptr); // Below Camera Data
atomic<char *> aboveCameraData(nullptr); // Above Camera Data
atomic<double> motorCurrentDraw[4];		 // Current Draw Data
atomic<double> imuLinearAcceleration_data[3];
atomic<double> imuAngularVelocity_data[3];
atomic<double> imuOrientation[4]; // Quaternion w, x, y, z
atomic<double> batteryVoltage[4];
atomic<double> batteryCharge[4];

// Other shared variables
atomic<bool> wasButtonYPressed(false); // Ensures button press only occurs on button down (wont be needed from nase station)
atomic<bool> wasButtonBPressed(false); // Ensures button press only occurs on button down (wont be needed from nase station)
atomic<bool> buttonA(false);		   // :)
atomic<bool> buttonX(false);		   // :)
atomic<bool> wasButtonAPressed(false); // :)
atomic<bool> wasButtonXPressed(false); // :)
int speedFactor = 1;

// Dimensions of the images (make sure these match the camera settings)
const int width = 640;	// Replace with your actual width
const int height = 480; // Replace with your actual height
const int cameraFPS = 30;

// Interrupt handler
void signalHandler(int signum)
{
	running = false; // Stop the loops
}

// Function for reading from controller in new thread (Replace with function to interpret data/commands from abse station)
void readController()
{
	// Controller connection status
	int fd;

	// Loop until the Controller is successfully opened
	while (true)
	{

		// Input event stream for contorller (might need to adjust for when event changes )
		const string device = SteelSeriesDeviceFinder::get_device_path("SteelSeries Stratus XL");

		// Access controller
		fd = open(device.c_str(), O_RDONLY);

		if (fd == -1)
		{
			cerr << "Failed to open input device " << device << ". Retrying..." << endl;
			sleep(1); // Wait for 1 second before trying again
		}
		else
		{
			cout << "Connected to device: " << device << endl;
			break; // Exit loop when successfully opened
		}
	}

	struct input_event ev;

	// Counter for low velocity to allow joystick parsing quickly through zero zone
	static int lowVelCounter = 0;

	// Controlle reading cycle
	while (running)
	{
		// Read event
		ssize_t n = read(fd, &ev, sizeof(ev));

		// Check if event read failes
		if (n == (ssize_t)-1)
		{
			cerr << "Failed to read input event." << endl;
			break;
		}

		// Only read button press (key) and joystick (abs) readings
		if (ev.type == EV_ABS || ev.type == EV_KEY)
		{
			// Handle deadzone for Right Trigger (code 9) and Left Trigger (code 10)
			if ((ev.code == 9 || ev.code == 10) && abs(ev.value) < 2048)
			{
				continue; // Skip further processing if within the deadzone
			}

			// Switch case for each event handling
			switch (ev.code)
			{
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
				running = false;
				break;
			case 304:								   // Button A
				static bool wasButtonAPressed = false; // Track the previous state of Button Y

				if (ev.value == 1 && !wasButtonAPressed)
				{
					// Button Y is pressed and was not pressed before (rising edge)
					buttonA = true;			  // Signal the button press
					wasButtonAPressed = true; // Update the state to pressed
				}
				else if (ev.value == 0)
				{
					// Button Y is released, reset the press state
					wasButtonAPressed = false;
					buttonA = false; // Clear buttonB to prevent continuous activation
				}
				break;
			case 307:								   // Button X
				static bool wasButtonXPressed = false; // Track the previous state of Button Y

				if (ev.value == 1 && !wasButtonXPressed)
				{
					// Button Y is pressed and was not pressed before (rising edge)
					buttonX = true;			  // Signal the button press
					wasButtonXPressed = true; // Update the state to pressed
				}
				else if (ev.value == 0)
				{
					// Button Y is released, reset the press state
					wasButtonXPressed = false;
					buttonX = false; // Clear buttonB to prevent continuous activation
				}
				break;
			case 305:								   // Button B
				static bool wasButtonBPressed = false; // Track the previous state of Button Y

				if (ev.value == 1 && !wasButtonBPressed)
				{
					// Button Y is pressed and was not pressed before (rising edge)
					buttonB = true;			  // Signal the button press
					wasButtonBPressed = true; // Update the state to pressed
				}
				else if (ev.value == 0)
				{
					// Button Y is released, reset the press state
					wasButtonBPressed = false;
					buttonB = false; // Clear buttonB to prevent continuous activation
				}
				break;
			case 308:								   // Button Y
				static bool wasButtonYPressed = false; // Track the previous state of Button Y

				if (ev.value == 1 && !wasButtonYPressed)
				{
					// Button Y is pressed and was not pressed before (rising edge)
					buttonY = true;			  // Signal the button press
					wasButtonYPressed = true; // Update the state to pressed
				}
				else if (ev.value == 0)
				{
					// Button Y is released, reset the press state
					wasButtonYPressed = false;
					buttonY = false; // Clear buttonY to prevent continuous activation
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

// Function to run in a separate thread to process and display images (Replace with function to send image to base station)
void displayImages()
{
	while (running)
	{

		// Check if we have new data for Camera 1
		if (frontCameraData.load() != nullptr)
		{
			// Create a cv::Mat from the raw data and convert BGRA to BGR
			cv::Mat imageBGRAFront(height, width, CV_8UC4, frontCameraData.load());
			cv::Mat imageBGRFront;
			cv::cvtColor(imageBGRAFront, imageBGRFront, cv::COLOR_BGRA2BGR);
			cv::imshow("Front Camera Image", imageBGRFront);

			frontCameraData.store(nullptr); // Reset the pointer after processing
		}

		// Check if we have new data for Camera 2
		if (rearCameraData.load() != nullptr)
		{
			// Create a cv::Mat from the raw data and convert BGRA to BGR
			cv::Mat imageBGRARear(height, width, CV_8UC4, rearCameraData.load());
			cv::Mat imageBGRRear;
			cv::cvtColor(imageBGRARear, imageBGRRear, cv::COLOR_BGRA2BGR);
			cv::imshow("Rear Camera Image", imageBGRRear);

			rearCameraData.store(nullptr); // Reset the pointer after processing
		}

		// Check if we have new data for Camera 3
		if (aboveCameraData.load() != nullptr)
		{
			// Create a cv::Mat from the raw data and convert BGRA to BGR
			cv::Mat imageBGRAabove(height, width, CV_8UC4, aboveCameraData.load());
			cv::Mat imageBGRabove;
			cv::cvtColor(imageBGRAabove, imageBGRabove, cv::COLOR_BGRA2BGR);
			cv::imshow("Above Camera Image", imageBGRabove);

			aboveCameraData.store(nullptr); // Reset the pointer after processing
		}

		// Check if we have new data for Camera 4 (wide)
		if (belowCameraData.load() != nullptr)
		{
			// Create a cv::Mat from the raw data and convert BGRA to BGR
			cv::Mat imageBGRAbelow(height, width, CV_8UC4, belowCameraData.load());
			cv::Mat imageBGRbelow;
			cv::cvtColor(imageBGRAbelow, imageBGRbelow, cv::COLOR_BGRA2BGR);
			cv::imshow("Below Camera Image", imageBGRbelow);

			belowCameraData.store(nullptr); // Reset the pointer after processing
		}

		cv::waitKey(10); // Minimal delay to allow OpenCV to process display events
	}
}

// Function to update atomic arrays with IMU readings
void updateIMUData(raisim::InertialMeasurementUnit *imu)
{
	// Get IMU readings
	auto linear_acceleration = imu->getLinearAcceleration();
	auto angular_velocity = imu->getAngularVelocity();
	auto orientation = imu->getOrientation(); // Quaternion (w, x, y, z)

	// Update linear acceleration
	imuLinearAcceleration_data[0] = linear_acceleration[0];
	imuLinearAcceleration_data[1] = linear_acceleration[1];
	imuLinearAcceleration_data[2] = linear_acceleration[2];

	// Update angular velocity
	imuAngularVelocity_data[0] = angular_velocity[0];
	imuAngularVelocity_data[1] = angular_velocity[1];
	imuAngularVelocity_data[2] = angular_velocity[2];

	// Update orientation quaternion
	imuOrientation[0] = orientation[0]; // w
	imuOrientation[1] = orientation[1]; // x
	imuOrientation[2] = orientation[2]; // y
	imuOrientation[3] = orientation[3]; // z
}

int main(int argc, char *argv[])
{
	auto binaryPath = raisim::Path::setFromArgv(argv[0]);
	raisim::RaiSimMsg::setFatalCallback([]()
										{ throw; });
	// Setup World
	raisim::World world;
	world.setTimeStep(RSStep);

	// Setup Ground
	raisim::TerrainProperties terrainProperties;
	terrainProperties.zScale = FlatMap ? 0.0 : 1.8;
	terrainProperties.frequency = 0.2;
	terrainProperties.xSize = 70.0;
	terrainProperties.ySize = 70.0;
	terrainProperties.xSamples = 70;
	terrainProperties.ySamples = 70;
	terrainProperties.fractalOctaves = 3;
	terrainProperties.fractalLacunarity = 2.0;
	terrainProperties.fractalGain = 0.25;
	auto hm = world.addHeightMap(0.0, 0.0, terrainProperties, "sand");
	hm->setAppearance("soil2");

	// Add lander platform
	raisim::Mat<3, 3> inertia;
	inertia.setIdentity();
	raisim::Vec<3> com = {0, 0, 0};
	auto lander = world.addMesh(binaryPath.getDirectory() + "/rsc/environment/meshes/lander.obj", 1.0, inertia, com);
	lander->setPosition(raisim::Vec<3>{0, 0, 1.5});
	lander->setBodyType(raisim::BodyType::STATIC);

	// Add Rover
	// auto rover = world.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/huskyDemo/urdf/husky.urdf");
	auto rover = world.addArticulatedSystem(binaryPath.getDirectory() + "/rsc/kanga/urdf/kangaSensors.urdf");
	rover->setName("smb");
	Eigen::VectorXd gc(rover->getGeneralizedCoordinateDim()), gv(rover->getDOF()), ga(rover->getDOF()), gf(rover->getDOF()), damping(rover->getDOF());
	gc.setZero();
	gv.setZero();
	gc.segment<7>(0) << 0, 0, 2.3, 0, 0, 0, 1;
	rover->setGeneralizedCoordinate(gc);
	rover->setGeneralizedVelocity(gv);
	damping.setConstant(0);
	damping.tail(4).setConstant(1.);
	rover->setJointDamping(damping);

	// Virtual Sensors
	auto frontCam = rover->getSensorSet("camera_front_camera_parent")->getSensor<raisim::RGBCamera>("color");
	frontCam->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
	auto rearCam = rover->getSensorSet("camera_rear_camera_parent")->getSensor<raisim::RGBCamera>("color");
	rearCam->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
	auto aboveCam = rover->getSensorSet("camera_above_camera_parent")->getSensor<raisim::RGBCamera>("color");
	frontCam->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
	auto belowCam = rover->getSensorSet("camera_below_camera_parent")->getSensor<raisim::RGBCamera>("wide");
	rearCam->setMeasurementSource(raisim::Sensor::MeasurementSource::VISUALIZER);
	auto imu = rover->getSensorSet("imu_camera_parent")->getSensor<raisim::InertialMeasurementUnit>("imu");

	/// System Variables
	// Power Params
	double batteryVoltageMaX = 25.2;
	double batteryVoltageMin = 18;
	double buckConverterOutput = 24;
	double batteryAh = 16;
	double batteryWh = batteryAh * batteryVoltageMaX;
	double remainingBatteryCapacityWh = batteryWh;
	double currentBatteryVoltage = batteryVoltageMaX;
	double computingPower = 0;
	double motorPower = 0;
	double nvidiaPowerConsumption = 10;

	// Drivetrain Parameter
	double motorI = 10;
	int motorKv = 100;
	double motorKt = (15 * sqrt(3) / M_PI) / motorKv;
	double ESCeff = 0.78;
	int reduction = 50;
	double gearboxEff = 0.9;
	double maxGearboxTorque = 20;

	double maxWheelRPM;
	double maxWheelRads;
	double maxMotorTorque = motorKt * motorI;
	double maxWheelTorque = maxMotorTorque * reduction;

	// Control Parameters
	bool active = false;
	double wheelVel[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double controlForce[4] = {0.0, 0.0, 0.0, 0.0};
	double kp = 10.0;
	double kd = 0.00;
	double Tp;
	double Td;
	double maxWheelTorqueDelta = 5;
	double differentialTorque = 0;

	/// Launch raisim server
	raisim::RaisimServer server(&world);
	server.launchServer();

	// Start the controller input reading in a separate thread
	thread input_thread(readController);
	// Register signal handler
	signal(SIGINT, signalHandler);
	// Start camera display thread
	thread displayThread(displayImages);

	// Wait for server connection
	cout << "Awaiting Connection to raisim server" << endl;
	while (!server.isConnected())
		;
	cout << "Server Connected" << endl;
	server.focusOn(rover);

	// Plotting duration
	int dur = 20;

	// Create vector variables for plotting
	vector<double> t(dur / RSStep);
	vector<vector<double>> frontLeftAngDesired(4, vector<double>(dur / RSStep));
	vector<vector<double>> frontLeftAngActual(4, vector<double>(dur / RSStep));
	vector<vector<double>> frontLeftTorqueActual(4, vector<double>(dur / RSStep));
	vector<vector<double>> frontLeftTorqueDesired(4, vector<double>(dur / RSStep));

	// Set start time
	int time = 0;
	auto startTime = chrono::high_resolution_clock::now();

	// Main Loop
	while (running)
	{
		// Real Time delay and physics integration
		RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
		server.integrateWorldThreadSafe();

		// Get rover State (In real rover, replace with functions that get the current data from sensors)
		gc = rover->getGeneralizedCoordinate().e();
		gv = rover->getGeneralizedVelocity().e();
		ga = rover->getGeneralizedAcceleration().e();
		gf = rover->getGeneralizedForce().e();

		// Calculate Battery power loss (Will be handled in microcontrollers for main system so replace with read from internal comms, talk to electrical)
		currentBatteryVoltage = 113.91062 * pow(-(1 - remainingBatteryCapacityWh / batteryWh) + 0.48318, 5) + 22.2;

		// Set Motor Speeds
		if (active)
		{
			maxWheelRPM = max(currentBatteryVoltage, buckConverterOutput) * motorKv * ESCeff;
			maxWheelRads = maxWheelRPM * M_PI / 30 / 50;

			wheelVel[1] = speedFactor * (inputAxisY / 2047) * maxWheelRads;	 // BL
			wheelVel[2] = speedFactor * (inputAxisY / 2047) * maxWheelRads;	 // BR
			wheelVel[4] = speedFactor * (-inputAxisZ / 2047) * maxWheelRads; // FR
			wheelVel[5] = speedFactor * (-inputAxisZ / 2047) * maxWheelRads; // BR
		}
		else
		{
			wheelVel[1] = 0.0;
			wheelVel[2] = 0.0;
			wheelVel[4] = 0.0;
			wheelVel[5] = 0.0;
		}

		// Per Joint
		for (size_t wheel = 0; wheel < 6; wheel++)
		{
			if (wheel == 0 || wheel == 3)
			{
				continue;
			}
			// PID Controller with velocity input and torque output
			// PID Terms
			Tp = kp * (wheelVel[wheel] - gv[6 + wheel]);
			Td = kd * (wheelVel[wheel] - gv[6 + wheel]) / RSStep;

			// Base Torques
			controlForce[wheel] = wheelVel[wheel] + Tp + Td;
			// Smoothed Torques
			controlForce[wheel] = max(min(controlForce[wheel], gf[6 + wheel] + maxWheelTorqueDelta), gf[6 + wheel] - maxWheelTorqueDelta);
			// Capped Torques
			controlForce[wheel] = max(min(controlForce[wheel], max(maxWheelTorque, maxGearboxTorque)), -max(maxWheelTorque, maxGearboxTorque));

			// For Plots
			if (PlotMotors && time < dur / RSStep)
			{
				t[time] = time;
				frontLeftAngDesired[wheel][time] = wheelVel[wheel];
				frontLeftAngActual[wheel][time] = gv[6 + wheel];
				frontLeftTorqueDesired[wheel][time] = controlForce[wheel];
				frontLeftTorqueActual[wheel][time] = gf[6 + wheel];
			}

			// Calculate motor current draws
			motorCurrentDraw[wheel] = abs(gf[6 + wheel]) / (reduction * gearboxEff * motorKt);
		}

		// Fake differential control (Not required on real model)
		differentialTorque = 60.0 * (gc[7] - gc[10]) + 2.0 * (gv[6] - gv[9]);

		// Send Forces to Rover simulation (For Real system replace with function to send values to motors via can)
		rover->setGeneralizedForce({0, 0, 0, 0, 0, 0, -differentialTorque, controlForce[1], controlForce[2], differentialTorque, controlForce[4], controlForce[5]});

		// Calculate Battery power loss (Will be handled in microcontrollers for main system so replace with read from internal comms, talk to electrical)
		motorPower = (motorCurrentDraw[1] + motorCurrentDraw[2] + motorCurrentDraw[4] + motorCurrentDraw[5]) * max(currentBatteryVoltage, buckConverterOutput);
		computingPower = nvidiaPowerConsumption;
		remainingBatteryCapacityWh -= (motorPower + computingPower) * (RSStep / 3600.0);

		// Check for battery depletion
		if (active && (currentBatteryVoltage <= batteryVoltageMin || remainingBatteryCapacityWh <= 0))
		{
			remainingBatteryCapacityWh -= (motorPower + computingPower) * (RSStep / 3600.0);
			cout << "BATTERY DEPLETED" << endl
				 << "TURNING OFF KANGA" << endl;
			active = false;
		}

		// Display Power Information (Only needed until display made)
		if ((currentBatteryVoltage > batteryVoltageMin && remainingBatteryCapacityWh > 0) && (time % (1000 / cameraFPS)) < RSStep)
		{
			cout << fixed << setprecision(2);
			// cout << "Front Left Current Draw: " << motorCurrentDraw[1] << " A, "
			// 	 << "Front Right Current Draw: " << motorCurrentDraw[2] << " A, "
			// 	 << "Rear Left Current Draw: " << motorCurrentDraw[4] << " A, "
			// 	 << "Rear Right Current Draw: " << motorCurrentDraw[5] << " A, "
			cout << "Motor Current Draw: " << motorCurrentDraw[1] + motorCurrentDraw[2] + motorCurrentDraw[4] + motorCurrentDraw[5] << " A,     "
				 << "Total Power Draw: " << motorPower + computingPower << " W,    "
				 << "Battery Percentage: " << remainingBatteryCapacityWh / batteryWh << " %,    "
				 << "Current Voltage: " << currentBatteryVoltage << " V,    "
				 << "Remaining Wh: " << remainingBatteryCapacityWh << " Wh    " << endl;
		}

		// Toggle rover activation
		if (buttonY)
		{ // Only runs on a true "buttonY" from rising edge detection
			// Enable Rover if battery isnt depleted and not currently active
			if (!active && currentBatteryVoltage > 18 && remainingBatteryCapacityWh > 0)
			{
				// Activation code
				cout << "TURNING ON MOTORS" << endl;
				active = true;
			}
			// Error message for if not active and battery depleted
			else if (!active)
			{
				cout << "BATTERY DEPLETED" << endl;
			}
			else
			// Deactive motors
			{
				// Deactivation code
				cout << "TURNING OFF MOTORS" << endl;
				active = false;
			}
			buttonY = false; // Reset buttonY after toggling to await the next rising edge
		}

		// Fun (not needed)
		if (buttonA)
		{
			speedFactor *= -1;
			buttonA = false;
		}

		// Fun (not needed)
		if (buttonX)
		{
			if (abs(speedFactor) == 2)
			{
				speedFactor *= 5;
			}
			else if (abs(speedFactor) == 10)
			{
				speedFactor /= 10;
			}
			else
			{
				speedFactor *= 2;
			}
			buttonX = false;
		}

		// If fallen off edge or triggered, rest kanga
		if (fabs(gc[0]) > 35. || fabs(gc[1]) > 35. || buttonB)
		{
			gc.segment<7>(0) << 0, 0, 2.3, 0, 0, 0, 1;
			gv.setZero();
			rover->setState(gc, gv);
			buttonB = false;
			remainingBatteryCapacityWh = batteryWh;
			cout << endl
				 << "KANGA RESET" << endl;
		}

		// Get IMU Data (Replace with function to read imu from sensor board through internal comms, talk to electrical)
		updateIMUData(imu);

		// Get Display only every couple iterations (For Real system replace with function to get camera data)
		if ((time % (1000 / cameraFPS)) < RSStep)
		{
			frontCameraData = frontCam->getImageBuffer().data();
			rearCameraData = rearCam->getImageBuffer().data();
			aboveCameraData = aboveCam->getImageBuffer().data();
			belowCameraData = belowCam->getImageBuffer().data();
		}

		// Save to atomic variables and add fluctuations for identification;
		batteryVoltage[0] = currentBatteryVoltage * 0.99;
		batteryVoltage[1] = currentBatteryVoltage * 1.02;
		batteryVoltage[2] = currentBatteryVoltage * 1.03;
		batteryVoltage[3] = currentBatteryVoltage * 0.96;
		batteryCharge[0] = remainingBatteryCapacityWh / 4 * 0.99;
		batteryCharge[1] = remainingBatteryCapacityWh / 4 * 1.02;
		batteryCharge[2] = remainingBatteryCapacityWh / 4 * 1.03;
		batteryCharge[3] = remainingBatteryCapacityWh / 4 * 0.96;

		// Increment Time
		time += RSStep * 1000;
	}

	server.killServer();

	// Calculate and display the duration
	auto endTime = chrono::high_resolution_clock::now();
	chrono::duration<double> elapsed = endTime - startTime;
	cout << "\nLoop duration: " << elapsed.count() << " seconds." << endl;

	if (PlotMotors)
	{
		plt::figure_size(1366, 768);
		plt::plot(t, frontLeftAngDesired[0], "r-");
		plt::plot(t, frontLeftAngActual[0], "b-");
		plt::title("Front Left Wheel Angular Velocity");
		plt::ylabel("Angular Velocity (rad/s)");
		plt::xlabel("Time (ms)");

		plt::figure_size(1366, 768);
		plt::plot(t, frontLeftTorqueDesired[0], "r-");
		plt::plot(t, frontLeftTorqueActual[0], "b-");
		plt::title("Front Left Wheel Torque");
		plt::ylabel("Torque (Nm)");
		plt::xlabel("Time (ms)");

		plt::figure_size(1366, 768);
		plt::plot(t, frontLeftAngDesired[1], "r-");
		plt::plot(t, frontLeftAngActual[1], "b-");
		plt::title("Front Right Wheel Angular Velocity");
		plt::ylabel("Angular Velocity (rad/s)");
		plt::xlabel("Time (ms)");

		plt::figure_size(1366, 768);
		plt::plot(t, frontLeftTorqueDesired[0], "r-");
		plt::plot(t, frontLeftTorqueActual[0], "b-");
		plt::title("Front Right Wheel Torque");
		plt::ylabel("Torque (Nm)");
		plt::xlabel("Time (ms)");

		plt::figure_size(1366, 768);
		plt::plot(t, frontLeftAngDesired[2], "r-");
		plt::plot(t, frontLeftAngActual[2], "b-");
		plt::title("Rear Left Wheel Angular Velocity");
		plt::ylabel("Angular Velocity (rad/s)");
		plt::xlabel("Time (ms)");

		plt::figure_size(1366, 768);
		plt::plot(t, frontLeftTorqueDesired[2], "r-");
		plt::plot(t, frontLeftTorqueActual[2], "b-");
		plt::title("Rear Left Wheel Torque");
		plt::ylabel("Torque (Nm)");
		plt::xlabel("Time (ms)");

		plt::figure_size(1366, 768);
		plt::plot(t, frontLeftAngDesired[3], "r-");
		plt::plot(t, frontLeftAngActual[3], "b-");
		plt::title("Rear Right Wheel Angular Velocity");
		plt::ylabel("Angular Velocity (rad/s)");
		plt::xlabel("Time (ms)");

		plt::figure_size(1366, 768);
		plt::plot(t, frontLeftTorqueDesired[3], "r-");
		plt::plot(t, frontLeftTorqueActual[3], "b-");
		plt::title("Rear Right Wheel Torque");
		plt::ylabel("Torque (Nm)");
		plt::xlabel("Time (ms)");

		plt::show();
	}
}
