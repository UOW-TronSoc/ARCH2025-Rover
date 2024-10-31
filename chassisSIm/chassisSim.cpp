// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "matplotlibcpp.h"
#include <stdio.h>

using namespace std;
namespace plt = matplotlibcpp;

#define HeightMap false
#define rsStep 0.001


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
	gc.segment<7>(0) << 0, 0, 0.5, 1, 0, 0, 0;
	robot->setGeneralizedCoordinate(gc);
	robot->setGeneralizedVelocity(gv);
	damping.setConstant(0);
	damping.tail(4).setConstant(1.);
	robot->setJointDamping(damping);

	/// launch raisim server
	raisim::RaisimServer server(&world);

	/// this method should be called before server launch
	// auto scans = server.addInstancedVisuals("scan points",
	//                                         raisim::Shape::Box,
	//                                         {0.05, 0.05, 0.05},
	//                                         {1,0,0,1},
	//                                         {0,1,0,1});
	// int scanSize1 = 40;
	// int scanSize2 = 50;

	// scans->resize(scanSize1*scanSize2);
	server.launchServer();

	// Wait for server connection
    cout << "Awaiting Connection to raisim server" << endl;
    while (!server.isConnected())
        ;
    cout << "Server Connected" << endl;

	server.focusOn(robot);

	Eigen::Vector3d direction;

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
    vector<double> t(dur / rsStep);
    vector<vector<double>> frontLeftAngDesired(4, vector<double>(dur / rsStep));
    vector<vector<double>> frontLeftAngActual(4, vector<double>(dur / rsStep));
	vector<vector<double>> frontLeftTorqueActual(4, vector<double>(dur / rsStep));
	vector<vector<double>> frontLeftTorqueDesired(4, vector<double>(dur / rsStep));

	// Main Loop
	for(int time=0; time<dur/rsStep; time++) {
		// Real Time delay and physics integration
		RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
		server.integrateWorldThreadSafe();

		// Get Robot State
		gc = robot->getGeneralizedCoordinate().e();
		gv = robot->getGeneralizedVelocity().e();
		ga = robot->getGeneralizedAcceleration().e();
		gf = robot->getGeneralizedForce().e();

		// Add delay to input velocity
		if (time == 2000) {
			wheelVel[0] = maxWheelRads;
			wheelVel[1] = -maxWheelRads;
			wheelVel[2] = maxWheelRads;
			wheelVel[3] = -maxWheelRads;
		}


		// PID Controller wit velocity input and torque output
		for (size_t wheel = 0; wheel < 4; wheel++) {

			Tp = kp * (wheelVel[wheel] - gv[6 + wheel]);
			Td = kd * (wheelVel[wheel] - gv[6 + wheel]) / rsStep;

			controlForce[wheel] = wheelVel[wheel] + Tp + Td;

			controlForce[wheel] = max(min(controlForce[wheel], gf[6 + wheel] + maxWheelTorqueDelta), gf[6 + wheel] - maxWheelTorqueDelta);

			controlForce[wheel] = max(min(controlForce[wheel], maxwheelTorque), -maxwheelTorque);

			// For Plots
			t[time] = time;
			frontLeftAngDesired[wheel][time] = wheelVel[wheel];
			frontLeftAngActual[wheel][time] = gv[6 + wheel];
			frontLeftTorqueDesired[wheel][time] = controlForce[wheel];
			frontLeftTorqueActual[wheel][time] = gf[6 + wheel];
		}

		// Control Delay
		if (time < 2000) {
			continue;
		}

		robot->setGeneralizedForce({0, 0, 0, 0, 0, 0, controlForce[0], controlForce[1], controlForce[2], controlForce[3]});
		
		// If fallen off edge send back to centre
		if(fabs(gc[0])>35. || fabs(gc[1])>35.) {
		  gc.segment<7>(0) << 0, 0, 2, 1, 0, 0, 0;
		  gv.setZero();
		  robot->setState(gc, gv);
		}
	}

	server.killServer();

	plt::figure_size(1366, 768);
	plt::plot(t, frontLeftAngDesired[0], "r-");
	plt::plot(t, frontLeftAngActual[0], "b-");
	plt::title("Front Left Wheel Angular Velocity");
	plt::ylabel("Angular Velocity (rad/s)");
	plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftTorqueDesired[0], "r-");
	// plt::plot(t, frontLeftTorqueActual[0], "b-");
	// plt::title("Front Left Wheel Torque");
	// plt::ylabel("Torque (Nm)");
	// plt::xlabel("Time (ms)");

	plt::figure_size(1366, 768);
	plt::plot(t, frontLeftAngDesired[1], "r-");
	plt::plot(t, frontLeftAngActual[1], "b-");
	plt::title("Front Right Wheel Angular Velocity");
	plt::ylabel("Angular Velocity (rad/s)");
	plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftTorqueDesired[0], "r-");
	// plt::plot(t, frontLeftTorqueActual[0], "b-");
	// plt::title("Front Right Wheel Torque");
	// plt::ylabel("Torque (Nm)");
	// plt::xlabel("Time (ms)");

	plt::figure_size(1366, 768);
	plt::plot(t, frontLeftAngDesired[2], "r-");
	plt::plot(t, frontLeftAngActual[2], "b-");
	plt::title("Rear Left Wheel Angular Velocity");
	plt::ylabel("Angular Velocity (rad/s)");
	plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftTorqueDesired[2], "r-");
	// plt::plot(t, frontLeftTorqueActual[2], "b-");
	// plt::title("Rear Left Wheel Torque");
	// plt::ylabel("Torque (Nm)");
	// plt::xlabel("Time (ms)");

	plt::figure_size(1366, 768);
	plt::plot(t, frontLeftAngDesired[3], "r-");
	plt::plot(t, frontLeftAngActual[3], "b-");
	plt::title("Rear Right Wheel Angular Velocity");
	plt::ylabel("Angular Velocity (rad/s)");
	plt::xlabel("Time (ms)");

	// plt::figure_size(1366, 768);
	// plt::plot(t, frontLeftTorqueDesired[3], "r-");
	// plt::plot(t, frontLeftTorqueActual[3], "b-");
	// plt::title("Rear Right Wheel Torque");
	// plt::ylabel("Torque (Nm)");
	// plt::xlabel("Time (ms)");

	plt::show();
}
