#include "../../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "kangaArm.h"
#include <iostream>
#include <fstream>
#include <memory>
#include "raisim/RaisimServer.hpp"

using namespace std;

const bool raisimSimulator = true;
const float rsStep = 5; // Real Time Step (ms)

RSTimedLoop rsLoop(rsStep);
// ArduinoController arduino("/dev/ttyACM0", 115200);

void parseCommand(const string& command, KangaArm &leg);

int main(int argc, char* argv[]) {
    Path binaryPath = raisim::Path::setFromArgv(argv[0]);

    fstream arduinoPort("/dev/ttyACM0");

    bool simulationMode;

    if (arduinoPort) {
        simulationMode = false;
        cout << "Arduino connected." << endl;
        
    } else {
        simulationMode = true;
        cout << "Arduino not connected. Running in simulation mode." << endl;
    }
    
    KangaArm leg(1, rsLoop, simulationMode, raisimSimulator, rsStep, binaryPath);

    string command;
    while (true) {
        cout << "Enter command: ";
        cin >> command;

        if (command == "exit") {
            break;
        };

        parseCommand(command, leg);
    }
    
    return 0;
}

void parseCommand(const string& command, KangaArm &leg) {
    if (command == "zero") {
        leg.moveToZero();
    } else if (command == "basic") {
        leg.moveToBasic();
    } else if (command == "off") {
        leg.moveToOff();
    } else if (command == "ikTest") {
        leg.doIKTest();
    } else if (command == "jacTest") {
        leg.doJacobianTest(1);
    } else {
        cout << "Unknown command: " << command << endl;
    }
}