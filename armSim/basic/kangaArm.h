#ifndef KANGAARM_H
#define KANGAARM_H

#include "../../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "../../modules/custom/raisimSimulator/raisimSimulator.h"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


using namespace std;

class KangaArm
{
public:
    KangaArm(unsigned int id, RSTimedLoop& rsLoop, bool simulationMode, bool raisimSimulator, float rsStep, Path binaryPath);
    ~KangaArm();

    Eigen::MatrixXd getJacobian() const;
    Eigen::Vector3d doIK(float x, float y, float z) const;
    Eigen::Vector3d doFK() const;

    void setAngs(float coxa, float femur, float tibia);
    void setAngs(const Eigen::Vector3d& angs);
    void moveToPos(float x, float y, float z);
    void moveToPos(const Eigen::Vector3d& pos);

    void moveToZero();
    void moveToBasic();
    void moveToOff();

    void doJacobianTest(const int &style);
    void doIKTest();

    int id;
    Eigen::Vector3d pos;
    Eigen::Vector3d currentAngles;
    Eigen::Vector3d currentAngularVelocities;

private:
    float rsStep;
    void sendAngs();
    void sendPos(float x, float y, float z);
    
    RSTimedLoop& rsLoop;

    bool simulationMode;

    unique_ptr<RaisimSimulator> simulator;
};

#endif