#include <string.h>
#include <math.h>
#include "ZRGame.h"
#include "ZR_API.h"
#include "spheres_types.h"
#include "spheres_constants.h"
#include "ctrl_attitude.h"
#include "ctrl_position.h"
#include "find_state_error.h"
#include "math_matrix.h"


#ifdef ZRSIMULATION
extern void _init(void);
void *_start = &_init;
#endif

#undef ZRSIMULATION

static int whichSphere; //DECL::VAR::whichSphere
static float prevScore; //DECL::VAR::prevScore
static float disToTarget (float * curPos, float * targetPos); //DECL::PROC::disToTarget
static void attToTarget (float * curPos, float * targetPos, float * att); //DECL::PROC::attToTarget
static float radiansToTarget (float * curAtt, float * targetAtt); //DECL::PROC::radiansToTarget
static void vecToTarget (float * curPos, float * targetPos, float * att); //DECL::PROC::vecToTarget
static void overshoot (float * myState, float * targetPos, float amt); //DECL::PROC::overshoot
static void doRevolve (float * myState, float * center, float * axis, float radius, float angVelocityRadians, int go); //DECL::PROC::doRevolve
static void shoot (float * myState, float * targetPos, int type); //DECL::PROC::shoot
static void spin (float * myState, float * center, float * axis, float angVelocityRadians); //DECL::PROC::spin

void ZRUser01()
{
    //BEGIN::PROC::ZRUser
    //Jerry M
    //Useful Variables
#define radToDeg 57.29577
#define opSpin 2U
#define opRev 3U
#define inSpin 4U
#define inRev 5U
#define revRad 0.334
#define spinDis 0.0335
#define p1 59.5
#define spinRate 30.06
#define revRate 3.868
#define lowFuel 1.0
#define startRev1 26
#define startRev2 22
#define startShoot 0.0155
#define enoughShots 9
    float shieldPos[3];
    float disruptorPos[3];
    float laserPos[3];
    float indigenPos[3];
    float opulenPos[3];
    float axis[3];
    float s1[3];
    float s2[3];
    float spinApproach[3];
    float zero[3];
    //Message Variables
    unsigned short message;
    unsigned short command;
    unsigned short toSend;
    unsigned short bits[17];
    unsigned short sentBits[17];
    int count;
    //Initialization
    shieldPos[0] = 0;
    shieldPos[1] = 0;
    shieldPos[2] = 0.4;
    disruptorPos[0] = 0;
    disruptorPos[1] = 0;
    disruptorPos[2] = -0.4;
    laserPos[1] = 0;
    laserPos[2] = 0;
    indigenPos[0] = 0;
    indigenPos[1] = 0.35;
    indigenPos[2] = 0.2;
    opulenPos[0] = 0;
    opulenPos[1] = -0.35;
    opulenPos[2] = -0.2;
    PgetAsteroidNormal(axis);
    mathVecNormalize(axis, 3);
    s1[0] = 0.6;
    s1[1] = 0;
    s1[2] = -0.5;
    s2[0] = -0.6;
    s2[1] = 0;
    s2[2] = 0.5;
    spinApproach[0] = opulenPos[0] - spinDis * axis[0];
    spinApproach[1] = opulenPos[1] - spinDis * axis[1];
    spinApproach[2] = opulenPos[2] - spinDis * axis[2];
    zero[0] = 0;
    zero[1] = 0;
    zero[2] = 0;
    //Which sphere & position of laser
    if (time < 1.5)
        whichSphere = (myState[0] > 0.0 ? 1 : 2);
    laserPos[0] = (whichSphere == 1 ? 0.4 : -0.4);
    //Process Message
    message = PgetMessage();
    for (count = 0; count < 16; count ++)
        bits[count + 1] = (message >> count) & 1U;
    if (time > 2.5) {
        if (bits[10] || bits[11]) { //yo
            toSend = 1024U;
            if (whichSphere == 1 || bits[10])
                command = opRev;
            else
                command = opSpin;
        }
        else if (bits[12] || bits[13] || bits[14] || bits[15] || bits[16]) { //delta
            toSend = 24576U;
            if (whichSphere == 1 || !bits[16] || !bits[15]) {
                command = opRev;
                toSend = toSend | 32768U;
            }
            else
                command = opSpin;
        }
        else { //normal
            if (whichSphere == 2 && message == opRev)
                command = toSend = opSpin;
            else
                command = toSend = opRev;
        }
    }
    else {
        toSend = 1024U;
        command = opRev;
    }
    for (count = 0; count < 16; count ++)
        sentBits[count + 1] = (toSend >> count) & 1U;
    PsendMessage(toSend);
    //Print Console
    DEBUG(("\n\n"));
    DEBUG(("DEVILTECH CONSOLE: SPH%d @ TIME %.0f FUEL %.3f\n", whichSphere, time, PgetPercentFuelRemaining()));
    DEBUG(("SCORE %.3f PER TURN %.3f\n", PgetScore(), PgetScore() - prevScore));
    DEBUG(("MESSAGE %u COMMAND %u SENT %u\n", message, command, toSend));
    for (count = 1; count <= 16; count ++)
        DEBUG(("%3d", count));
    DEBUG(("\n"));
    for (count = 1; count <= 16; count ++)
        DEBUG(("%3u", bits[count]));
    DEBUG(("\n"));
    for (count = 1; count <= 16; count ++)
        DEBUG(("%3u", sentBits[count]));
    DEBUG(("\n"));
    DEBUG(("%-10s %-10.3f %-10.3f %-10.3f\n", "POSITION", myState[0], myState[1], myState[2]));
    DEBUG(("%-10s %-10.3f %-10.3f %-10.3f\n", "VELOCITY", myState[3], myState[4], myState[5]));
    DEBUG(("%-10s %-10.3f %-10.3f %-10.3f\n", "ATTITUDE", myState[6], myState[7], myState[8]));
    DEBUG(("%-10s %-10.3f %-10.3f %-10.3f\n", "ANGVEL", myState[9], myState[10], myState[11]));
    DEBUG(("%-10s %-10s %-10s\n", "SPIN", (PinAsteroid(myState) ? (PinAsteroid(myState) == 1 ? "OPULEN" : "INDIGEN") : "NONE"), (PinAsteroid(otherState) ? (PinAsteroid(otherState) == 1 ? "OPULEN" : "INDIGEN") : "NONE")));
    DEBUG(("%-10s %-10s %-10s\n", "REVOLVE", (PisRevolving(myState) ? (PisRevolving(myState) == 1 ? "OPULEN" : "INDIGEN") : "NONE"), (PisRevolving(otherState) ? (PisRevolving(otherState) == 1 ? "OPULEN" : "INDIGEN") : "NONE")));
    DEBUG(("%-10s %-10d %-10d\n", "LASERS", PiceHits(), PotherIceHits()));
    prevScore = PgetScore();
    //Low Fuel
    if (PgetPercentFuelRemaining() > lowFuel) {
        if (time < p1) {
            if (!PhaveLaser())
                overshoot(myState, laserPos, 0.1);
            else {
                if (command == opSpin) {
                    if (disToTarget(myState, spinApproach) > startShoot)
                        overshoot(myState, spinApproach, 0.1);
                    else
                        ZRSetVelocityTarget(zero);
                }
                else
                    doRevolve(myState, opulenPos, axis, revRad, 0, 0);
                shoot(myState, opulenPos, 0);
            }
        }
        else if (!PiceMelted() && PgetCharge()) {
            if (command == opSpin)
                ZRSetVelocityTarget(zero);
            else if ((PotherIceHits() > enoughShots && PiceHits() + PotherIceHits() < startRev2) || (PotherIceHits() <= enoughShots && PiceHits() + PotherIceHits() < startRev1))
                doRevolve(myState, opulenPos, axis, revRad, 0, 0);
            else
                doRevolve(myState, opulenPos, axis, revRad, revRate / radToDeg, 1);
            shoot(myState, opulenPos, 1);
        }
        else {
            if (command == opSpin)
                spin(myState, opulenPos, axis, spinRate / radToDeg);
            else
                doRevolve(myState, opulenPos, axis, revRad, revRate / radToDeg, 1);
        }
    }
    //END::PROC::ZRUser
}
void ZRInit01()
{
    //BEGIN::PROC::ZRInit
    whichSphere = 0;
    prevScore = 0;
    //END::PROC::ZRInit
}
//User-defined procedures
static float disToTarget (float * curPos, float * targetPos)
{
    //BEGIN::PROC::disToTarget
    //Jerry M
    float newVec[3];
    mathVecSubtract(newVec, targetPos, curPos, 3);
    return mathVecMagnitude(newVec, 3);
    //END::PROC::disToTarget
}
static void attToTarget (float * curPos, float * targetPos, float * att)
{
    //BEGIN::PROC::attToTarget
    //Jerry M
    mathVecSubtract(att, targetPos, curPos, 3);
    mathVecNormalize(att, 3);
    //END::PROC::attToTarget
}
static float radiansToTarget (float * curAtt, float * targetAtt)
{
    //BEGIN::PROC::radiansToTarget
    //Jerry M
    return acos(mathVecInner(curAtt, targetAtt, 3) / mathVecMagnitude(curAtt, 3) / mathVecMagnitude(targetAtt, 3));
    //END::PROC::radiansToTarget
}
static void vecToTarget (float * curPos, float * targetPos, float * att)
{
    //BEGIN::PROC::vecToTarget
    //Jerry M
    mathVecSubtract(att, targetPos, curPos, 3);
    //END::PROC::vecToTarget
}
static void overshoot (float * myState, float * targetPos, float amt)
{
    //BEGIN::PROC::overshoot
    //Jerry M
    float newLoc[3], att[3];
    DEBUG(("OVERSHOOT\n"));
    attToTarget(myState, targetPos, att);
    newLoc[0] = targetPos[0] + att[0] * amt;
    newLoc[1] = targetPos[1] + att[1] * amt;
    newLoc[2] = targetPos[2] + att[2] * amt;
    if (disToTarget(myState, targetPos) < amt)
        ZRSetPositionTarget(targetPos);
    else
        ZRSetPositionTarget(newLoc);
    //END::PROC::overshoot
}
static void doRevolve (float * myState, float * center, float * axis, float radius, float angVelocityRadians, int go)
{
    //BEGIN::PROC::doRevolve
    //Jerry M
#define radToDeg 57.29577
    float vec[3], perp[3], newVel[3], final[3], dis, diff;
    DEBUG(("REVOLVE\n"));
    vecToTarget(center, myState, vec);
    mathVecCross(newVel, vec, axis);
    mathVecCross(perp, newVel, axis);
    dis = mathVecMagnitude(perp, 3);
    diff = dis - radius;
    DEBUG(("DIS %.3f\n", dis));
    if (go) {
        mathVecNormalize(perp, 3);
        final[0] = perp[0] * diff + newVel[0] * angVelocityRadians;
        final[1] = perp[1] * diff + newVel[1] * angVelocityRadians;
        final[2] = perp[2] * diff + newVel[2] * angVelocityRadians;
        DEBUG(("CUR ANGVEL %.3f\n", mathVecMagnitude(myState + 3, 3) / dis * radToDeg));
        DEBUG(("FVL %.3f %.3f %.3f\n", final[0], final[1], final[2]));
        ZRSetVelocityTarget(final);
        mathVecNormalize(perp, 3);
        ZRSetAttitudeTarget(perp);
    }
    else {
        mathVecNormalize(perp, 3);
        perp[0] *= radius;
        perp[1] *= radius;
        perp[2] *= radius;
        mathVecSubtract(final, center, perp, 3);
        DEBUG(("POS %.3f %.3f %.3f\n", final[0], final[1], final[2]));
        overshoot(myState, final, 0.1);
    }
    //END::PROC::doRevolve
}
static void shoot (float * myState, float * targetPos, int type)
{
    //BEGIN::PROC::shoot
    //Jerry M
#define radToDeg 57.29577
    float att[3];
    DEBUG(("SHOOT\n"));
    if (!PgetCharge())
        return;
    attToTarget(myState, targetPos, att);
    DEBUG(("TARGETED %.3f %.3f %.3f\n", att[0], att[1], att[2]));
    ZRSetAttitudeTarget(att);
    if (radiansToTarget(myState + 6, att) * radToDeg < 5.985) {
        if (type == 0);
        else if (type == 1 || (PotherHasShield() && PgetShieldStrength()))
            Plaser();
        else if (type == 2)
            Prepulsor();
        else if (type == 3)
            Ptractor();
    }
    //END::PROC::shoot
}
static void spin (float * myState, float * center, float * axis, float angVelocityRadians)
{
    //BEGIN::PROC::spin
    //Jerry M
#define radiansPerNM 20.1
#define radToDeg 57.29577
    float torques[3];
    float diff = radiansToTarget(myState + 6, axis);
    float newTarget = angVelocityRadians / cosf(diff);
    DEBUG(("SPIN\n"));
    DEBUG(("DIFF %.3f OLD %.3f NEW %.3f CURVEL %.3f\n", diff * radToDeg, angVelocityRadians * radToDeg, newTarget * radToDeg, myState[9] * radToDeg));
    overshoot(myState, center, 0.02);
    torques[0] = (newTarget - myState[9]) / radiansPerNM;
    if (diff > 60 / radToDeg)
        ZRSetAttitudeTarget(axis);
    torques[1] = torques[2] = 0.0;
    ZRSetTorques(torques);
    //END::PROC::spin
}
