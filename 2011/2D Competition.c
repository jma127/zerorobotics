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
static float radiansToDegrees (float radians); //DECL::PROC::radiansToDegrees
static float degreesToRadians (float degrees); //DECL::PROC::degreesToRadians
static float degreesToTarget (float * curAtt, float * targetAtt); //DECL::PROC::degreesToTarget
static void shoot (float * myState, float * targetPos, int type); //DECL::PROC::shoot
static void attToTarget (float * curPos, float * targetPos, float * att); //DECL::PROC::attToTarget
static float disToTarget (float * curPos, float * targetPos); //DECL::PROC::disToTarget
static void overshoot (float * myState, float * targetPos, float amt); //DECL::PROC::overshoot
static float radiansToTarget (float * curAtt, float * targetAtt); //DECL::PROC::radiansToTarget
static void spin (float * myState, float * center, float angVelocityRadians); //DECL::PROC::spin
static void revolve (float * myState, float * center, float radius, float angVelocityRadians); //DECL::PROC::revolve
static unsigned char getMessage (); //DECL::PROC::getMessage

void ZRUser01()
{
    //BEGIN::PROC::ZRUser
    //Jerry M
    //Initialization
    float shieldPos[3];
    float disruptorPos[3];
    float laserPos[3];
    float indigenPos[3];
    float opulenPos[3];
    float s1[3];
    float s2[3];
    float origin[3];
    float c1;
    float c2;
    float c3;
    shieldPos[0] = 0;
    shieldPos[1] = 0.4;
    shieldPos[2] = 0;
    disruptorPos[0] = 0;
    disruptorPos[1] = 0.2;
    disruptorPos[2] = 0;
    laserPos[1] = 0;
    laserPos[2] = 0;
    indigenPos[0] = 0;
    indigenPos[1] = 0.6;
    indigenPos[2] = 0;
    opulenPos[0] = 0;
    opulenPos[1] = -0.6;
    opulenPos[2] = 0;
    s1[0] = -0.6;
    s1[1] = 0;
    s1[2] = 0;
    s2[0] = 0.6;
    s2[1] = 0;
    s2[2] = 0;
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
    c1 = 60;
    c2 = 96;
    c3 = 146;
#define go() (disToTarget(otherState, indigenPos) < 0.5 ? overshoot(myState, origin, 0.08) : overshoot(myState, indigenPos, 0.08))
    //Which sphere & position of laser
    if (time == 0)
        whichSphere = (myState[0] > 0 ? 1 : 2);
    laserPos[0] = (whichSphere == 1 ? 0.4 : -0.4);
    //Print Console
    DEBUG(("\n\n"));
    DEBUG(("DEVILTECH CONSOLE: SPH%d @ TIME %.0f SCORE %.3f FUEL %.3f MESSAGE %d\n", whichSphere, time, PgetScore(), PgetPercentFuelRemaining(), getMessage()));
    DEBUG(("%-10s %-10.3f %-10.3f %-10.3f\n", "POSITION", myState[0], myState[1], myState[2]));
    DEBUG(("%-10s %-10.3f %-10.3f %-10.3f\n", "VELOCITY", myState[3], myState[4], myState[5]));
    DEBUG(("%-10s %-10.3f %-10.3f %-10.3f\n", "ATTITUDE", myState[6], myState[7], myState[8]));
    DEBUG(("%-10s %-10.3f %-10.3f %-10.3f\n", "ANGVEL", myState[9], myState[10], myState[11]));
    DEBUG(("%-10s %-10.3f %-10.3f\n", "SCORE", PgetScore(), PgetOtherScore()));
    DEBUG(("%-10s %-10.3f %-10.3f\n", "S1 DIS", disToTarget(myState, s1), disToTarget(otherState, s1)));
    DEBUG(("%-10s %-10.3f %-10.3f\n", "S2 DIS", disToTarget(myState, s2), disToTarget(otherState, s2)));
    DEBUG(("%-10s %-10s %-10s\n", "SPIN", (PinAsteroid(myState) ? (PinAsteroid(myState) == 1 ? "OPULEN" : "INDIGEN") : "NONE"), (PinAsteroid(otherState) ? (PinAsteroid(otherState) == 1 ? "OPULEN" : "INDIGEN") : "NONE")));
    DEBUG(("%-10s %-10s %-10s\n", "REVOLVE", (PisRevolving(myState) ? (PisRevolving(myState) == 1 ? "OPULEN" : "INDIGEN") : "NONE"), (PisRevolving(otherState) ? (PisRevolving(otherState) == 1 ? "OPULEN" : "INDIGEN") : "NONE")));
    //Fuel Low
    if (PgetPercentFuelRemaining() > 0.8) {
        //Items
        if (time < c1) {
            int repel = 1;
            if (!PhaveLaser())
                overshoot(myState, laserPos, 0.08);
            else if (!PdisruptorUpgraded() && !PotherDisruptorUpgraded() && PgetPercentFuelRemaining() > 75.0)
                overshoot(myState, disruptorPos, 0.08);
            else if (!PhaveShield() && !PotherHasShield() && PgetPercentFuelRemaining() > 75.0)
                overshoot(myState, shieldPos, 0.08);
            else {
                float att[3];
                repel = 0;
                attToTarget(myState, opulenPos, att);
                ZRSetAttitudeTarget(att);
                go();
            }
            if (repel && disToTarget(myState, otherState) < 0.5 && PgetCharge() > 19)
                shoot(myState, otherState, 2);
            PsendMessage(1);
        }
        //Melt Ice
        else if (time < c2 && PiceHits() < 15 && !PiceMelted() && PgetCharge()) {
            shoot(myState, opulenPos, 1);
            go();
            PsendMessage(1);
        }
        //Indigen
        else if (!PiceMelted() && time < c2) {
            if (getMessage() == 4 || PinAsteroid(otherState) == 2) {
                revolve(myState, indigenPos, 0.45, degreesToRadians(4));
                PsendMessage(5);
            }
            else {
                spin(myState, indigenPos, degreesToRadians(30));
                PsendMessage(4);
            }
        }
        //Opulen
        else if (time < c3) {
            if (getMessage() == 2 || PinAsteroid(otherState) == 1 || (disToTarget(otherState, opulenPos) < 0.25)) {
                revolve(myState, opulenPos, 0.45, degreesToRadians(4));
                PsendMessage(3);
            }
            else {
                spin(myState, opulenPos, degreesToRadians(30));
                PsendMessage(2);
            }
        }
        //Race/Shoot
        else {
            float dis1 = disToTarget(myState, s1), dis2 = disToTarget(myState, s2), oDis1 = disToTarget(otherState, s1), oDis2 = disToTarget(otherState, s2);
            if ((dis1 <= oDis1 && dis2 <= oDis2) || (dis1 > oDis1 && dis2 > oDis2)) {
                if (dis1 < dis2)
                    overshoot(myState, s1, 0.08);
                else
                    overshoot(myState, s2, 0.08);
            }
            else if (dis1 <= oDis1)
                overshoot(myState, s1, 0.08);
            else
                overshoot(myState, s2, 0.08);
            if (!PatMiningStation()) {
                if (otherState[0] < 0) {
                    if (dis1 < oDis1)
                        shoot(myState, otherState, 2);
                    else
                        shoot(myState, otherState, 3);
                }
                else {
                    if (dis2 < oDis2)
                        shoot(myState, otherState, 2);
                    else
                        shoot(myState, otherState, 3);
                }
            }
        }
    }
    //END::PROC::ZRUser
}
void ZRInit01()
{
    //BEGIN::PROC::ZRInit
    whichSphere = 0;
    //END::PROC::ZRInit
}
//User-defined procedures
static float radiansToDegrees (float radians)
{
    //BEGIN::PROC::radiansToDegrees
    //Jerry M
    return radians * 180.0 / 3.14159;
    //END::PROC::radiansToDegrees
}
static float degreesToRadians (float degrees)
{
    //BEGIN::PROC::degreesToRadians
    //Jerry M
    return degrees * 3.14159 / 180.0;
    //END::PROC::degreesToRadians
}
static float degreesToTarget (float * curAtt, float * targetAtt)
{
    //BEGIN::PROC::degreesToTarget
    //Jerry M
    return radiansToDegrees(radiansToTarget(curAtt, targetAtt));
    //END::PROC::degreesToTarget
}
static void shoot (float * myState, float * targetPos, int type)
{
    //BEGIN::PROC::shoot
    //Jerry M
    float att[3];
    DEBUG(("SHOOT\n"));
    if (!PgetCharge())
        return;
    attToTarget(myState, targetPos, att);
    ZRSetAttitudeTarget(att);
    if (degreesToTarget(myState + 6, att) < 5.9) {
        if (type == 1 || (PotherHasShield() && PgetShieldStrength()))
            Plaser();
        else if (type == 2)
            Prepulsor();
        else if (type == 3)
            Ptractor();
    }
    DEBUG(("SUCCESSFUL\n"));
    //END::PROC::shoot
}
static void attToTarget (float * curPos, float * targetPos, float * att)
{
    //BEGIN::PROC::attToTarget
    //Jerry M
    mathVecSubtract(att, targetPos, curPos, 3);
    mathVecNormalize(att, 3);
    //END::PROC::attToTarget
}
static float disToTarget (float * curPos, float * targetPos)
{
    //BEGIN::PROC::disToTarget
    //Jerry M
    float newVec[3];
    mathVecSubtract(newVec, targetPos, curPos, 3);
    return mathVecMagnitude(newVec, 3);
    //END::PROC::disToTarget
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
    DEBUG(("SUCCESSFUL\n"));
    //END::PROC::overshoot
}
static float radiansToTarget (float * curAtt, float * targetAtt)
{
    //BEGIN::PROC::radiansToTarget
    //Jerry M
    return acos(mathVecInner(curAtt, targetAtt, 3) / mathVecMagnitude(curAtt, 3) / mathVecMagnitude(targetAtt, 3));
    //END::PROC::radiansToTarget
}
static void spin (float * myState, float * center, float angVelocityRadians)
{
    //BEGIN::PROC::spin
    //Jerry M
#define radiansPerNM 20.5;
    DEBUG(("SPIN\n"));
    if (disToTarget(myState, center) < 0.05) {
        float torques[3] = {0, 0, 0};
        torques[2] = (angVelocityRadians - myState[11]) / radiansPerNM;
        ZRSetTorques(torques);
    }
    overshoot(myState, center, 0.1);
    DEBUG(("SUCCESSFUL\n"));
    //END::PROC::spin
}
static void revolve (float * myState, float * center, float radius, float angVelocityRadians)
{
    //BEGIN::PROC::revolve
    float newPos[3], att[3], dis = disToTarget(myState, center), mag = radius * angVelocityRadians;
    DEBUG(("REVOLVE\n"));
    attToTarget(center, myState, att);
    newPos[0] = center[0] + att[0] * radius;
    newPos[1] = center[1] + att[1] * radius;
    newPos[2] = center[2] + att[2] * radius;
    if (fabs(dis - radius) > 0.045)
        overshoot(myState, newPos, 0.045);
    else {
        newPos[0] = att[1] * mag;
        newPos[1] = -att[0] * mag;
        ZRSetVelocityTarget(newPos);
    }
    DEBUG(("ROTATING AT VELOCITY %.3f\n", radiansToDegrees(atan(mathVecMagnitude(myState + 3, 3) / dis))));
    //END::PROC::revolve
}
static unsigned char getMessage ()
{
    //BEGIN::PROC::getMessage
    //Jerry M
    unsigned short message = PgetMessage();
    unsigned short revolve = (message >> 15) & 1U, opulen = (message >> 14) & 1U;
    if (message < 16)
        return message;
    if (!revolve && opulen)
        return 2;
    if (revolve && opulen)
        return 3;
    if (!revolve && !opulen)
        return 4;
    return 5;
    //END::PROC::getMessage
}
