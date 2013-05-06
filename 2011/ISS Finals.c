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

static float otherPrevScore; //DECL::VAR::otherPrevScore
static char ms; //DECL::VAR::ms
static char sph1; //DECL::VAR::sph1
static float prevScore; //DECL::VAR::prevScore
static void vecToTarget (float curPos[3], float targetPos[3], float vec[3]); //DECL::PROC::vecToTarget
static void attToTarget (float curPos[3], float targetPos[3], float att[3]); //DECL::PROC::attToTarget
static float disToTarget (float curPos[3], float targetPos[3]); //DECL::PROC::disToTarget
static void overshoot (float myState[12], float targetPos[3], float amt); //DECL::PROC::overshoot
static void shoot (float myState[12], float targetPos[3], unsigned char type); //DECL::PROC::shoot
static void stationshoot (float myState[12], float targetPos[3], float dis, float mag); //DECL::PROC::stationshoot
static char whichMS (float time1, float time2, float oTime1, float oTime2, float time); //DECL::PROC::whichMS
static void spin (float myState[12], float center[3], float axis[3], float angVelocityRadians); //DECL::PROC::spin
static void doRevolve (float myState[12], float center[3], float axis[3], float radius, float angVelocityRadians, unsigned char go); //DECL::PROC::doRevolve
static float timeToMS (float myState[6], float Station[3]); //DECL::PROC::timeToMS

void ZRUser01()
{
    //BEGIN::PROC::ZRUser
    /**
     * Purpose: Commands the SPHERE to play AsteroSPHERES
     * Parameters:
     *   float myState[12]: current state of SPHERE
     *   float otherState[12]: current state of other SPHERE
     *   float time: current time in the game
     */
    
#define MIN(a,b)   (((a) < (b)) ? (a) : (b))
#define MAX(a,b)   (((a) > (b)) ? (a) : (b))
    
    //Useful Variables
#define radToDeg 57.29577f //multiply with radians to get degrees
#define opSpin 2U //command and normal protocol message for spinning on opulens
#define opRev 3U //similar
#define inSpin 4U //similar
#define inRev 5U //similar
#define revRad 0.334f //radius for revolving
#define spinDis 0.0335f //if spinning this match, how far from asteroid to laser it
#define p1 59.5f //before this, it's phase 1
#define p2 119.5f //before this, it's phase 2
#define raceTime 153.5f //before this, score
#define stopOtherTime 159.5f //if they are scoring after this, repel them
#define stopValue 0.008f //points per second to be considered "scoring"
#define tooClose 0.6f //too close, signal earth
#define spinRate 30.04f //rate in degrees at which we want to spin
#define revRate 3.88f //rate in degrees at which we want to revolve
#define lowFuel 1.5f //below this, we want to stop
#define startRev1 26 //if only we are lasering, revolve after this many ice hits
#define startRev2 22 //if both are lasering, revolve after this many ice hits
#define stopApproach 0.0155f //if within this distance of spin approach position, stop
#define enoughShots 9 //if other ice hits is above this, they are trying to laser
#define danger 3.0f //if we're beating other sphere by less than this distance in race, shoot
#define printBasic 1 //boolean determines whether we print basic console
#define printTimes 1 //boolean determines printing of times
#define printDebug 0 //boolean determines printing of uberconsole
#define bit7 64U
#define bit8 128U
#define bit9 256U
#define benefit 3.2f
    //#define dot(a,b)    mathVecInner(a, b, 3) / mathVecMagnitude(a, 3) / mathVecMagnitude (b, 3)
    float laserPos[3]; //our laser position
    float opulenPos[3]; //opulen position
    float axis[3]; //asteroid normal axis attitude
    float s1[3]; //station 1 position
    float s2[3]; //station 2 position
    float spinApproach[3]; //spin approach position
    float zero[3]; //array of zeros
    float earth[3];
    //float temp[3]; //temporary array
    //float dis1; //us to s1
    //float dis2; //us to s2
    //float oDis1; //opponent to s1
    //float oDis2; //opponent to s2
    //float oDeg1; //opponent degree between velocity and direction to s1
    //float oDeg2; //opponent degree between velocity and direction to s2
    //float oDot1; //opponent cosine of angle between velocity and direction to s1
    //float oDot2; //opponent cosine of angle between velocity and direction to s2
    float time1, time2, oTime1, oTime2; //times to MS1 and MS2
    float oScore; //other's score
    float oScoreDeriv; //change in other's score
    //unsigned char go1; //do we want to go to s1?
    //unsigned char oGo1; //is the other sphere going towards s1?
    unsigned char type; //if racing, which type of beam are we shooting?
    //Message Variables
    unsigned short message; //message we received
    unsigned short command; //command we will execute
    unsigned short toSend; //message we will send
    unsigned short bits[17]; //bitmap of received bits
    unsigned char count; //for-loop iterator
    //Variable Initialization
    opulenPos[0] = 0.0f;
    opulenPos[1] = -0.35f;
    opulenPos[2] = -0.2f;
    PgetAsteroidNormal(axis);
    mathVecNormalize(axis, 3);
    s1[0] = 0.5f;
    s1[1] = -0.31f;
    s1[2] = 0.55f;
    s2[0] = -0.5f;
    s2[1] = 0.31f;
    s2[2] = -0.55f;
    for (count = 0; count < 3; count ++) {
        spinApproach[count] = opulenPos[count] - spinDis * axis[count];
        laserPos[count] = zero[count] = earth[count] = 0.0f;
    }
    earth[1] = 10000000.0f;
    time1 = timeToMS(myState, s1);
    time2 = timeToMS(myState, s2);
    oTime1 = timeToMS(otherState, s1);
    oTime2 = timeToMS(otherState, s2);
    type = 0;
    oScore = PgetOtherScore();
    oScoreDeriv = oScore - otherPrevScore;
    //Which sphere & position of laser
    if (time < 1.5)
        sph1 = (myState[0] > 0.0f);
    laserPos[0] = (sph1 ? 0.4f : -0.4f);
    //Process Message
    message = PgetMessage();
    command = opSpin;
    toSend = 1024U;
    for (count = 0; count < 16; count ++) //process bits
        bits[count + 1] = (message >> count) & 1U;
    if (time > 2.5f) { //Identify protocol and command
        if (bits[10] || bits[11]) { //yo
            if (sph1 || bits[10])
                command = opRev;
        }
        else if (bits[14] || bits[15] || bits[16]) { //delta
            toSend = 24576U;
            if (sph1 || !bits[16] || !bits[15]) {
                toSend = 57334U;
                command = opRev;
            }
        }
        else { //default
            toSend = opSpin;
            if (sph1 || message != opRev)
                command = toSend = opRev;
        }
        if (bits[10] || bits[11] || bits[14] || bits[15] || bits[16]) {
            toSend = toSend | bit7;
            if (sph1) {
                if (ms == 1)
                    toSend = toSend | bit8;
                if (ms == 2)
                    toSend = toSend | bit9;
            }
            else if (bits[7])
                toSend = (toSend | bit8) | bit9;
        }
    }
    PsendMessage(toSend); //send message
    //Analyze for times
    if (!sph1 && bits[7]) {
        if (bits[8] && !bits[9])
            oTime1 -= benefit;
        if (!bits[8] && bits[9])
            oTime2 -= benefit;
    }
    //Gameplay
    if (PgetPercentFuelRemaining() > lowFuel) { //enough fuel to continue
        if (!PhaveLaser() && time < p1) //get the laser
            stationshoot(myState, laserPos, 0.1f, 0.03f);
        else if (!PiceMelted() && PgetCharge() && time < p2) { //prepare to/start melting
            if (command == opSpin) { //start the spin approach
                if (disToTarget(myState, spinApproach) > stopApproach && time < p1) //should approach
                    stationshoot(myState, spinApproach, 0.1f, 0.03f);
                else //should stop to get ready for lasering
                    ZRSetVelocityTarget(zero);
            }
            else if ((PotherIceHits() > enoughShots && PiceHits() + PotherIceHits() < startRev2) || (PotherIceHits() <= enoughShots && PiceHits() + PotherIceHits() < startRev1)) //start the revolve approach
                doRevolve(myState, opulenPos, axis, revRad, 0.0f, 0);
            else //revolve while lasering to maximise points
                doRevolve(myState, opulenPos, axis, revRad, revRate / radToDeg, 1);
            if (time < p1) //prepare to shoot
                shoot(myState, opulenPos, 0);
            else //start shooting
                shoot(myState, opulenPos, 1);
        }
        else{
            if (((time1 + time >= 169.0f) || (time2 + time >= 169.0f)) && !ms) //time to decide whether or not to race!
                ms = whichMS(time1, time2, oTime1, oTime2, time);
            if (!PatMiningStation() && ms) {
                /*
                 if (((ms % 2) && (time1 + danger > oTime2) && (oTime2 < oTime1)) || (!(ms%2) && (time2 + danger > oTime1) && (oTime1 < oTime2))) //tractor
                 type = 3;
                 if (time > stopOtherTime && oScoreDeriv > stopValue) //repel
                 type = 2;
                 shoot(myState, otherState, type);
                 */
                shoot(myState, earth, 0);
                ZRSetPositionTarget((ms % 2) ? s1 : s2);
            }
            else if (ms) {
                ZRSetVelocityTarget(zero);
                if (oScoreDeriv > stopValue || disToTarget(myState, otherState) < tooClose)
                    shoot(myState, earth, 1);
            }
            else{
                if (command == opSpin)
                    spin(myState, opulenPos, axis, spinRate / radToDeg);
                else {
                    doRevolve(myState, opulenPos, axis, revRad, revRate / radToDeg, 1);
                    shoot(myState, otherState, 0);
                }
            }
        }
    }
    else //low fuel, stop moving to avoid penalty
        ZRSetVelocityTarget(zero);
    //Print Console
#if printBasic
    DEBUG(("\n\n"));
    DEBUG(("DEVILTECH CONSOLE: SPH%d @ TIME %.0f FUEL %.4f\n", 2 - sph1, time, PgetPercentFuelRemaining()));
    DEBUG(("SCORE %.4f PER TURN %.4f\n", PgetScore(), PgetScore() - prevScore));
    DEBUG(("OSCORE %.4f PER TURN %.4f\n", oScore, oScoreDeriv));
    DEBUG(("MESSAGE %u SENT %u\n", message, toSend));
    DEBUG(("%-10s %-10.4f %-10.4f %-10.4f\n", "POSITION", myState[0], myState[1], myState[2]));
    DEBUG(("%-10s %-10.4f %-10.4f %-10.4f\n", "VELOCITY", myState[3], myState[4], myState[5]));
    DEBUG(("%-10s %-10.4f %-10.4f %-10.4f\n", "ATTITUDE", myState[6], myState[7], myState[8]));
    DEBUG(("%-10s %-10.4f %-10.4f %-10.4f\n", "ANGVEL", myState[9], myState[10], myState[11]));
    prevScore = PgetScore(); //set previous score
#endif
#if printTimes
    DEBUG(("%-10.4f %-10.4f\n", time1, oTime1));
    DEBUG(("%-10.4f %-10.4f\n", time2, oTime2));
    DEBUG(("%-10d\n", ms + 9231));
#endif
#if printDebug
    DEBUG(("COMMAND %u\n", command));
    for (count = 1; count <= 16; count ++)
        DEBUG(("%3d", count));
    DEBUG(("\n"));
    DEBUG(("RECEIVED\n"));
    for (count = 1; count <= 16; count ++)
        DEBUG(("%3u", bits[count]));
    DEBUG(("\n"));
    for (count = 0; count < 16; count ++) //generate bitmap of sent message
        bits[count + 1] = (toSend >> count) & 1U;
    DEBUG(("SENT\n"));
    for (count = 1; count <= 16; count ++)
        DEBUG(("%3u", bits[count]));
    DEBUG(("\n"));
    DEBUG(("%-10s %-10s %-10s\n", "SPIN", (PinAsteroid(myState) ? (PinAsteroid(myState) == 1 ? "OPULEN" : "INDIGEN") : "NONE"), (PinAsteroid(otherState) ? (PinAsteroid(otherState) == 1 ? "OPULEN" : "INDIGEN") : "NONE")));
    DEBUG(("%-10s %-10s %-10s\n", "REVOLVE", (PisRevolving(myState) ? (PisRevolving(myState) == 1 ? "OPULEN" : "INDIGEN") : "NONE"), (PisRevolving(otherState) ? (PisRevolving(otherState) == 1 ? "OPULEN" : "INDIGEN") : "NONE")));
    DEBUG(("%-10s %-10d %-10d\n", "LASERS", PiceHits(), PotherIceHits()));
    DEBUG(("%-10s %-10s\n", "TYPE", (type ? (type == 2 ? "REPEL" : "TRACTOR") : "NONE")));
#endif
    otherPrevScore = oScore; //set other prev score
    //END::PROC::ZRUser
}
void ZRInit01()
{
    //BEGIN::PROC::ZRInit
    otherPrevScore = 0;
    ms = 0;
    sph1 = 0;
    prevScore = 0.0f;
    //END::PROC::ZRInit
}
//User-defined procedures
static void vecToTarget (float curPos[3], float targetPos[3], float vec[3])
{
    //BEGIN::PROC::vecToTarget
    /**
     * Purpose: Generates the vector from curPos to targetPos
     * Parameters:
     *   float curPos[3]: current position of SPHERE
     *   float targetPos[3]: position vector will point towards
     *   float vec[3]: array to store vector into
     */
    
    mathVecSubtract(vec, targetPos, curPos, 3);
    //END::PROC::vecToTarget
}
static void attToTarget (float curPos[3], float targetPos[3], float att[3])
{
    //BEGIN::PROC::attToTarget
    /**
     * Purpose: Generates the attitude from curPos to targetPos
     * Parameters:
     *   float curPos[3]: current position of SPHERE
     *   float targetPos[3]: position attitude will point towards
     *   float att[3]: array to store attitude into
     */
    
    mathVecSubtract(att, targetPos, curPos, 3);
    mathVecNormalize(att, 3);
    //END::PROC::attToTarget
}
static float disToTarget (float curPos[3], float targetPos[3])
{
    //BEGIN::PROC::disToTarget
    /**
     * Purpose: Gets the distance between two points
     * Parameters:
     *   float curPos[3]: current position of SPHERE
     *   float targetPos[3]: position to get distance from
     */
    
    float newVec[3]; //stores the vector between two points
    mathVecSubtract(newVec, targetPos, curPos, 3);
    return mathVecMagnitude(newVec, 3);
    //END::PROC::disToTarget
}
static void overshoot (float myState[12], float targetPos[3], float amt)
{
    //BEGIN::PROC::overshoot
    /**
     * Purpose: Faster modification of ZRSetPositionTarget
     * Parameters:
     *   float myState[12]: current state of SPHERE
     *   float targetPos[3]: where we want to go to
     *   float amt: magnitude we want to overshoot
     */
    
    float newLoc[3]; //new location we want to target
    float att[3]; //attitude from SPHERE to target
    attToTarget(myState, targetPos, att);
    newLoc[0] = targetPos[0] + att[0] * amt; //set newLoc beyond target
    newLoc[1] = targetPos[1] + att[1] * amt;
    newLoc[2] = targetPos[2] + att[2] * amt;
    if (disToTarget(myState, targetPos) < amt) //already within amount to overshoot, slow down
        ZRSetPositionTarget(targetPos);
    else //overshoot
        ZRSetPositionTarget(newLoc);
    //END::PROC::overshoot
}
static void shoot (float myState[12], float targetPos[3], unsigned char type)
{
    //BEGIN::PROC::shoot
    /**
     * Purpose: commands the SPHERE to play AsteroSPHERES
     * Parameters:
     *   float myState[12]: current state of SPHERE
     *   float targetPos[3]: position target of beam
     *   int type:
     *     0 if just adjusting attitude
     *     1 if lasering
     *     2 if repulsing
     *     3 if tractoring
     */
    
#define radToDeg 57.29577f //multiply with radians to get degrees
#define cosConeAngle 0.994523f //half of the degree width of shooting cone
    float att[3]; //attitude that we want
    if (!PgetCharge()) //no charge, stop
        return;
    attToTarget(myState, targetPos, att); //get attitude vector
    ZRSetAttitudeTarget(att); //point to attitude vector
    if (mathVecInner(myState + 6, att, 3) > cosConeAngle) { //within range
        if (type == 1) //laser beam
            Plaser();
        else if (type == 2) //repulsor disrupt beam
            Prepulsor();
        else if (type == 3) //tractor disrupt beam
            Ptractor();
    }
    //END::PROC::shoot
}
static void stationshoot (float myState[12], float targetPos[3], float dis, float mag)
{
    //BEGIN::PROC::stationshoot
    /**
     * Purpose: Faster modification of ZRSetPositionTarget
     * Parameters:
     *   float myState[12]: current state of SPHERE
     *   float targetPos[3]: where we want to go to
     *   float dis: distance from which we want to overshoot
     *   float mag: magnitude we want to overshoot
     */
    
    float newLoc[3]; //new location we want to target
    float att[3]; //attitude from SPHERE to target
    attToTarget(myState, targetPos, att);
    newLoc[0] = targetPos[0] + att[0] * mag; //set newLoc beyond target
    newLoc[1] = targetPos[1] + att[1] * mag;
    newLoc[2] = targetPos[2] + att[2] * mag;
    if (disToTarget(myState, targetPos) < dis) //already within amount to overshoot, slow down
        ZRSetPositionTarget(targetPos);
    else //overshoot
        ZRSetPositionTarget(newLoc);
    //END::PROC::stationshoot
}
static char whichMS (float time1, float time2, float oTime1, float oTime2, float time)
{
    //BEGIN::PROC::whichMS
    // time1  - the time it would take us to arrive at MS1 if we were to leave now
    // time2  - the time it would take us to arrive at MS2 if we were to leave now
    // oTime1 - the time it would take the opponent to arrive at MS1 if it were to leave now
    // oTime2 - the time it would take the opponent to arrive at MS2 if it were to leave now
    
#define OC -1.0f // Overshoot Constant: how much more time it takes the opponent to travel than it takes us.
#define MIN(a,b)   (((a) < (b)) ? (a) : (b))
#define MAX(a,b)   (((a) > (b)) ? (a) : (b))
#define MSC 0.5f // MS Constant: max difference between time1 and time2 at which we consider oTime instead of time.
    
    char ans = 0;
    float tmp;
    
    // tmp holds the opponent's time to our closer mining station
    if (time1 < time2) {
        tmp = oTime1 + OC;
    }
    else {
        tmp = oTime2 + OC;
    }
    
    // If our time to our closer mining station is less than their time to our
    // closer mining station and we can get to our closer mining station on time
    if (MIN(time1, time2) < tmp &&
        time + MIN(time1, time2) <= 179.0f) {
        
        ans = (2 - (time1 < time2));
    }
    
    else {
        /*
         // tmp holds the opponent's time to our farther mining station.
         if (time1 >= time2) {
         tmp = oTime1 + OC;
         }
         else {
         tmp = oTime2 + OC;
         }
         
         // If our time to our farther mining station is less than their time to our
         // farther mining station and we can get to our closer mining station on time
         if (MAX(time1, time2) < tmp &&
         time + MAX(time1, time2) <= 179.0f) {
         
         
         ans = (2 - (time1 > time2));
         }
         else {
         ans = 0;
         }
         */
        if (time + MAX(time1, time2) <= 179.0f) {
            if (oTime1 < oTime2)
                ans = 2;
            else
                ans = 1;
        }
    }
    
    // what if time1 ~= time2?  if fabsf(time1 - time2) < 1, going to the closer one (timewise)
    //doesn't help much.
    // check where opponent wants to go (based on time), and go to the other one
    if (ans && (fabsf(time1 - time2) < MSC)) { // neither one's much closer
        if (oTime1 < oTime2) {  // they want MS 1
            ans = (time + time2 >= 179.0f) ? ans : 2;      // let them take it
        }
        else {
            ans = (time + time1 >= 179.0f) ? ans : 1;
        }
    }
    
    return ans;
    
    //END::PROC::whichMS
}
static void spin (float myState[12], float center[3], float axis[3], float angVelocityRadians)
{
    //BEGIN::PROC::spin
    /**
     * Purpose: Spin on asteroid about an axis
     * Parameters:
     *   float myState[12]: current state of SPHERE
     *   float center[3]: position of asteroid
     *   float axis[3]: attitude of asteroid normal axis
     *   float angVelocityRadians: angular velocity in radians of desired spin rate
     */
    
#define radiansPerNM 21.5f //How many radians/sec of angular velocity one NM/sec of torque adds
    float torques[3]; //array to hold desired torques
    torques[0] = (angVelocityRadians / mathVecInner(myState + 6, axis, 3) - myState[9]) / radiansPerNM; //X torque based on desired and current velocities
    torques[1] = torques[2] = 0.0f; //Y and Z torques should be 0
    ZRSetTorques(torques);
    stationshoot(myState, center, 0.1f, 0.03f);
    //END::PROC::spin
}
static void doRevolve (float myState[12], float center[3], float axis[3], float radius, float angVelocityRadians, unsigned char go)
{
    //BEGIN::PROC::doRevolve
    /**
     * Purpose: Revolve around asteroid about an axis
     * Parameters:
     *   float myState[12]: current state of SPHERE
     *   float center[3]: position of asteroid
     *   float axis[3]: attitude of asteroid normal axis
     *   float radius: distance from axis we want to revolve
     *   float angVelocityRadians: angular velocity in radians of desired spin rate
     *   int go: boolean determining whether we should revolve or prepare
     */
    
#define radToDeg 57.29577f //multiply with radians to get degrees
    float vec[3]; //vector from center to SPHERE
    float newVel[3]; //new velocity, not including distance from asteroid adjustment
    float perp[3]; //vector from SPHERE perpendicular to and touching axis
    float final[3]; //new velocity, including distance adjustment OR position to go to
    float diff; //amount we need to move in/out from asteroid
    vecToTarget(center, myState, vec);
    mathVecCross(newVel, vec, axis);
    mathVecCross(perp, newVel, axis);
    diff = mathVecMagnitude(perp, 3) - radius;
    if (go) { //revolve
        mathVecNormalize(perp, 3);
        final[0] = perp[0] * diff + newVel[0] * angVelocityRadians;
        final[1] = perp[1] * diff + newVel[1] * angVelocityRadians;
        final[2] = perp[2] * diff + newVel[2] * angVelocityRadians;
        ZRSetVelocityTarget(final);
    }
    else { //prepare to revolve
        mathVecNormalize(perp, 3);
        perp[0] *= radius;
        perp[1] *= radius;
        perp[2] *= radius;
        mathVecSubtract(final, center, perp, 3);
        stationshoot(myState, final, 0.1f, 0.03f);
    }
    //END::PROC::doRevolve
}
static float timeToMS (float myState[6], float Station[3])
{
    //BEGIN::PROC::timeToMS
    /**
     * Parameters:
     *  myState: location of satellite (0-2), velocity of satellite (3-5)
     *  Station: location of mining station
     * Returns:
     *  Time it will take to go to the mining station.
     * 
     *
     * Note:
     *  ZRSetPositionTarget at speed = 0:
     *  return 21.2 * sqrt(disToTarget(myState, Station)) + 4.33;
     * 
     **/
    
#define safety 3.0f;
    
    float dist = disToTarget(myState, Station);
    float speed, diff[3], cross[3];
    
    speed = mathVecMagnitude(&myState[3], 3);
    if (0.0f == speed)
        speed = 0.00005f; // prevent division by 0
    mathVecSubtract(diff, Station, myState, 3);
    mathVecCross(cross, myState + 3, diff);
    
    // t(theta) * |v| + t(d)
    return ((-96.154f - 108.736f * (mathVecInner(myState + 3, diff, 3) / speed / dist /*cos*/
                                    - mathVecMagnitude(cross, 3) / speed / dist / 2.0f /*sin*/) + 2.6f / 0.023f) * speed)
    + (21.2f * sqrtf(dist) + 4.33f + 2.6f) + safety;
    //return 10.0f * dist + 13.0f + ( -115.61f * mathVecInner(myState + 3, diff, 3) / speed / dist - 97.12f ) * speed + safety;
    //END::PROC::timeToMS
}
