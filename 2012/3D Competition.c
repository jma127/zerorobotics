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
#include "ZRUser.hpp"

#undef ZRSIMULATION

extern ZeroRoboticsGame game;
static ZeroRoboticsAPI& api = game.api;

/*
 Project: cloudtroll
 Game: RetroSPHERES
 Created by: jerryma1121
 Last Modified: 2012-10-31 19:13:44.0
 */
class ZRUser01 : public ZRUser
{
public:
    
    //BEGIN::PAGE::adef
	typedef unsigned char cmd;
	typedef float vec[3];
	
#define accPerNM 43.0f
#define coneAng 0.1047f
#define degPerRad 57.2957795131f
#define floatOne 1.0f
#define floatZero 0.0f
#define floatItsml 0.00001f
#define maxAcc 0.09f
#define osMag 0.5f
#define lim 0.25f
#define printDebug 0
#define stillDis 0.049f
	
	vec zeroVec;
	bool red;
	bool alreadySet;
	
#define nothing 0u
	
#define disToHalt(vel) (1.05f * (46.413f * vel * vel - 0.1385f * vel + 0.0031f))
#define degToRad(angle) (angle / degPerRad)
#define getBit(message, bit) ((message >> bit) & 1U)
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define printVec(name, v) DEBUG(("%s\t%.3f\t%.3f\t%.3f\n", name, v[0], v[1], v[2]))
#define radToDeg(angle) (angle * degPerRad)
#define stateAngVel(state) (state + 9)
#define stateAtt(state) (state + 6)
#define statePos(state) (state)
#define stateVel(state) (state + 3)
	
	float fuel;
	float dust;
	float myScore;
	float otherScore;
	int chg;
	unsigned short otherMessage;
	ZRState myState;
	ZRState otherState;
	int time;
	int phase;
	int obsCreated;
	float top;
    
    //END::PAGE::adef
    //BEGIN::PAGE::ctrl
	void overshoot(vec targetPos) {
		vec nPos, att;
		float dis = disBetweenPts(statePos(myState), targetPos);
		attToTarget(statePos(myState), targetPos, att);
		nPos[0] = targetPos[0] + att[0] * osMag * dis; //set target location beyond target
		nPos[1] = targetPos[1] + att[1] * osMag * dis;
		nPos[2] = targetPos[2] + att[2] * osMag * dis;
		float velComponent = mathVecInner(stateVel(myState), att, 3);
		float disLeft = dis - stillDis;
		float velDis = disToHalt(velComponent);
#if printDebug
		DEBUG(("Overshoot Info\n"));
		DEBUG(("\tOvershoot Magnitude\t%.3f\n", osMag));
		DEBUG(("\tOvershoot Threshold\t%.3f\n", lim));
		DEBUG(("\tDis to Target (m)\t%.3f\n", dis));
		DEBUG(("\tVel Component (m/s)\t%.3f\n", velComponent));
		DEBUG(("\tCur Vel Dis (m)\t%.3f\n", velDis));
		printVec("\tTarget Pos (m)", targetPos);
		printVec("\tTarget Att (unitvec)", att);
		printVec("\tNew Pos (m)", nPos);
#endif
		if (osMag <= floatZero || hitTarget(myState, targetPos)) {
#if printDebug
			DEBUG(("\tsetPositionTarget\n"));
#endif
			api.setPositionTarget(targetPos);
		}
		else if (dis < lim && velDis > disLeft + 0.001f) {
#if printDebug
			DEBUG(("\tHalting\n"));
#endif
			setVelocity(zeroVec);
		}
		else { //overshoot
#if printDebug
			DEBUG(("\tSpeeding Up\n"));
#endif
			api.setPositionTarget(nPos);
		}
	}
	
	void setTargetAngVel(vec targetAngVel) {
		vec torques;
		mathVecSubtract(torques, targetAngVel, stateAngVel(myState), 3);
		vecScale(torques, floatOne / accPerNM);
#if printDebug
		DEBUG(("Angular Velocity Targeting Info\n"));
		printVec("\tTarget AngVel (rad/s)", targetAngVel);
		printVec("\tTorques (rad/s^2)", torques);
#endif
		api.setTorques(torques);
	}
	
	void setVelocity (vec targetVel) {
#if printDebug
		DEBUG(("Velocity Targeting Info\n"));
		printVec("Target Vel (m)", targetVel);
#endif
		vec temp;
		mathVecSubtract(temp, targetVel, stateVel(myState), 3);
		float magnitude = mathVecMagnitude(temp, 3);
		if (magnitude < 0.02f) {
			api.setVelocityTarget(targetVel);
			return;
		}
		vecScale(temp, 4.2f);
		magnitude = fabs(mathVecMagnitude(temp, 3));
		if (magnitude > 0.11f) vecScale(temp, 0.11f / magnitude);
		api.setForces(temp);
	}
    
    //END::PAGE::ctrl
    //BEGIN::PAGE::game
	float maintainZ () {
		float ret = 0.04f - statePos(myState)[2];
		if (statePos(myState)[2] < floatZero) {
			ret = -0.04f - statePos(myState)[2];
		}
		return norm(ret, 0.02f);
	}
	
	void getItem (int itemNo) {
#if printDebug
		DEBUG(("getItem(%d) Info\n", itemNo));
#endif
		vec iloc;
		game.getItemLocation(itemNo, iloc);
#if printDebug
		printVec("\tItemPos (m)", iloc);
#endif
		api.setPositionTarget(iloc);
		if (hitTarget(myState, iloc)) {
			vec temp = {0.75f, floatZero, floatZero};
			setTargetAngVel(temp);
		}
		else {
			setTargetAngVel(zeroVec);
		}
	}
	
	bool startedYet;
	float prevSize;
	
	bool makeObs (float x, float y) {
		vec pos = {x, y, floatZero}, deg30 = {0.5f, floatZero, floatZero};
		if (hitTarget(myState, pos)) {
			setVelocity(zeroVec);
			setTargetAngVel(deg30);
			if (!startedYet) {
#if printDebug
				DEBUG(("Started obstacle\n"));
#endif
				game.startObstacle();
			}
			else {
				float curSize = game.getCurrentObstacleSize();
				if (dust < floatItsml || curSize > 0.299f || curSize - prevSize < floatItsml) {
#if printDebug
					DEBUG(("Ended obstacle\n"));
#endif
					game.stopObstacle();
					return true;
				}
				prevSize = curSize;
			}
			startedYet = true;
		}
		else {
			api.setPositionTarget(pos);
			//slowGo(pos);
			startedYet = false;
			setTargetAngVel(zeroVec);
			prevSize = floatZero;
		}
		return false;
	}
	
	Obstacle obstacles[10];
	
	void remDust () {
        int count = game.getIdentifiedObstacles(obstacles);
		for (int i = 0; i < count; i ++) {
			if (obstacles[i].visible && chg) {
				game.shrinkObstacle(obstacles[i].ID);
				return;
			}
		}
	}
    
    //END::PAGE::game
    //BEGIN::PAGE::main
	void init() {
		
		//Constant Variables
		
		zeroVec[0] = zeroVec[1] = zeroVec[2] = floatZero;
		
		//Global Variables
		
        time = -1;
		obsCreated = 0;
		alreadySet = false;
	}
	
	void loop() {
		
		//Variable Initialization
		
        time ++;
		api.getMyZRState(myState);
        api.getOtherZRState(otherState);
		phase = game.getCurrentPhase();
		fuel = game.getFuelRemaining();
		myScore = game.getScore();
		otherScore = game.getOtherScore();
		dust = game.getRemainingMaterial();
		chg = game.getCharge();
		
		if (time < 2) {
			red = statePos(myState)[0] < floatZero;
		}
		
		//Debug Console
		
		DEBUG(("\nDevilTech Message Screen: Fall 2012\n"));
#if printDebug
		DEBUG(("Debug Info\n"));
		DEBUG(("\tTime (s)\t%d\tPhase\t%d\tPlayer\t%s\n", time, phase, (red ? "Red" : "Blue")));
		DEBUG(("\tFuel\t%.3f\tDust\t%.3f\tChg\t%d\n", fuel, dust, chg));
		DEBUG(("\tScore\t%.2f\tOther Score\t%.2f\n", myScore, otherScore));
		DEBUG(("\tOther Message\t%u\n", otherMessage));
		printVec("\tMy Pos (m)", statePos(myState));
		printVec("\tMy Vel (m/s)", stateVel(myState));
		printVec("\tMy Att (unitvec)", stateAtt(myState));
		printVec("\tMy AngVel (rad/s)", stateAngVel(myState));
		printVec("\tOther Pos (m)", statePos(otherState));
		printVec("\tOther Vel (m/s)", stateVel(otherState));
		printVec("\tOther Att (unitvec)", stateAtt(otherState));
		printVec("\tOther AngVel (rad/s)", stateAngVel(otherState));
#endif
		
		vec att = {floatZero, -1.0f, floatZero};
		if (!obsCreated) {
			if (makeObs(red ? -0.17f : 0.17f, -0.63f)) obsCreated ++;
		}
		else if (obsCreated == 1) {
			if (makeObs(red ? -0.47f : 0.47f, -0.03f)) obsCreated ++;
		}
		else if (phase == 1) {
			vec temp = {norm((red ? -0.45f : 0.45f) - myState[0], 0.015f), 0.05f, floatZero};
			setVelocity(temp);
			//api.setAttitudeTarget(att);
		}
		else {
			if (!game.haveObject(0)) getItem(0);
			else if (!game.haveObject(1) && !game.otherHasObject(1)) getItem(1);
            else if (!game.haveObject(2) && !game.otherHasObject(2) && !game.haveObject(1)) getItem(2);
            else {
				if (game.haveObject(1) || game.haveObject(2)) {
                    api.setAttitudeTarget(att);
				}
                vec temp = {floatZero, floatZero, floatZero};
                if (phase == 2) {
                    vec temp2 = {(red ? 0.04f : -0.04f), 0.09f, floatZero};
                    attToTarget(myState, temp2, temp);
                    vecScale(temp, 0.057f);
                    temp[2] = maintainZ();
                    setVelocity(temp);
                }
                else {
                    if (!alreadySet) {
                        alreadySet = true;
                        top = myState[2] > floatZero;
                    }
                    if (myState[1] > -0.4f) {
						remDust();
                        temp[0] = norm((red ? 0.045f : -0.045f) - myState[0], 0.015f);
                        temp[1] = -0.051f;
                        temp[2] = maintainZ();
                    }
                    else if (myState[1] > -0.6f) {
						temp[0] = norm((red ? 0.045f : -0.045f) - myState[0], 0.015f);
                        temp[1] = -0.022f;
                        temp[2] = maintainZ();
                    }
                    else {
						temp[0] = norm((red ? 0.045f : -0.045f) - myState[0], 0.015f);
                        temp[1] = floatZero;
                        temp[2] = (top ? -0.055f : 0.055f);
                        DEBUG(("Top:\t%d\t%.3f\n", top, temp[2]));
                    }
                    setVelocity(temp);
                }
            }
        }
	}
    
    //END::PAGE::main
    //BEGIN::PAGE::util
	vec utilVec1;
	
	float angBetweenVecs (vec v1, vec v2) {
		return acosf(mathVecInner(v1, v2, 3) / (mathVecMagnitude(v1, 3) * mathVecMagnitude(v2, 3)));
	}
	
	void attToTarget (vec satPos, vec targetPos, vec att) {
		mathVecSubtract(att, targetPos, satPos, 3);
		mathVecNormalize(att, 3);
	}
	
	float disBetweenPts (vec pt1, vec pt2) {
		mathVecSubtract(utilVec1, pt2, pt1, 3);
		return mathVecMagnitude(utilVec1, 3);
	}
	
	bool hitTarget(ZRState satState, vec targetPos) {
		return (mathVecMagnitude(stateVel(satState), 3) < 0.0095f && disBetweenPts(statePos(satState), targetPos) < stillDis);
	}
	
	void vecCpy(vec src, vec dest) {
		dest[0] = src[0];
		dest[1] = src[1];
		dest[2] = src[2];
	}
	
	void vecScale (vec v, float a) {
		v[0] *= a;
		v[1] *= a;
		v[2] *= a;
	}
	
	float norm (float f, float mx) {
		if (f > mx) return mx;
		if (f < -mx) return -mx;
		return f;
	}
    
    //END::PAGE::util
    
};

static ZRUser01 userInstance;
ZRUser *zruser01 = &userInstance;
