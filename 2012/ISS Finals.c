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
Project: SubmissionISS
Game: RetroSPHERES_ALLIANCE
Created by: jerryma1121
Last Modified: 2012-12-17 23:43:11.0
*/
class ZRUser01 : public ZRUser
{
public:

//BEGIN::PAGE::adef
	typedef float vec[3];
	
	#define mass 4.3f
	#define accPerNM 43.0f
	#define floatOne 1.0f
	#define floatZero 0.0f
	#define floatItsml 1.0E-4f
	#define printDebug 1
	#define stillDis 0.0455f
	#define stillVel 0.0090f
	#define pickUpSpeed 0.57f
	#define gravCons 0.0749108f
	
	vec zeroVec;
	bool red;
	bool otherRush;
	
	#define printVec(name, v) DEBUG(("%s\t%.3f\t%.3f\t%.3f\n", name, v[0], v[1], v[2]))
	
	float dust;
	int chg;
	ZRState myState;
	ZRState otherState;
	int time;
	int phase;
	int obsCreated;
	int obsDetected;
	float top;
	bool alreadySet;
	bool cloudOn;
	
	ZRState tripos[200];
	float tridis[200][3];
	Obstacle obstacles[10];
	float obsdis[10];
	
	//Calculations
	
	#define acc 0.01f
	#define i0x 21
	#define i0y 21
	#define i12y 26
	#define i12z 121
	
	vec ilocs[3];

//END::PAGE::adef
//BEGIN::PAGE::ctrl
	void setTargetAngVel(vec targetAngVel) {
		vec torques;
		mathVecSubtract(torques, targetAngVel, myState + 9, 3);
		vecScale(torques, floatOne / accPerNM);
		#if printDebug
		DEBUG(("Angular Velocity Targeting Info\n"));
		printVec("\tTarget AngVel (rad/s)", targetAngVel);
		printVec("\tTorques (rad/s^2)", torques);
		#endif
		api.setTorques(torques);
	}
	
	void goStraight (float * pos, int dim, float vel) {
		pos[dim] = myState[dim];
		vec vels = {floatZero, floatZero, floatZero};
		vels[dim] = vel;
		api.setPositionTarget(pos);
		api.setVelocityTarget(vels);
		#if printDebug
		DEBUG(("goStraight(pos target, %d, %.3f)\n", dim, vel));
		printVec("\tPos target", pos);
		printVec("\tVel target", vels);
		#endif
	}

//END::PAGE::ctrl
//BEGIN::PAGE::game
	#define restTime 2
	#define modu 7
	#define modu2 7
	#define ecalc(time, item) fabs(disBetweenPts(storeTemp, tripos[time]) - tridis[time][item])
	#define allowErr 0.000011f
	
	void calcaccs () {
		if (time < restTime)
			return;
		if (time < modu + restTime) {
			//1
			int seg = time % modu;
			int i = (i12y / modu) * seg + MIN(seg, i12y % modu);
			int j;
			int	ilim = i + (i12y / modu) + (seg < i12y % modu ? 1 : 0);
			storeTemp[0] = floatZero;
			storeTemp[1] = 0.5f + 0.01f * i;
		  #if printDebug
		  DEBUG(("calcaccs item 1\n"));
			DEBUG(("\t[%d, %d) %.3f\n", i, ilim, storeTemp[1]));
		  #endif
			for (; i < ilim; i ++, storeTemp[1] += acc) {
				for (j = 0, storeTemp[2] = -0.6f; j < i12z; j ++, storeTemp[2] += acc) {
					if (ecalc(0, 1) + ecalc(2, 1) + ecalc(1, 1) < allowErr) {
						vecCpy(storeTemp, ilocs[1]);
					}
				}
			}
		}
		else if (time < modu + restTime + modu2) {
			//0
			int seg = time % modu2;
			int i = (i0x / modu2) * seg + MIN(seg, i0x % modu2);
			int j;
			int ilim = i + (i0x / modu2) + (seg < i0x % modu2 ? 1 : 0);
			storeTemp[2] = floatZero;
			storeTemp[0] = (red ? -0.55f : 0.35f) + 0.01f * i;
			#if printDebug
		  DEBUG(("calcaccs item 0\n"));
			DEBUG(("\t[%d, %d) %.3f\n", i, ilim, storeTemp[0]));
		  #endif
			for (; i < ilim; i ++, storeTemp[0] += acc) {
				for (j = 0, storeTemp[1] = 0.4f; j < i0y; j ++, storeTemp[1] += acc) {
					if (ecalc(0, 0) + ecalc(2, 0) + ecalc(1, 0) < allowErr) {
						vecCpy(storeTemp, ilocs[0]);
					}
				}
			}
		}
	}
	
	void getItem (int itemNo) {
		if (time < modu + restTime - 1) {
			float redvel = 0.001f;
			if (time)
				redvel = (red ? 0.02f : -0.02f);
			api.setVelocityTarget(makeVec(redvel, time ? 0.060f : 0.007f, (time % 2) ? -0.001f : 0.001f));
			return;
		}
		#if printDebug
		DEBUG(("getItem(%d)\n", itemNo));
		printVec("\tItemPos (m)", ilocs[itemNo]);
		DEBUG(("\tVel Magnitude (m/s)\t%.3f\n", mathVecMagnitude(myState + 3, 3)));
		DEBUG(("\tAngvel Magnitude (deg/s)\t%.3f\n", mathVecMagnitude(myState + 9, 3) * 57.296f));
		DEBUG(("\tDis from Item (m)\t%.3f\n", disBetweenPts(myState, ilocs[itemNo])));
		#endif
		api.setPositionTarget(ilocs[itemNo]);
		if (hitTarget(myState, ilocs[itemNo])) {
			#if printDebug
			DEBUG(("\tHit target\n"));
			#endif
			setTargetAngVel(makeVec(floatZero, floatZero, pickUpSpeed));
		}
		else {
			setTargetAngVel(zeroVec);
		}
	}
	
	vec targetAtt;
	
	bool makeObs (vec loc) {
		#if printDebug
		DEBUG(("makeObs()\n"));
		printVec("\tAt loc (m):", loc);
		#endif
	  if (!cloudOn) {
	    targetAtt[0] = -myState[6];
	    targetAtt[1] = -myState[7];
	    targetAtt[2] = -myState[8];
	    game.startObstacle();
			cloudOn = true;
	  }
		else if (hitTarget(myState, loc)) {
	    game.stopObstacle();
			cloudOn = false;
	    return true;
	  }
		#if printDebug
		printVec("\tTarget att", targetAtt);
		#endif
	  api.setAttitudeTarget(targetAtt);
	  api.setPositionTarget(loc);
	  return false;
	}
	
	void remDust () {
		int i = obsDetected = game.getIdentifiedObstacles(obstacles);
		api.setAttitudeTarget(makeVec(floatZero, -1.0f, floatZero));
		#if printDebug
		DEBUG(("remDust()\n"));
		DEBUG(("\tDetected\t%d\n", obsDetected));
		#endif
		while (i -- && chg) {
			#if printDebug
			DEBUG(("\tObstacle ID\t%d\tRadius\t%.3f\n", obstacles[i].ID, obstacles[i].size));
			printVec("\t\tObstacle Location", obstacles[i].loc);
			#endif
			if (obstacles[i].visible) {
				#if printDebug
				DEBUG(("\t\tShrinking\n"));
			  #endif
				game.shrinkObstacle(obstacles[i].ID);
				return;
			}
		}
	}

//END::PAGE::game
//BEGIN::PAGE::main
	void init() {
		
		//Global Variables
		
	  time = -1;
		obsCreated = 0;
		alreadySet = false;
		cloudOn = false;
		otherRush = false;
	}
	
	void loop() {
		
		//Constant Variables
		
		zeroVec[0] = zeroVec[1] = zeroVec[2] = floatZero;
		
		//Variable Initialization
		
	  time ++;
		api.getMyZRState(myState);
	  api.getOtherZRState(otherState);
		phase = game.getCurrentPhase();
		dust = game.getRemainingMaterial();
		chg = game.getCharge();
		
		if (!time)
			red = myState[0] < floatZero;
		if (time < 30 && otherState[1] > 0.1f)
			otherRush = true;
		
		//Trilateration
		
		api.getMyZRState(tripos[time]);
		game.pingForItems(tridis[time]);
		
		//Debug Console
		
		#if printDebug
		DEBUG(("\nUS#7 DevilTech BACON Hart District\nMessage Screen: Fall 2012\n"));
		DEBUG(("Debug Info\n"));
		DEBUG(("\tTime (s)\t%d\tPhase\t%d\tPlayer\t%s\n", time, phase, (red ? "Red" : "Blue")));
		DEBUG(("\tFuel\t%.3f\tDust\t%.3f\tChg\t%d\n", game.getFuelRemaining(), dust, chg));
		DEBUG(("\tMy Score\t%.3f\tOther Score\t%.3f\n", game.getScore(), game.getOtherScore()));
		DEBUG(("\tObstacles Created\t%d\n", obsCreated));
		DEBUG(("\tOther Rush\t%d\n", otherRush));
		printVec("\tMy Pos (m)", myState);
		printVec("\tMy Vel (m/s)", (myState + 3));
		printVec("\tMy Att (unitvec)", (myState + 6));
		printVec("\tMy AngVel (rad/s)", (myState + 9));
		printVec("\tOther Pos (m)", otherState);
		printVec("\tOther Vel (m/s)", (otherState + 3));
		printVec("\tOther Att (unitvec)", (otherState + 6));
		printVec("\tOther AngVel (rad/s)", (otherState + 9));
		#endif
		
		//Game Logic
		calcaccs();
		if (!time)
			game.startObstacle();
		if (time == 1)
			game.stopObstacle();
		if (!game.haveObject(0)) {
			if (!game.haveObject(1) && !game.otherHasObject(1))
				getItem(1);
			else
				getItem(0);
		}
		else {
			vec temp = {(red ? 0.15f : -0.15f), floatZero, (otherRush ? myState[2] : -0.61f)};
			obsDetected = game.getIdentifiedObstacles(obstacles);
			for (int j = 0; j < obsDetected; j ++) {
				temp[1] = obstacles[j].loc[1];
				obsdis[j] = disBetweenPts(temp, myState);
				if (printDebug) {
			 		DEBUG(("\t\tID %d, rad %.3f\n", obstacles[j].ID, obstacles[j].size));
					printVec("\t\tLocation", obstacles[j].loc);
					DEBUG(("\t\tDis %.3f\n", disBetweenPts(obstacles[j].loc, temp)));
				}
			}
			temp[1] = floatZero;
			for (int i = 0; i < obsDetected; i ++) {
				if (printDebug)
					DEBUG(("Considering obs %d dis %.3f\n", obstacles[i].ID, obsdis[i]));
				if (MIN(disBetweenPts(myState, obstacles[i].loc), obsdis[i]) < obstacles[i].size + 0.058f && obstacles[i].visible) {
					game.shrinkObstacle(obstacles[i].ID);
					if (printDebug)
						DEBUG(("\tShrunk\n"));
					break;
				}
			}
			if ((red && myState[0] < floatZero) || (!red && myState[0] > floatZero)) {
				goStraight(temp, 1, -0.020f);
			}
		  else if (myState[1] > -0.66f) {
				goStraight(temp, 1, (myState[1] > -0.50f && fabs(myState[2] - temp[2]) < 0.06f) ? -0.047f : -0.020f);
			}
			else {
				if (!alreadySet) {
		  	 	alreadySet = true;
		  	 	top = myState[2] > floatZero;
		  	}
				temp[1] = -0.725f;
				goStraight(temp, 2, top ? -0.055f : 0.055f);
			}
	  }
	}

//END::PAGE::main
//BEGIN::PAGE::util
	vec utilVec1;
	
	void attToTarget (vec satPos, vec targetPos, vec att) {
		mathVecSubtract(att, targetPos, satPos, 3);
		mathVecNormalize(att, 3);
	}
	
	inline float disBetweenPts (vec pt1, vec pt2) {
		mathVecSubtract(utilVec1, pt2, pt1, 3);
		return mathVecMagnitude(utilVec1, 3);
	}
	
	bool hitTarget(ZRState satState, vec targetPos) {
		return (mathVecMagnitude(satState + 3, 3) < stillVel && disBetweenPts(satState, targetPos) < stillDis);
	}
	
	void vecScale (vec v, float a) {
		v[0] *= a;
		v[1] *= a;
		v[2] *= a;
	}
	
	vec storeTemp;
	
	float * makeVec (float a, float b, float c) {
		storeTemp[0] = a;
		storeTemp[1] = b;
		storeTemp[2] = c;
		return storeTemp;
	}
	
	inline void vecCpy (vec src, vec dest) {
		dest[0] = src[0];
		dest[1] = src[1];
		dest[2] = src[2];
	}
	
	float MIN (float a, float b) {
		return (a < b ? a : b);
	}

//END::PAGE::util

};

static ZRUser01 userInstance;
ZRUser *zruser01 = &userInstance;
