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
Project: Rush
Game: RetroSPHERES_ALLIANCE
Created by: jerryma1121
Last Modified: 2012-12-07 20:54:13.0
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
	#define printDebug 0
	#define stillDis 0.0490f
	#define stillVel 0.0095f
	#define pickUpSpeed 0.55f
	#define gravCons 0.0749108f
	
	vec zeroVec;
	bool red;
	bool otherRush;
	
	#define printVec(name, v) DEBUG(("%s\t%.3f\t%.3f\t%.3f\n", name, v[0], v[1], v[2]))
	#define getZero getItem(0, red ? -0.55f : 0.35f, red ? -0.35f : 0.55f, .4f, .6f, floatZero, floatZero, 0.01f)
	#define getOne getItem(1, floatZero, floatZero, 0.5f, 0.75f, -0.6f, 0.6f, 0.01f)
	#define getTwo getItem(2, floatZero, floatZero, 0.15f, 0.4f, -0.6f, 0.6f, 0.01f)
	
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
	vec iloc;
	
	void getItem (int itemNo, float x0, float x1, float y0, float y1, float z0, float z1, float acc) {
		float best = floatZero;
		int lim = 40;
		if (time < lim)
			lim = time;
	  for (float x = x0; x < x1 + floatItsml; x += acc) {
	    for (float y = y0; y < y1 + floatItsml; y += acc) {
	      for (float z = z0; z < z1 + floatItsml; z += acc) {
	        float temp = floatZero;
	        for (int i = 0; i <= lim; i ++) {
						float diff = fabs(tridis[i][itemNo] - disBetweenPts(makeVec(x, y, z), tripos[i]));
	          if (diff < 0.0101f)
							temp += floatOne - diff;
					}
	        if (temp > best) {
	          iloc[0] = x; iloc[1] = y; iloc[2] = z;
	          best = temp;
	        }
	      }
	    }
	  }
		#if printDebug
		DEBUG(("getItem(%d)\n", itemNo));
		printVec("\tItemPos (m)", iloc);
		DEBUG(("\tAccuracy Score\t%.3f\n", best / (time + 1.0f)));
		DEBUG(("\tVel Magnitude (m/s)\t%.3f\n", mathVecMagnitude(myState + 3, 3)));
		DEBUG(("\tAngvel Magnitude (deg/s)\t%.3f\n", mathVecMagnitude(myState + 9, 3) * 57.296f));
		DEBUG(("\tDis from Item (m)\t%.3f\n", disBetweenPts(myState, iloc)));
		#endif
		api.setPositionTarget(iloc);
		if (hitTarget(myState, iloc)) {
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
		if (time < 4) {
			api.setVelocityTarget(makeVec(floatZero, floatZero, 0.0095f));
		}
		if (!time) {
			game.startObstacle();
			return;
		}
		if (time == 1) {
			game.stopObstacle();
		}
	
		
		if (!game.haveObject(0)) {
			if (!game.haveObject(1) && !game.otherHasObject(1))
				getOne;
			else
				getZero;
		}
		else {
			vec temp = {(red ? 0.15f : -0.15f), floatZero, myState[2]};/*
			if (!obsDetected && ((red && myState[0] > -0.05f) || (!red && myState[0] < 0.05f))) {
				int res = game.extendView();
				if (printDebug) {
					DEBUG(("Extending View: %d\n", res));
				}
			}*/
			float tryy[7] = {(otherRush ? myState[2] : 0.38f), 0.11f, 0.22f, 0.34f, 0.45f, 0.55f, 0.68f};
			obsDetected = game.getIdentifiedObstacles(obstacles);
			bool shr = false;
			for (int i = 0; i < 7; i ++) {
				if (printDebug)
					DEBUG(("\tTrying %.3f\n", tryy[i]));
				bool good = true;
				temp[2] = tryy[i];
				for (int j = 0; j < obsDetected; j ++) {
					temp[1] = obstacles[j].loc[1];
					if (printDebug) {
				 		DEBUG(("\t\tID %d, rad %.3f\n", obstacles[j].ID, obstacles[j].size));
						printVec("\t\tLocation", obstacles[j].loc);
						DEBUG(("\t\tDis %.3f\n", disBetweenPts(obstacles[j].loc, temp)));
					}
					if (disBetweenPts(obstacles[j].loc, temp) < obstacles[j].size + 0.058f) {
						good = false;
						break;
					}
					if (!shr && obstacles[j].visible) {
						DEBUG(("\t\tShrinking\n"));
						shr = true;
						game.shrinkObstacle(obstacles[j].ID);
					}
				}
				if (good) {
					if (printDebug)
						DEBUG(("Decided %.3f\n", tryy[i]));
					temp[2] = tryy[i];
					break;
				}
			}
			temp[1] = floatZero;
			if ((red && myState[0] < floatZero) || ((!red && myState[0] > floatZero))) {
				goStraight(temp, 1, -0.024f);
			}
		  else if (myState[1] > -0.66f) {
				goStraight(temp, 1, (myState[1] > -0.48f && fabs(myState[2] - temp[2]) < 0.058f ? -0.0475f : -0.0235f));
			}
			else {
				if (!alreadySet) {
		  	 	alreadySet = true;
		  	 	top = myState[2] > floatZero;
		  	}
				temp[1] = -0.71f;
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
	
	float disBetweenPts (vec pt1, vec pt2) {
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
	
	void vecCpy (vec src, vec dest) {
		dest[0] = src[0];
		dest[1] = src[1];
		dest[2] = src[2];
	}

//END::PAGE::util

};

static ZRUser01 userInstance;
ZRUser *zruser01 = &userInstance;
