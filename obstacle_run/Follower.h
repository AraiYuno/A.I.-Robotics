/*
 *   BallFollower.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _FOLLOWER_H_
#define _FOLLOWER_H_

#include "Point.h"
#include "BallTracker.h"


namespace Robot
{
	class Follower
	{
	private:
		int m_NoBallMaxCount;
		int m_NoBallCount;
		int m_KickBallMaxCount;
		int m_KickBallCount;

		double m_MaxFBStep;
		double m_MaxRLStep;
		double m_MaxDirAngle;

		double m_KickTopAngle;
		double m_KickRightAngle;
		double m_KickLeftAngle;

		double m_FollowMaxFBStep;
        	double m_FollowMinFBStep;
		double m_FollowMaxRLTurn;
       		double m_FitFBStep;
		double m_FitMaxRLTurn;
		double m_UnitFBStep;
		double m_UnitRLTurn;
		
		double m_GoalFBStep;
		double m_GoalRLTurn;
		double m_FBStep;
		double m_RLTurn;
		
		bool reverse;

	protected:

	public:
		bool DEBUG_PRINT;
		int KickBall;		// 0: No ball 1:Left -1:Right

		Follower();
		~Follower();

		void Process(Point2D ball_pos);
		void setWalk(int value); 
		void setReverse(bool boolValue);
	};
}

#endif
