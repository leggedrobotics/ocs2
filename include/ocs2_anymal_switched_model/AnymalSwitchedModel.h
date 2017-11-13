/*
 * HyqSwitchedModel.h
 *
 *  Created on: Aug 9, 2016
 *      Author: farbod
 */

#ifndef ANYMAL_SWITCHEDMODEL_H_
#define ANYMAL_SWITCHEDMODEL_H_

#include <map>
#include <array>
#include <string>

namespace anymal {

struct Options
{
	Options()
	: constrainedIntegration_(true),
	  useCartesianContactForce_(false),
	  contactForceWeight_(0.1),
	  zDirectionPositionWeight_(5.0),
	  zDirectionVelocityWeight_(0.5),
	  swingLegLiftOff_(0.3),
	  mpcStrideLength_(0.35),
	  numPhasesInfullGaitCycle_(4),
	  defaultStartMode_(15),
	  defaultFinalMode_(15)
	 {}

	bool constrainedIntegration_;
	bool useCartesianContactForce_;
	double contactForceWeight_;
	double zDirectionPositionWeight_;
	double zDirectionVelocityWeight_;
	double swingLegLiftOff_;
	double mpcStrideLength_;

	size_t numPhasesInfullGaitCycle_;
	size_t defaultStartMode_;
	size_t defaultFinalMode_;
};


enum ModeNumber { // {LF, RF, LH, RH}
	FLY				=  0,
	RH				=  1,
	LH				=  2,
	LH_RH			=  3,
	RF				=  4,
	RF_RH			=  5,
	RF_LH			=  6,
	RF_LH_RH	=  7,
	LF				=  8,
	LF_RH			=  9,
	LF_LH			= 10,
	LF_LH_RH	= 11,
	LF_RF			= 12,
	LF_RF_RH	= 13,
	LF_RF_LH	= 14,
	STANCE		= 15,
};


class Mode
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static std::array<bool,4> ModeNumber2StanceLeg(const size_t& modeNumber)  {

		std::array<bool,4> stanceLegs;  // {LF, RF, LH, RH}

		switch (modeNumber)  {
		case 0:	 stanceLegs = std::array<bool,4>{0,0,0,0};	break;	// 0:  0-leg-stance
		case 1:	 stanceLegs = std::array<bool,4>{0,0,0,1};	break;  // 1:  RH
		case 2:	 stanceLegs = std::array<bool,4>{0,0,1,0};	break;  // 2:  LH
		case 3:	 stanceLegs = std::array<bool,4>{0,0,1,1};	break;  // 3:  RH, LH
		case 4:	 stanceLegs = std::array<bool,4>{0,1,0,0};	break;  // 4:  RF
		case 5:	 stanceLegs = std::array<bool,4>{0,1,0,1};	break;  // 5:  RF, RH
		case 6:	 stanceLegs = std::array<bool,4>{0,1,1,0};	break;	// 6:  RF, LH
		case 7:	 stanceLegs = std::array<bool,4>{0,1,1,1};	break;  // 7:  RF, LH, RH
		case 8:	 stanceLegs = std::array<bool,4>{1,0,0,0};	break;	// 8:  LF,
		case 9:	 stanceLegs = std::array<bool,4>{1,0,0,1};	break;	// 9:  LF, RH
		case 10: stanceLegs = std::array<bool,4>{1,0,1,0};	break;	// 10: LF, LH
		case 11: stanceLegs = std::array<bool,4>{1,0,1,1};  break;  // 11: LF, LH, RH
		case 12: stanceLegs = std::array<bool,4>{1,1,0,0};	break;  // 12: LF, RF
		case 13: stanceLegs = std::array<bool,4>{1,1,0,1};	break;  // 13: LF, RF, RH
		case 14: stanceLegs = std::array<bool,4>{1,1,1,0};	break;  // 14: LF, RF, LH
		case 15: stanceLegs = std::array<bool,4>{1,1,1,1};	break; 	// 15: 4-leg-stance
		}

		return stanceLegs;
	}


	static std::string ModeNumber2String(const size_t& modeNumber)  {

		// build the map from mode number to name
		std::map<size_t,std::string> modeToName;
		modeToName[FLY]      = "FLY";
		modeToName[RH]       = "RH";
		modeToName[LH]       = "LH";
		modeToName[LH_RH]    = "LH_RH";
		modeToName[RF]       = "RF";
		modeToName[RF_RH]    = "RF_RH";
		modeToName[RF_LH]    = "RF_LH";
		modeToName[RF_LH_RH] = "RF_LH_RH";
		modeToName[LF]       = "LF";
		modeToName[LF_RH]    = "LF_RH";
		modeToName[LF_LH]    = "LF_LH";
		modeToName[LF_LH_RH] = "LF_LH_RH";
		modeToName[LF_RF]    = "LF_RF";
		modeToName[LF_RF_RH] = "LF_RF_RH";
		modeToName[LF_RF_LH] = "LF_RF_LH";
		modeToName[STANCE]   = "STANCE";

		return modeToName[modeNumber];
	}


	static size_t String2ModeNumber(const std::string& modeString)  {

		// build the map from name to mode number
		std::map<std::string,size_t> nameToMode;
		nameToMode["FLY"]      = FLY;
		nameToMode["RH"]       = RH;
		nameToMode["LH"]       = LH;
		nameToMode["LH_RH"]    = LH_RH;
		nameToMode["RF"]       = RF;
		nameToMode["RF_RH"]    = RF_RH;
		nameToMode["RF_LH"]    = RF_LH;
		nameToMode["RF_LH_RH"] = RF_LH_RH;
		nameToMode["LF"]       = LF;
		nameToMode["LF_RH"]    = LF_RH;
		nameToMode["LF_LH"]    = LF_LH;
		nameToMode["LF_LH_RH"] = LF_LH_RH;
		nameToMode["LF_RF"]    = LF_RF;
		nameToMode["LF_RF_RH"] = LF_RF_RH;
		nameToMode["LF_RF_LH"] = LF_RF_LH;
		nameToMode["STANCE"]   = STANCE;

		return nameToMode[modeString];
	}

};

}  // end of anymal namespace

#endif /* ANYMAL_SWITCHEDMODEL_H_ */
