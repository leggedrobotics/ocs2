/*
 * SwitchedModelAuxiliaryFuntions.h
 *
 *  Created on: Jun 5, 2016
 *      Author: farbod
 */

#ifndef HYQ_SWITCHEDMODELAUXILIARYFUNTIONS_H_
#define HYQ_SWITCHEDMODELAUXILIARYFUNTIONS_H_

#include <array>

namespace hyq {

enum ModeNumber { // {LF, RF, LH, RH}
	FLY         = 0,
	RH 			= 1,
	LH    		= 2,
	LH_RH		= 3,
	RF   		= 4,
	RF_RH		= 5,
	RF_LH		= 6,
	RF_LH_RH	= 7,
	LF			= 8,
	LF_RH		= 9,
	LF_LH		= 10,
	LF_LH_RH	= 11,
	LF_RF		= 12,
	LF_RF_RH	= 13,
	LF_RF_LH	= 14,
	STANCE   	= 15,
};

inline std::array<bool,4> ModeNumber2StanceLeg(size_t modeNumber)  {

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

}  // end of hyq namespace

#endif /* HYQ_SWITCHEDMODELAUXILIARYFUNTIONS_H_ */
