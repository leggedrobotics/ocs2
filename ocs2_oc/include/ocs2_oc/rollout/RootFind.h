#include "ocs2_core/Dimensions.h"
#include <utility>
#include <vector>

using scalar_t = double;
using interval_t = std::pair<scalar_t, scalar_t>;

namespace ocs2{
class RootFind{

public:
	RootFind(){};
	~RootFind(){};

	/**
	 * Set the initial bracket when the RootFinding method is intialized.
	 * Normally done when first zero crossing is detected
	 *
	 * @param [in] time_int: 		Pair of two time moments
	 * @param [in] guard_int:		Pair of two function values at time_int times, should have opposite sign
	 *
	 */
	void set_Init_Bracket(const interval_t &time_int, const interval_t &guard_int)
	{
		if(guard_int.first * guard_int.second > 0)
		{throw std::runtime_error("Bracket function values should have opposite sign");}

		time_int_m = time_int;
		guard_int_m = guard_int;

	}

	/**
	 * Set the initial bracket when the RootFinding method is intialized.
	 * Normally done when first zero crossing is detected
	 *
	 * @param [in] t0: 		First time of bracketing interval
	 * @param [in] t1: 		Second time of bracketing interval
	 * @param [in] f0: 		Function value corresponding to t0
	 * @param [in] f1: 		Function value corresponding to t1, of opposite sign to f0
	 *
	 */

	void set_Init_Bracket(const scalar_t t0,const scalar_t t1, const scalar_t f0, const scalar_t f1)
	{
		if(f0 * f1 > 0)
		{	std::cout<<t0<<";"<<t1<<std::endl;
			std::cout<<f0<<";"<<f1<<std::endl;
			std::cout<<t1-100<<std::endl;
			throw std::runtime_error("Bracket function values should have opposite sign");}


		time_int_m = std::make_pair(t0,t1);
		guard_int_m= std::make_pair(f0,f1);
	}

	/**
	 * Update Current bracket, based on based on sign of the new query point
	 *
	 * @param [in] query:		Time moment of last query point
	 * @param [in] f_query:		Function evaluation of last query time
	 *
	 */

	void Update_Bracket(const scalar_t &query,const scalar_t &f_query)
	{
		if(f_query * guard_int_m.first < 0)
		{
			guard_int_m.second = guard_int_m.first;
			time_int_m.second = time_int_m.first;

			guard_int_m.first = f_query;
			time_int_m.first = query;

		}
		else
		{
			scalar_t gamma = 1 - (f_query/guard_int_m.first);

			if (gamma < 0 )
			{
				gamma = 0.5;
			}

			gamma = 0.5;

			guard_int_m.first = f_query;
			time_int_m.first = query;

			guard_int_m.second *= gamma;
		}
	}

	/**
	 * Use (adapted-) regula falsi method to obtain a new query point
	 *
	 * @param [out] query:		Time moment of new query point
	 *
	 */

	void getNewQuery(double &query)
	{
		scalar_t fa = guard_int_m.first;
		scalar_t ta = time_int_m.first;

		scalar_t fb = guard_int_m.second;
		scalar_t tb = time_int_m.second;

		query = (ta * fb  - tb * fa)/(fb-fa);

	}

	/**
	 * Display relevant bracketing information
	 *
	 */

	void Display()
	{
		std::cout<<"Root Finding Information"<<std::endl;
		std::cout<<"Time Bracket: ["<< time_int_m.first<<";"<<time_int_m.second<<"]"<<std::endl;
		std::cout<<"Value Bracket: ["<< guard_int_m.first<<";"<<guard_int_m.second<<"]"<<std::endl;
	}


private:
	interval_t time_int_m;
	interval_t guard_int_m;
};
}
