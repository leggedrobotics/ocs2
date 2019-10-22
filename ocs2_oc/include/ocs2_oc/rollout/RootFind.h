#include "ocs2_core/Dimensions.h"
#include <utility>
#include <vector>

using scalar_t = double;
using interval_t = std::pair<scalar_t, scalar_t>;

namespace ocs2{
class Anderson_Bjorck{

public:
	Anderson_Bjorck(){};
	~Anderson_Bjorck(){};

	void set_Init_Bracket(const interval_t &time_int, const interval_t &guard_int)
	{
		time_int_m = time_int;
		guard_int_m = guard_int;

	}

	void set_Init_Bracket(const scalar_t t0,const scalar_t t1, const scalar_t f0, const scalar_t f1)
	{
		time_int_m = std::make_pair(t0,t1);
		guard_int_m= std::make_pair(f0,f1);
	}

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

			guard_int_m.first = f_query;
			time_int_m.first = query;

			guard_int_m.second *= gamma;
		}
	}

	void getNewQuery(double &query)
	{
		scalar_t fa = guard_int_m.first;
		scalar_t ta = time_int_m.first;

		scalar_t fb = guard_int_m.second;
		scalar_t tb = time_int_m.second;

		query = (ta * fb  - tb * fa)/(fb-fa);


	}


private:
	interval_t time_int_m;
	interval_t guard_int_m;
};
}
