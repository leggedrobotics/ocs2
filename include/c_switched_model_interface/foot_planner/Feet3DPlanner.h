/*
 * Feet3DPlanner.h
 *
 *  Created on: Jul 10, 2017
 *      Author: farbod
 */

#ifndef FEETPLANNER_H_
#define FEETPLANNER_H_

#include <memory>
#include <array>
#include <vector>

#include "c_switched_model_interface/foot_planner/FeetPlannerBase.h"
#include "c_switched_model_interface/foot_planner/cpg/FootCPG.h"

namespace switched_model {

template <typename scalar_t>
class Feet3DPlanner : public FeetPlannerBase<scalar_t, FootCPG<scalar_t>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	typedef std::shared_ptr<Feet3DPlanner<scalar_t>>  Ptr;

	typedef FootCPG<scalar_t>								foot_cpg_t;
	typedef std::array<std::shared_ptr<foot_cpg_t>,4> 		feet_cpg_ptr_t;
	typedef std::array<const std::shared_ptr<foot_cpg_t>,4>	feet_cpg_const_ptr_t;

	typedef FeetPlannerBase<scalar_t, foot_cpg_t> 	BASE;

	typedef typename BASE::bool_array_t			bool_array_t;
	typedef typename BASE::size_array_t     	size_array_t;
	typedef typename BASE::scalar_array_t   	scalar_array_t;

	typedef Eigen::Matrix<scalar_t,3,1>	vector_3d_t;
	typedef std::array<vector_3d_t, 4> 	vector_3d_array_t;

	/**
	 * default constructor
	 */
	Feet3DPlanner()
	: Feet3DPlanner(0.15, 1.0)
	{}

	/**
	 * Constructor
	 *
	 * @param [in] swingLegLiftOff: Maximum swing leg lift-off
	 * @param [in] swingTimeScale: The scaling factor for adapting swing leg's lift-off
	 */
	Feet3DPlanner(const scalar_t& swingLegLiftOff, const scalar_t& swingTimeScale = 1.0);

	/**
	 * copy constructor
	 *
	 * @param [in] rhs
	 */
	Feet3DPlanner(const Feet3DPlanner& rhs);

	/**
	 * destructor.
	 */
	~Feet3DPlanner() override = default;

	/**
	 * clone the class
	 */
	Feet3DPlanner<scalar_t>* clone() const override;

	/**
	 * Plans the CPG for the swing legs in the indexed mode.
	 *
	 * @param [in] index: The index of the subsystem for which the CPG should be designed.
	 * @param [in] phaseIDsStock: An array of the natural number which gives a unique ID to 2^n (e.g. for
	 * a quadruped 2^4) possible stance leg choices.
	 * @param [in] eventTimes: The event times.
	 * @param [out] plannedCPG: An array of pointers to the CPG class for each endeffector.
	 */
	void planSingleMode(const size_t& index,
			const size_array_t& phaseIDsStock,
			const scalar_array_t& eventTimes,
			feet_cpg_ptr_t& plannedCPG) override;

	/**
	 *
	 * @param [in] touchdownTimeStock
	 * @param [in] touchdownPosStock
	 * @param [in] touchdownVelStock
	 */
	void setFootholdsPlan(
			const scalar_array_t& touchdownTimeStock,
			const std::vector<vector_3d_array_t>& touchdownPosStock,
			const std::vector<vector_3d_array_t>& touchdownVelStock);


private:
	scalar_t swingLegLiftOff_;
	scalar_t swingTimeScale_;

	size_array_t phaseIDsStock_;
	scalar_array_t eventTimes_;
	std::array<bool_array_t,4> eesContactFlagStocks_;

	scalar_array_t touchdownTimeStock_;
	std::vector<vector_3d_array_t> touchdownPosStock_;
	std::vector<vector_3d_array_t> touchdownVelStock_;
};

}  // end of switched_model namespace

#include "implementation/Feet3DPlanner.h"

#endif /* FEETPLANNER_H_ */
