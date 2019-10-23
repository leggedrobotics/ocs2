namespace ocs2{


class ball_tester_logic : public HybridLogicRules{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using BASE = HybridLogicRules;

	ball_tester_logic()=default;

	~ball_tester_logic() = default;

	ball_tester_logic(scalar_array_t switchingTimes, size_array_t sybsystemsSequence)
	{}

	void rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime)
	{}

	void update()
	{}

protected:
	
	void insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate,
					const scalar_t& startTime,
					const scalar_t& finalTime) override {};	


};


class ball_tester_dyn : public ControlledSystemBase<2,1>{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ball_tester_dyn() = default;
	~ball_tester_dyn() = default;

	void computeFlowMap( const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double,1,1>& u, Eigen::Vector2d& dxdt){

	  Eigen::Matrix<double,2,2> A;
	    A << 0,1,0,0 ;
	    Eigen::Matrix<double,2,1> B;
	    B << 0,0;
	    Eigen::Matrix<double,2,1> F;
	    F << 0,-9.81;

	    dxdt = A*x + B*u + F;
}

	void computeJumpMap(	const scalar_t& time, 
				const state_vector_t& state,
				state_vector_t& mappedState	)
	{
			mappedState[0] = state[0];
			mappedState[1] = -state[1];
	}
				

	void computeGuardSurfaces(	const scalar_t& time, 
					const state_vector_t& state, 
					dynamic_vector_t& guardSurfacesValue){

	guardSurfacesValue = Eigen::Matrix<double,2,1>();
	guardSurfacesValue[0] = state[0];
	guardSurfacesValue[1] = -state[0] + 0.1 + time/50;
	}

	ball_tester_dyn* clone() const final{
	    return new ball_tester_dyn(*this);
	}

};
}
