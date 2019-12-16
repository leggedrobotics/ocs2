#include <Eigen/Dense>
#include <vector>

#include <ocs2_core/Dimensions.h>

/*
 *  Observer used to obtain state trajectory while integrating the reference input
 * 	to extend the reference past event times
 */
class Observer {
 using DIMENSIONS = ocs2::Dimensions<3, 1>;
 using state_vector_t = typename DIMENSIONS::state_vector_t;
 using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
 using scalar_t = typename DIMENSIONS::scalar_t;
 using scalar_array_t = typename DIMENSIONS::scalar_array_t;
 public:
  /*
   * Constructor
   *
   * @param [in] tStore: Pointer to Vector in which to store times
   * @param [in] xStore: Pointer to Vector in which to store statetrajectory
   */
  Observer(scalar_array_t* tStore, state_vector_array_t* xStore) : tStore_(tStore), xStore_(xStore) {}

  void operator()(const state_vector_t& x, const scalar_t& t) {
    tStore_->push_back(t);
    xStore_->push_back(x);
  }

 private:
  state_vector_array_t* xStore_;
  scalar_array_t* tStore_;
};
