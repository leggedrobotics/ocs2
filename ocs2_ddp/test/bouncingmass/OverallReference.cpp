#include "ocs2_ddp/test/bouncingmass/OverallReference.h"

OverallReference::OverallReference(const scalar_array_t trajTimes, const vector_array_t trajStates) {
  References_.clear();
  References_.resize(trajTimes.size() - 1);

  switchtimes_ = trajTimes;
  vector_t x0 = trajStates[0];

  for (int i = 0; i < trajTimes.size() - 1; i++) {
    References_[i] = Reference(trajTimes[i], trajTimes[i + 1], x0, trajStates[i + 1]);
    x0 = trajStates[i + 1];
    jumpMap(x0);
  }
}

int OverallReference::getIndex(scalar_t time) {
  for (int i = 0; i < switchtimes_.size() - 1; i++) {
    if (switchtimes_[i + 1] >= time) {
      return i;
    }
  }

  return -1;
}

void OverallReference::getInput(scalar_t time, vector_t& input) {
  int idx = getIndex(time);
  if (idx >= 0 && idx < References_.size()) {
    References_[idx].getInput(time, input);
  } else {
    input.setZero(INPUT_DIM);
  }
}

vector_t OverallReference::getInput(scalar_t time) {
  vector_t u;
  getInput(time, u);
  return u;
}

void OverallReference::getState(int idx, scalar_t time, vector_t& x) {
  if (idx >= 0 && idx < References_.size()) {
    if (time < 2) {
      References_[idx].getState(time, x);
    } else {
      getState(time, x);
    }
  } else {
    References_[0].getState(0, x);
  }
}

vector_t OverallReference::getState(int idx, scalar_t time) {
  vector_t state;
  getState(idx, time, state);
  return state;
}

void OverallReference::getState(scalar_t time, vector_t& x) {
  int idx = getIndex(time);
  if (idx >= 0 && idx < References_.size()) {
    References_[idx].getState(time, x);
  } else {
    References_[0].getState(0, x);
  }
}

vector_t OverallReference::getState(scalar_t time) {
  vector_t state;
  getState(time, state);
  return state;
}

void OverallReference::extendref(scalar_t delta) {
  for (int i = 0; i < References_.size(); i++) {
    Reference* pre;
    Reference* post;
    if (i > 0) {
      pre = &References_[i - 1];
    } else {
      pre = nullptr;
    }

    if (i <= References_.size() - 1) {
      post = &References_[i + 1];
    } else {
      post = nullptr;
    }
    References_[i].extendref(delta, pre, post);
  }
}

void OverallReference::display(int i) {
  References_[i].display();
}

void OverallReference::jumpMap(vector_t& x) {
  scalar_t e = 0.95;
  x[1] = x[1] - (1 + e) * x[1];
  x[2] = x[2] + 1;
}
