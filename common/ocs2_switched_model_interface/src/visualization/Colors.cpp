//
// Created by rgrandia on 20.02.20.
//

#include "ocs2_switched_model_interface/visualization/Colors.h"

namespace switched_model {

std::array<double, 3> getRGB(Color color) {
  switch (color) {
    case Color::blue:
      return {0, 0.4470, 0.7410};
    case Color::orange:
      return {0.8500, 0.3250, 0.0980};
    case Color::yellow:
      return {0.9290, 0.6940, 0.1250};
    case Color::purple:
      return {0.4940, 0.1840, 0.5560};
    case Color::green:
      return {0.4660, 0.6740, 0.1880};
    case Color::red:
      return {0.6350, 0.0780, 0.1840};
    case Color::black:
      return {0.25, 0.25, 0.25};
    default:
      return {0.0, 0.0, 0.0};
  }
}

}  // namespace switched_model
