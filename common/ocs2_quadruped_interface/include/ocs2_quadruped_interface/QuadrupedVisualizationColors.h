//
// Created by rgrandia on 20.02.20.
//

#pragma once

#include <array>

namespace switched_model {

enum class Color { blue, orange, yellow, purple, green, red, black };

std::array<double, 3> getRGB(Color color);

}  // namespace switched_model
