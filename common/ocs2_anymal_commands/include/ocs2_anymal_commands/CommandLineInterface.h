//
// Created by rgrandia on 04.05.20.
//

#pragma once

#include <chrono>
#include <future>
#include <string>
#include <vector>

inline std::string getCommandLineString() {
  // Set up a thread to read user inputs
  std::string line{""};
  std::atomic_bool lineRead(false);
  std::thread thr([&line, &lineRead]() {
    lineRead = false;
    getline(std::cin, line);
    lineRead = true;
  });

  // wait till line is read or terminate if ROS is gone.
  ::ros::WallRate rate(30);
  while (!lineRead) {
    if (!ros::ok() || !ros::master::check()) {
      std::terminate();  // Need to terminate thread that is still waiting for input
    }
    rate.sleep();
  }
  if (thr.joinable()) {
    thr.join();
  }
  return line;
}

inline std::vector<std::string> stringToWords(const std::string& str) {
  std::istringstream iss(str);
  std::vector<std::string> words{std::istream_iterator<std::string>{iss},
                                 std::istream_iterator<std::string>{}};
  return words;
}