
#ifndef SFML_VISUALIZER_HPP_
#define SFML_VISUALIZER_HPP_

// C/C++
#include <cmath>
#include <thread>
#include <mutex>
#include <unistd.h>

// SFML 2D graphics library
#include <SFML/Graphics.hpp>

class SfmlVisualizer {
 public:
  SfmlVisualizer() = delete;

  //! Define custom constructor
  SfmlVisualizer(std::string name, unsigned int windowWidth = 400, unsigned int windowHeight = 600):
    name_(name),
    window_(),
    event_(),
    eventThread_(),
    loopMutex_(),
    windowWidth_(windowWidth),
    windowHeight_(windowHeight),
    scale_(1.0f),
    visualizerIsRunning_(false)
  {
  };

  //! Destructor
  virtual ~SfmlVisualizer() {
    if (visualizerIsRunning_)
      stop();
  };

  //! Start event thread (necessary)
  virtual void start() {
    // Create the visualization window
    window_.create(sf::VideoMode(windowWidth_, windowHeight_),
                   name_,
                   sf::Style::Titlebar | sf::Style::Close,
                   sf::ContextSettings(24, 8, 4));
    // Activate window in the main thread
    window_.setActive();
    // Launch event worker thread to poll window for input events
    eventThread_ = std::unique_ptr<std::thread>( new std::thread(&SfmlVisualizer::windowEventWorkerCallback, this));
    // Flag that execution has commenced
    visualizerIsRunning_ = true;
    loopMutex_.unlock();
  };

  //! Start event thread (necessary)
  virtual void stop() {
    visualizerIsRunning_ = false;
    if(eventThread_)
      eventThread_->join();
    window_.close();
  };

  //! Save a captured frame
  void saveFrameAsImage(std::string filename) {
    loopMutex_.lock();
    window_.setActive(true);
    window_.capture().saveToFile(filename);
    window_.setActive(false);
    loopMutex_.unlock();
  };

 protected:

  void windowEventWorkerCallback() {
    // NO RENDERING HERE (to avoid calling window_.setActive())
    while (true) {
      if (loopMutex_.try_lock()) {
        if (!visualizerIsRunning_) break;
        // check all the window's events that were triggered since the last iteration of the loop
        if (window_.pollEvent(event_)) {
          // "close requested" event: we close the window
          if (event_.type == sf::Event::Closed) {
            window_.close();
            visualizerIsRunning_ = false;
          }
          if (event_.type == sf::Event::MouseWheelMoved) {
            if (event_.mouseWheel.delta > 0) scale_ *= 1.05;
            else scale_ *= 0.95;
          }
        }
        loopMutex_.unlock();
      }
      usleep(5); // for smooth visualization
    }
  }

 public:
  //! Visualizer lexical identifier
  std::string name_;
  //! SFML rendering object
  sf::RenderWindow window_;
  // SFML event worker and synchronization mechanisms
  sf::Event event_;
  std::unique_ptr<std::thread> eventThread_;
  std::mutex loopMutex_;
  // SFML window configurations
  unsigned int windowWidth_;
  unsigned int windowHeight_;
  float scale_;
  //! Runtime flag
  bool visualizerIsRunning_;
};

#endif // SFML_VISUALIZER_HPP_
