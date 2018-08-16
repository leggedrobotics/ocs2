#ifndef CARTPOLE_VISUALIZER_HPP_
#define CARTPOLE_VISUALIZER_HPP_

// C/C++
#include <cmath>
#include <thread>

// SFML 2D graphics library
#include <SFML/Graphics.hpp>

// Package headers
#include "SfmlVisualizer.hpp"

#include <pathfile.h>

/// This visualizer doesn't support video recording
class CartpoleVisualizer : public SfmlVisualizer
{
 public:

  //! Constructor
  CartpoleVisualizer(unsigned int windowWidth = 2*600, unsigned int windowHeight = 400) :
    SfmlVisualizer( "Cartpole", windowWidth, windowHeight),
    cart_(sf::Vector2f(50.0f, 30.0f)),
    pole_(sf::Vector2f(10.0f, 150.0f)),
    track_(sf::Vector2f(.96f * windowWidth, 10.0f)),
    center_(2.0f),
    sprite_(),
    arrowTexture_(),
    scaleX_(15.0)
  {
    arrowTexture_.loadFromFile(std::string(PACKAGE_PATH) + std::string("/test/cartpole_arrow.png"));
    sprite_.setTexture(arrowTexture_);
    sprite_.setColor(sf::Color(200, 75, 66));
    sprite_.setOrigin(0.0f, arrowTexture_.getSize().y / 2.0f);

    track_.setPosition(0.02f * windowWidth_, 4.0f * windowHeight_ / 5.0f);
    track_.setFillColor(sf::Color(200, 200, 200, 255));

    cart_.setOrigin(25.0f, 15.0f);
    cart_.setFillColor(sf::Color::Black);

    pole_.setOrigin((float) 5.0f, (float) 145.0f);
    pole_.setFillColor(sf::Color(180, 180, 67, 255));

    center_.setOrigin(2.0f, 2.0f);
    center_.setFillColor(sf::Color::Black);
  }

  //! Use the default destructor
  ~CartpoleVisualizer() = default;

  //! Update visualization
  void drawWorld(double x, double angle, double action) {
    loopMutex_.lock();

    if (window_.isOpen()) {
      if(!visualizerIsRunning_)
        throw std::runtime_error("You should call start() first");

      if(x < 0.0) {
        x = scaleX_/2.0 - fmod(-x+scaleX_/2.0, scaleX_);
      }
      if(x > scaleX_) {
        x = -1.5*scaleX_ + fmod(x+scaleX_/2.0, scaleX_);
      }
      float scaledX = (float) x * windowWidth_ / scaleX_ + windowWidth_ / 2.0f;

      float trackY = 4.0f * windowHeight_ / 5.0f;
      // cart
      cart_.setRotation(0.0f);
      cart_.setPosition(scaledX, trackY);
      // pole
      pole_.setPosition(scaledX, trackY);
      pole_.setRotation((float) (-angle * 180.0 / M_PI));
      // center
      center_.setPosition(scaledX, trackY);

      //scale objects
      track_.setScale(1, scale_);
      cart_.setScale(scale_, scale_);
      pole_.setScale(scale_, scale_);
      center_.setScale(scale_, scale_);

      float scale = (float) action / 2.0f * scale_;
      if (action > 0) {
        sprite_.setPosition(scaledX + 25.0f, trackY);
        sprite_.setScale(scale, scale_);
      } else {
        sprite_.setPosition(scaledX - 25.0f, trackY);
        sprite_.setScale(scale, scale_);
      }

      //draw
      window_.clear(sf::Color::White);
      window_.draw(track_);
      window_.draw(sprite_);
      window_.draw(cart_);
      window_.draw(pole_);
      window_.draw(center_);
      window_.display();
    }
    loopMutex_.unlock();
  }

 public:

  // SFML graphics objects
  sf::RectangleShape cart_;
  sf::RectangleShape pole_;
  sf::RectangleShape track_;
  sf::CircleShape center_;
  sf::Sprite sprite_;
  sf::Texture arrowTexture_;
  double scaleX_;
};

#endif // CARTPOLE_VISUALIZER_HPP_
