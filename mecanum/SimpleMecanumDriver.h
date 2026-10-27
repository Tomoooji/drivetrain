#pragma once
#include <utility>
#include <Arduino.h>
#include <Vector2D.h>
#include <AnalogMotorDriver.h>

#if __cplusplus >= 202002L
template <AnalogMotorConcept MD>
  requires requires(MD m) { m.begin(); }
#else
template <class MD>
#endif
class SimpleMecanumDriver {
private:
  enum ID { ID_FL, ID_FR, ID_BL, ID_BR, ID_ALL };
  MD motors_[ID::ID_ALL];
  const float (&speed_ratio_)[ID::ID_ALL];
  Vector2D<float> velocity_;
  float rotation_;

public:
  SimpleMecanumDriver(MD &&mtFL, MD &&mtFR, MD &&mtBL, MD &&mtBR,
                      const float (&speed_ratio)[ID::ID_ALL],
                      Vector2D<float> &&init_velocity = Vector2D<float>(0, 0))
    : motors_{ std::move(mtFL), std::move(mtFR), std::move(mtBL), std::move(mtBR) },
      speed_ratio_{ speed_ratio }, velocity_{ std::move(init_velocity) }, rotation_{ 0.0f } {}

  void begin() {
    for (auto &motor : this->motors_) {
      motor.begin();
    }
  }

  void update() {
    float vx = this->velocity_.getX();
    float vy = this->velocity_.getY();
    this->motors_[ID::ID_FL].write((vy + vx + this->rotation_) * this->speed_ratio_[ID::ID_FL]);
    this->motors_[ID::ID_FR].write((vy - vx - this->rotation_) * this->speed_ratio_[ID::ID_FR]);
    this->motors_[ID::ID_BL].write((vy - vx + this->rotation_) * this->speed_ratio_[ID::ID_BL]);
    this->motors_[ID::ID_BR].write((vy + vx - this->rotation_) * this->speed_ratio_[ID::ID_BR]);
  }

  void setVelocity(const Vector2D<float> velocity, float rotation) {
    this->velocity_ = std::move(velocity);
    this->rotation_ = rotation;
  }
  void setVelocity(Vector2D<float> velocity) {
    this->velocity_ = std::move(velocity);
  }
  const Vector2D<float> &getVelocity() const { return this->velocity_; }
  
  const float &getRotation() const { return this->rotation_; }
};

template<class MD>
using SimpleOmuni4Driver = SimpleMecanumDriver<MD>;