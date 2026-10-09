#pragma once
#include <Arduino.h>

template <typename T = float>
class PID {
public:
  struct Gain {
    T kp;
    T ki;
    T kd;
  };

private:
  const Gain &gain;
  const T &integral_limit;
  T target;
  T integral;
  T last_error;
  unsigned long last_time;

public:
  PID(const Gain &gain, const T &integral_limit)
      : gain(gain), integral_limit(integral_limit), target(0), integral(0),
        last_error(0), last_time(millis()) {}

  void reset() {
    integral = 0;
    last_error = 0;
    last_time = millis();
  }

  T update(T input, float dt = 0) {
    if (dt <= 0) {
      unsigned long current_time = millis();
      dt = (current_time - last_time) /
           1000.0; // Convert milliseconds to seconds
      last_time = current_time;
    }
    T error = target - input;

    // Limit the integral term to prevent windup
    integral = integral + error * dt;
    if (integral_limit < integral) {
      integral = integral_limit;
    } else if (-integral_limit > integral) {
      integral = -integral_limit;
    }

    T derivative = (error - last_error) / dt;
    last_error = error;

    return gain.kp * error + gain.ki * integral + gain.kd * derivative;
  }

  void setTarget(T target) {
    this->target = target;
    last_time = millis();
  }
};