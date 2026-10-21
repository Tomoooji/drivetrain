#include <BluePad32.h>
#include <uni.h>

#include "SimpleMecanumDriver.h"

#include <AccelMotorDriver.h>
#include <Vector2D.h>

namespace Pins {
constexpr uint8_t dc_omuni4_FL[] = { 19, 21 };
constexpr uint8_t dc_omuni4_FR[] = { 27, 26 };
constexpr uint8_t dc_omuni4_BR[] = { 33, 25 };
constexpr uint8_t dc_omuni4_BL[] = { 16, 17 };
constexpr uint8_t led_builtin = 2;
}  // namespace Pins

namespace Gains {
constexpr int omuni4_accel = 2;
constexpr float omuni4_speed_ratio[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
}  // namespace Gains

SimpleOmuni4Driver<AccelMotor> Omuni4{
  AccelMotor(Pins::dc_omuni4_FL, Gains::omuni4_accel),
  AccelMotor(Pins::dc_omuni4_FR, Gains::omuni4_accel),
  AccelMotor(Pins::dc_omuni4_BL, Gains::omuni4_accel),
  AccelMotor(Pins::dc_omuni4_BR, Gains::omuni4_accel),
  Gains::omuni4_speed_ratio
};

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void setup() {
  Serial.begin(115200);
  Omuni4.begin();
  pinMode(led_builtin, OUTPUT);
  
  BP32.enableVirtualDevice(false);
  BP32.forgetBluetoothKeys();
  BP32.setup(&onConnectedController, &onDisconnectedController);
}


void loop() {
  ControllerPtr ctrl = findController();
  digitalWrite(2, LOW);
  if (ctrl != nullptr) {
    digitalWrite(2, HIGH);
    //dumpGamepad(ctrl);
    Omuni4.setVelocity(
      Vector2D<float>(ctrl->axisX() * 0.5, -ctrl->axisY() * 0.5),
      (ctrl->throttle() - ctrl->brake()) / 8);
  }// else {
    //Omuni4.setVelocity(Vector2D<float>(0.0f, 0.0f), 0.0f);
  //}
  Omuni4.update();
  delay(10);
  Serial.print(Omuni4.getVelocity().X());
  Serial.print(",");
  Serial.print(Omuni4.getVelocity().Y());
  Serial.print(",");
  Serial.print(Omuni4.getRotation());
  Serial.println();
}

// コントローラー接続処理
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                    ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

//コントローラー接続解除処理
void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }
  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

//デバッグ用
void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(), ctl->dpad(), ctl->buttons(), ctl->axisX(), ctl->axisY(),
    ctl->axisRX(), ctl->axisRY(), ctl->brake(), ctl->throttle(), ctl->miscButtons(),
    ctl->gyroX(), ctl->gyroY(), ctl->gyroZ(), ctl->accelX(), ctl->accelY(), ctl->accelZ());
}

ControllerPtr findController() {
  if (BP32.update()) {
    for (auto myController : myControllers) {
      if (myController && myController->isConnected() && myController->hasData()) {
        if (myController->isGamepad()) {
          return myController;
        } else {
          Serial.println("Unsupported controller");
        }
      }
    }
  }
  return nullptr;
}
