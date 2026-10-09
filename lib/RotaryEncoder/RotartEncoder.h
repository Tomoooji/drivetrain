/**
 * @file RotartEncoder.h
 * @brief ロータリーエンコーダーの入力クラス
 * @details ESP32用とArduino用で分岐している
 *          InputDeviceに同じものがある
 * @author tomoooji
 * @date 2026-10-09
 */

#pragma once
#include <Arduino.h>

/**
 * @brief ロータリーエンコーダーの入力タイプ
 * @details
 *  SINGLEEDGE : A相の立ち上がりのみ検知
 *  HALFQUAD : A相の立ち上がりと立下りを検知
 *  FULLQUAD : A相とB相の立ち上がりと立下りを検知
 */
enum ENCODER_TYPE { SINGLEEDGE = 1, HALFQUAD = 2, FULLQUAD = 4 };

#ifdef ESP32
#include <ESP32Encoder.h>

/**
 * @class Encoder
 * @brief 2相インクリメンタル型ロータリーエンコーダーの入力クラス(ESP32用)
 * @details ESP32Controllerのラッパークラス
 *
 * @tparam TYPE パルス入力の検知方法
 */
template <ENCODER_TYPE TYPE>
class Encoder {
private:
  ESP32Encoder encoder_;
  const uint8_t &pinA;
  const uint8_t &pinB;
  const bool pulluped;
  const int counts_per_revolution;

public:
  /**
   * @brief Encoder オブジェクトを作成
   * 
   * @param pinA A相のピンの参照
   * @param pinB A相のピンの参照
   * @param pullup 内蔵プルアップの有効化
   * @param counts_per_revolution 一周あたりのパルス数
   */
  Encoder(const uint8_t &pinA, const uint8_t &pinB, const bool pullup = true, const int counts_per_revolution = 0)
      : encoder_(), pinA(pinA), pinB(pinB), pulluped(pullup), counts_per_revolution(counts_per_revolution * TYPE) {}

  /**
   * @brief Encoder オブジェクトを作成(委譲コンストラクタ)
   * 
   * @param pins ピン配列の参照
   * @param pullup 内蔵プルアップの有効化
   * @param counts_per_revolution 一周あたりのパルス数
   */
  Encoder(const uint8_t (&pins)[2], const bool pullup = true, const int counts_per_revolution = 0)
      : Encoder(pins[0], pins[1], pullup, counts_per_revolution) {}

  /** @brief ピンの設定を行う初期化関数 */
  void begin() {
    if (this->pulluped)
      ESP32Encoder::useInternalWeakPullResistors = puType::up;
    if constexpr (TYPE == SINGLEEDGE) {
      this->encoder_.attachSingleEdge(this->pinA, pinB);
    } else if constexpr (TYPE == HALFQUAD) {
      this->encoder_.attachHalfQuad(this->pinA, pinB);
    } else if constexpr (TYPE == FULLQUAD) {
      this->encoder_.attachFullQuad(this->pinA, pinB);
    }
    this->reset();
  }

  /** @brief カウントのリセット関数 */
  void reset() { this->encoder_.clearCount(); }

  /** @brief 直前のリセット以降に回転した量を取得する関数 */
  int read() { return this->encoder_.getCount(); }

  /** @brief 回転量を角度(度数法)として読む関数 */
  float getAngleDeg() {
    if (this->counts_per_revolution) {
      return static_cast<float>(this->read()) / this->counts_per_revolution * 360.0f;
    } else {
      return 0.0f;
    }
  }

  /** @brief 回転量を角度(ラジアン)として読む関数 */
  float getAngleRad() {
    if (this->counts_per_revolution) {
      return static_cast<float>(this->read()) / this->counts_per_revolution * TWO_PI;
    } else {
      return 0.0f;
    }
  }
};

#else

/**
 * @class Encoder
 * @brief 2相インクリメンタル型ロータリーエンコーダーの入力クラス(Arduino用)
 *
 * @tparam ID 割り込み処理を複数のインスタンスで行うためのID
 * @tparam TYPE パルス入力の検知方法
 */
template <uint8_t ID, ENCODER_TYPE TYPE>
class EncoderClass {
private:
  inline static EncoderClass<ID, TYPE> *instance;
  const uint8_t &pinA;
  const uint8_t &pinB;
  const bool pulluped;
  const int counts_per_revolution;
  volatile int count;

  /** @brief パルスから回転量を求めるISR */
  static void countISR() {
    instance->count += (digitalRead(instance->pinA) == digitalRead(instance->pinB) ? 1 : -1);
  }

public:
  /**
   * @brief EncoderClass オブジェクトを作成
   * 
   * @param pinA A相のピンの参照
   * @param pinB A相のピンの参照
   * @param pullup 内蔵プルアップの有効化
   * @param counts_per_revolution 一周あたりのパルス数
   */
  EncoderClass(const uint8_t &pinA, const uint8_t &pinB, const bool pullup = true, const int counts_per_revolution = 0)
      : pinA(pinA), pinB(pinB), pulluped(pullup), counts_per_revolution(counts_per_revolution * TYPE), count(0) {
    instance = this;
  }

  /**
   * @brief EncoderClass オブジェクトを作成(委譲コンストラクタ)
   * 
   * @param pins ピン配列の参照
   * @param pullup 内蔵プルアップの有効化
   * @param counts_per_revolution 一周あたりのパルス数
   */
  EncoderClass(const uint8_t (&pins)[2], const bool pullup = true, const int counts_per_revolution = 0)
      : EncoderClass(pins[0], pins[1], pullup, counts_per_revolution) {}

  /** @brief ピンの設定を行う初期化関数 */
  void begin() {
    pinMode(this->pinA, this->pulluped ? INPUT_PULLUP : INPUT);
    pinMode(this->pinB, this->pulluped ? INPUT_PULLUP : INPUT);
    if constexpr (TYPE == SINGLEEDGE) {
      attachInterrupt(digitalPinToInterrupt(this->pinA), countISR, RISING);
    } else if constexpr (TYPE == HALFQUAD) {
      attachInterrupt(digitalPinToInterrupt(this->pinA), countISR, CHANGE);
    } else if constexpr (TYPE == FULLQUAD) {
      attachInterrupt(digitalPinToInterrupt(this->pinA), countISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(this->pinB), countISR, CHANGE);
    }
    this->reset();
  }

  /** @brief カウントのリセット関数 */
  void reset() { this->count = 0; }

  /** @brief 直前のリセット以降に回転した量を取得する関数 */
  int read() { return this->count; }

  /** @brief 回転量を角度(度数法)として読む関数 */
  float getAngleDeg() {
    if (this->counts_per_revolution) {
      return static_cast<float>(this->read()) / this->counts_per_revolution * 360.0f;
    } else {
      return 0.0f;
    }
  }

  /** @brief 回転量を角度(ラジアン)として読む関数 */
  float getAngleRad() {
    if (this->counts_per_revolution) {
      return static_cast<float>(this->read()) / this->counts_per_revolution * TWO_PI;
    } else {
      return 0.0f;
    }
  }
};

/**
 * @brief EncoderClassの実体化用マクロ
 * @details EncoderClassのIDを宣言時に自動で割り振ってくれる
 */
#define Encoder(ENCODER_TYPE) EncoderClass<__COUNTER__, ENCODER_TYPE>

#endif
