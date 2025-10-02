//
//  pool_pump_rs485.h
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-26.
//  Refactored by Gemini AI 2025-09-26
//

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/uart/uart.h"
#include "common_types.h"

namespace esphome {
namespace pool_pump_rs485 {

// RS-485 Message Sequencing States
enum MsgSequencingStates {
  sendSolarSpeedOn,    // send solarSpeedOn message (if applicable)
  waitSolarSpeedOn,    // wait for reply to solarSpeedOn message
  sendSolarSpeedOff,  // send solarSpeedOff message (if applicable)
  waitSolarSpeedOff,  // wait for reply to solarSpeedOff message
  sendStatusRequest,  // send pump status request message
  waitStatusReply,    // wait for reply to pump status request
  waitForNextPollInterval,    // wait for next polling interval to start
};

//
// RS-485 Message Parsing States for Pump and Controller
//
enum MsgStates {
  expectStart,    // message starts with FF
  expectA5,        // got FF, wait for A5
  expect00,        // got A5, wait for 00
  expectSrc,        // expect 60 (pump) or 10 (controller)
  expectDst,        // expect 60 (pump) or 10 (controller)
  expectCmd,        // various
  expectLen,        // number of data bytes to follow
  expectData,        // data bytes
  expectChkH,        // checksum hi byte
  expectChkL,     // checksum lo byte
  msgComplete
};

enum MsgPrefixes : uint8_t {
  mPrefixA5 = 0xA5,               // marks begin of message
  mProtocolRev0 = 0x00,            // protocol version?
};

// RS-485 Message source and destination ids
enum MsgDeviceIds : uint8_t {
  PumpId = 0x60,
  CtlrId = 0x10,
};

// RS-485 Message Actions (codes used by Intelliflo pump)
enum MsgActions : uint8_t {
  noAction = 0,
  rqstSetSpeed = 1,
  rqstEquipStatus = 2,
  lockDisplay = 4,
  rqstMode = 5,
  turnOnOff = 6,
  rqstStatus = 7,
  runAtGPM = 9,
  invalidAction = 0xFF,
};

// specifies which of the "external programs" (stored in the pump settings)
// to be run.
enum MsgExternalPrograms : uint8_t {
  noExtProg = 0,
  extProg1 = 0x08,
  extProg2 = 0x10,
  extProg3 = 0x80,
  extProg4 = 0x20,
};

enum MsgPumpRegisters : uint16_t {
  programRegister = 0x0321,
};

enum PumpSpeedValues : uint16_t {
  pumpSpeedBySked  =  999,    // set pump speed for pool, solar off
  pumpSpaSolarOff  = 1001,    // set pump speed for spa mode, solar off
  pumpPoolSolarOff = 2401,    // set pump speed for pool, solar off
  pumpPoolSolarOn  = 2701,    // set pump speed for pool, solar on
  pumpSpaSolarOn   = 2702,    // set pump speed for spa mode, solar on
};

//
// RS-485 Message Data Structure, not including preamble (FF 00 FF)
//
static constexpr int MaxMsgData = 255;

struct Message {
  MsgPrefixes prefix;         // marks message start (always 0xA5)
  MsgPrefixes protocolRev;    // protocol version (always 0 for my pump)
  MsgDeviceIds dest;          // message destination
  MsgDeviceIds source;        // message source
  MsgActions action;
  uint8_t length;                // expected data length
  uint8_t actualLen;            // actual received length
  uint8_t data[MaxMsgData];    // received data
  uint16_t checksum;            // expected checksum
  uint16_t actualChecksum;    // computed checksum
};

// preamble before a message starts
static constexpr uint8_t msgPreamble[3] = { 0xFF, 0x00, 0xFF };

// prefix marks the start of a message
static constexpr MsgPrefixes msgPrefix[2] = { mPrefixA5, mProtocolRev0 };

// destination and source of message
static constexpr MsgDeviceIds msgDestSource[2] = { PumpId, CtlrId };

// These messages hold the data portion of the Message struct
static constexpr uint8_t pumpExtProg4On[4] = {
  (programRegister >> 8),     // programRegister MSB
  (programRegister & 0xFF),   // programRegister LSB
  0x00,
  extProg4 };                 // turns on external program 4

static constexpr uint8_t pumpExtProg4Off[4] = {
  (programRegister >> 8),     // programRegister MSB 0x03
  (programRegister & 0xFF),   // programRegister LSB 0x21
  0x00,
  noExtProg };                // turns off any external program running

static constexpr MilliSec PumpPollIntervalMS = 30000;
static constexpr MilliSec ReplyTimeoutMS = 1000;
static constexpr int MinPumpSpeed = 0;
static constexpr int MaxPumpSpeed = 3000;
static constexpr int PIPE_TEMP_VALID_INTERVAL_S = 4 * 60;   // 4 minutes


// --- Class Declaration ---
class PoolPumpRS485 : public esphome::Component, public esphome::uart::UARTDevice {
public:
  // singleton access
  static PoolPumpRS485* getInstance();

  void setup() override;
  void loop() override;

  // --- SETTERS ---
  // Assigns the sensor object for pump RPM.
  void set_rpm_sensor(esphome::sensor::Sensor* sensor) { this->rpmSensor_ = sensor; }
  // Assigns the sensor object for pump power consumption in watts.
  void set_watts_sensor(esphome::sensor::Sensor* sensor) { this->wattsSensor_ = sensor; }
  // Assigns the sensor object for pump flow rate.
  void set_flow_sensor(esphome::sensor::Sensor* sensor) { this->flowSensor_ = sensor; }
  // Assigns the sensor object for pump power percentage.
  void set_power_sensor(esphome::sensor::Sensor* sensor) { this->powerSensor_ = sensor; }
  // Assigns the sensor object for today's total run time.
  void set_run_time_sensor(esphome::sensor::Sensor* sensor) { this->runTimeSensor_ = sensor; }

  // --- GETTERS ---
  // Returns the sensor object for pump RPM.
  esphome::sensor::Sensor* get_rpm_sensor() const { return this->rpmSensor_; }
  // Returns the sensor object for pump power consumption in watts.
  esphome::sensor::Sensor* get_watts_sensor() const { return this->wattsSensor_; }
  // Returns the sensor object for pump flow rate.
  esphome::sensor::Sensor* get_flow_sensor() const { return this->flowSensor_; }
  // Returns the sensor object for pump power percentage.
  esphome::sensor::Sensor* get_power_sensor() const { return this->powerSensor_; }
  // Returns the sensor object for today's total run time.
  esphome::sensor::Sensor* get_run_time_sensor() const { return this->runTimeSensor_; }

  void requestPumpSpeed(long speed);
  long pumpSpeedForSolar(bool solarHeatOn) const;
  bool isPipeTempValid() const;
  void printDebugInfo() const;

  // this component needs to know if the spa is in use to manage the solar
  void set_spa_mode(bool spa_mode) { spa_mode_ = spa_mode; }

private:
  // --- SENSOR MEMBERS ---
  // Private pointers to the sensor objects that we will manage.
  esphome::sensor::Sensor* rpmSensor_{nullptr};            // pool pump RPM
  esphome::sensor::Sensor* wattsSensor_{nullptr};          // pool pump power consumption
  esphome::sensor::Sensor* flowSensor_{nullptr};           // pool pump flow rate gal/min
  esphome::sensor::Sensor* powerSensor_{nullptr};          // pool pump % of max power
  esphome::sensor::Sensor* runTimeSensor_{nullptr};        // today's total pump run time, in hours

  // Constructor
  PoolPumpRS485(esphome::uart::UARTComponent *parent);       // singleton constructor
  static inline PoolPumpRS485* instance_{nullptr};  // pointer to instance

  // --- Private Members ---
  bool spa_mode_{false};
  bool shouldRequestExtPgmOn = false;
  uint8_t shouldRequestExtPgmOff = 0;
  long pumpSpeedRequested = 0;
  MilliSec msLastPumpPoll = 0;
  MilliSec msLastPumpStatusReply = 0;
  MilliSec msPumpStartTime = 0;
  uint16_t lastPumpRPM = 9999;
  uint16_t lastPumpWatts = 9999;
  uint8_t lastPumpFlow = 255;
  uint8_t lastPumpPower = 255;

  // --- Private Method Declarations ---
  void setNextPollToHappenIn(MilliSec duration);
  const char *const seqImage(const MsgSequencingStates state) const;
  MsgStates gotMessageByte(const uint8_t byte, const MsgStates msgState, Message *msg) const;
  Message makeMessage(const MsgActions action, const uint8_t length, const uint8_t *data) const;
  void sendMessage(const Message &msg);
  void printMessage(const Message &msg);
  uint16_t checksumForMessage(const Message &msg) const;
  bool isValidMessage(const Message &msg) const;
  void handlePumpStatusReply(const Message &msg);
  void updatePumpStartTime(const float pumpRPM);

};

}  // namespace pool_pump_rs485
}  // namespace esphome
