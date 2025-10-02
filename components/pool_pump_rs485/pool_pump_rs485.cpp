//
//  pool_pump_rs485.cpp
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-26.
//

#include "pool_pump_rs485.h"

namespace esphome {
namespace pool_pump_rs485 {

PoolPumpRS485* PoolPumpRS485::getInstance() {
  return instance_;
}

// constructor
PoolPumpRS485::PoolPumpRS485(esphome::uart::UARTComponent *parent) : esphome::uart::UARTDevice(parent) {
  instance_ = this;
}

//
// setup() -- one time setup
//
void PoolPumpRS485::setup() {
  // init a few things
  msLastPumpPoll = millis();              // init time of "previous" polling interval
}

//
// loop() -- main loop, called about every 16 milliseconds
//
void PoolPumpRS485::loop() {
  static MsgSequencingStates msgSequenceState = waitForNextPollInterval;
  static MsgStates msgState = expectStart;
  static MilliSec msReplyWaitStart = 0;
  static Message msg;

  const MilliSec msNow = millis();

  switch (msgSequenceState) {
    case waitForNextPollInterval:
    {
      const MilliSec msElapsed = msNow - msLastPumpPoll;
      if (msElapsed >= PumpPollIntervalMS) {
        msgSequenceState = sendSolarSpeedOn;
        msLastPumpPoll = msNow;
      }
    }
      break;

    case sendSolarSpeedOn:
      if (shouldRequestExtPgmOn) {
        // transmit a request for pump to switch on to solar speed
        const auto sendMsg = makeMessage(rqstSetSpeed,
                                         sizeof(pumpExtProg4On),
                                         pumpExtProg4On);
        printMessage(sendMsg);
        sendMessage(sendMsg);

        msgState = expectStart;
        msgSequenceState = waitSolarSpeedOn;
        msReplyWaitStart = msNow;
      } else {
        msgSequenceState = sendSolarSpeedOff;
      }
      break;

    case sendSolarSpeedOff:
      if (shouldRequestExtPgmOff > 0) {
        // decrement counter
        shouldRequestExtPgmOff--;

        // transmit a request for pump to switch off solar speed
        const auto sendMsg = makeMessage(rqstSetSpeed,
                                         sizeof(pumpExtProg4On),
                                         pumpExtProg4Off);
        printMessage(sendMsg);
        sendMessage(sendMsg);

        msgState = expectStart;
        msgSequenceState = waitSolarSpeedOff;
        msReplyWaitStart = msNow;
      } else {
        msgSequenceState = sendStatusRequest;
      }
      break;

    case sendStatusRequest:
      // transmit a pump status request via RS-485 serial
      sendMessage( makeMessage(rqstStatus, 0, NULL) );

      msgState = expectStart;
      msgSequenceState = waitStatusReply;
      msReplyWaitStart = msNow;
      break;

    case waitSolarSpeedOn:
    case waitSolarSpeedOff:
    case waitStatusReply:
      //------------------------------------
      // wait for more bytes if message is incomplete
      if (msgState != msgComplete) {
        // check for timeouts before we continue waiting for incoming message
        MilliSec msReplyWait = msNow - msReplyWaitStart;
        if (msReplyWait >= ReplyTimeoutMS) {
          // if we timed out on the pump status message, then report RPM as unknown
          if (msgSequenceState == waitStatusReply) {
            rpmSensor_->publish_state(NAN);
            wattsSensor_->publish_state(NAN);
            flowSensor_->publish_state(NAN);
            powerSensor_->publish_state(NAN);

            // reset the total run time counter for this polling cycle
            msLastPumpStatusReply = 0;
          }
          msgState = msgComplete;        // timed out, give up, call it done and move on
          ESP_LOGD("custom","***** WARNING: timed out waiting for RS-485 message (seq:%d)", msgSequenceState);
          msg.source = CtlrId;
          msg.dest   = CtlrId;
          msg.action = invalidAction;     // force it to fail the check later
          msg.checksum = 0;
        }
      }

      //------------------------------------
      // if we don't have a complete message in our receive buffer,
      // then check to see if there are available incoming data bytes
      while (msgState != msgComplete) {
        MilliSec msStart = millis();
        const bool isAvailable = available();
        MilliSec msElapsed = millis() - msStart;

        if (msElapsed >= 10) {
          ESP_LOGD("custom","***** WARNING: RS-485 available() took %lu ms", msElapsed);
          break;     // taking too long, continue on next loop() call
        }

        if (isAvailable) {
          msStart = millis();
          uint8_t byte = read();
          msgState = gotMessageByte(byte, msgState, &msg);
          msElapsed = millis() - msStart;
          if (msElapsed >= 10) {
            ESP_LOGD("custom","***** WARNING: RS-485 read()+gotMB() took %lu ms", msElapsed);
            break;     // taking too long, continue on next loop() call
          }
        } else {
          break;
        }
      }

      //-------------------------------------
      // process the complete message
      //
      if (msgState == msgComplete) {
        // debug print out the message received
        printMessage(msg);

        // we received a complete reply message (or timed out)
        switch (msg.action) {
          case rqstSetSpeed:
            // NOT IMPLEMENTED YET -- verify the pump did what we requested,
            // if it didn't there's not much we can do just report errors
            break;

          case rqstStatus:
            // message won't be valid if we timed out earlier while waiting for it
            if (isValidMessage(msg) && msg.action == rqstStatus) {
              handlePumpStatusReply(msg);
            }
            break;

          default:
            // invalid or not of interest message
            ESP_LOGD("custom","***** WARNING: unexpected RS-485 message received");
            break;
        }

        // we processed the message, transition to next state
        switch (msgSequenceState) {
          case waitSolarSpeedOn:
            msgSequenceState = sendStatusRequest;        // next sequence state
            break;

          case waitSolarSpeedOff:
            msgSequenceState = sendStatusRequest;        // next sequence state
            break;

          case waitStatusReply:
            msgSequenceState = waitForNextPollInterval;  // next sequence state
            break;
        }
      }
  } // switch

  //        const MilliSec msElapsed = millis() - msNow;
  //        if (msElapsed >= 20) {
  //            ESP_LOGD("custom","***** WARNING: RS-485 loop() too slow (%lu ms, seq: %s, msg: %d)",
  //                     msElapsed, seqImage(msgSequenceState), msgState);
  //        }
}

const char* const PoolPumpRS485::seqImage(MsgSequencingStates state) const {
  switch (state) {
    case sendSolarSpeedOn: return ("sendSolarSpeedOn");
    case waitSolarSpeedOn: return ("waitSolarSpeedOn");
    case sendSolarSpeedOff: return ("sendSolarSpeedOff");
    case waitSolarSpeedOff: return ("waitSolarSpeedOff");
    case sendStatusRequest: return ("sendStatusRequest");
    case waitStatusReply: return ("waitStatusReply");
    case waitForNextPollInterval: return ("waitForNextPollInterval");
    default: return "???";
  }
}

//
// requestPumpSpeed -- needs to check if the pump is running, turn it on
//                     if needed, then set it to the desired RPM.
//
void PoolPumpRS485::requestPumpSpeed(long speed) {
  ESP_LOGD("custom","----- requestPumpSpeed: %ld", speed);

  if (speed > MaxPumpSpeed || speed < MinPumpSpeed) {
    return;         // ignore invalid requests
  }

  shouldRequestExtPgmOn  = (speed == pumpPoolSolarOn ||
                            speed == pumpSpaSolarOn ||
                            speed == pumpSpaSolarOff);

  if (speed == pumpPoolSolarOff) {
    shouldRequestExtPgmOff = 3;     // send this message 3 times
  } else {
    shouldRequestExtPgmOff = 0;     // don't send this message
  }

  if (shouldRequestExtPgmOn || shouldRequestExtPgmOff > 0) {
    // adjust last pump poll time so that next poll occurs in 1 second
    // that poll will also set the new pump speed
    setNextPollToHappenIn(1000);        // 1 sec in future
  }

  ESP_LOGD("custom","----- requestPumpSpeed: %ld", speed);
  pumpSpeedRequested = speed;
}

//
// pumpSpeedForMode -- returns the correct pump speed setting given the
//                     solar on/off setting and pool/spa mode setting
//
long PoolPumpRS485::pumpSpeedForSolar(bool solarHeatOn) const {
  if (spa_mode_) {
    return solarHeatOn ? pumpSpaSolarOn : pumpSpaSolarOff;
  } else {
    return solarHeatOn ? pumpPoolSolarOn : pumpPoolSolarOff;
  }
}

//
// setNextPollToHappenIn -- fix polling to happen next in durationMS
// if time to the next poll less than duration, no change made
//
void PoolPumpRS485::setNextPollToHappenIn(MilliSec duration) {
  MilliSec now = millis();
  MilliSec msUntilNextPoll = (msLastPumpPoll + PumpPollIntervalMS) - now;
  if (msUntilNextPoll <= duration) {
    return;     // it's going to happen soon anyway, leave it alone
  }

  ESP_LOGD("custom","+++++ setNextPollToHappenIn: %0.3fs", duration/1000.0);
  MilliSec msAtDesiredNextPoll = now + duration;
  msLastPumpPoll = msAtDesiredNextPoll - PumpPollIntervalMS;
}

//
// gotMessageByte -- updates received message data according to state
//
MsgStates PoolPumpRS485::gotMessageByte(const uint8_t byte,
                                        const MsgStates msgState,
                                        Message* msg) const {
  switch (msgState) {
    case expectStart:
      if (byte == 0xFF) {
        return expectA5;
      }
      return expectStart;

    case expectA5:
      if (byte == mPrefixA5) {
        // we got the prefix of a message, zero out the
        // previous message in the buffer and start new
        std::memset(msg, 0, sizeof(Message));

        msg->prefix = mPrefixA5;
        return expect00;
      }
      return expectStart;

    case expect00:
      if (byte == mProtocolRev0) {
        msg->protocolRev = mProtocolRev0;
        return expectDst;
      }
      return expectStart;

    case expectDst:
      if (byte == PumpId || byte == CtlrId) {
        msg->dest = static_cast<MsgDeviceIds>(byte);
        return expectSrc;
      }
      break;

    case expectSrc:
      if (byte == PumpId || byte == CtlrId) {
        msg->source = static_cast<MsgDeviceIds>(byte);
        return expectCmd;
      }
      break;

    case expectCmd:
      msg->action = static_cast<MsgActions>(byte);
      return expectLen;

    case expectLen:
      msg->length = byte;        // #bytes expected in message
      msg->actualLen = 0;        // #bytes actually recieved
                                 // if no data then checksum is next
      return (msg->length == 0) ? expectChkH : expectData;

    case expectData:
      if (msg->actualLen > msg->length || msg->actualLen >= MaxMsgData) {
        return expectStart;        // invalid length, start over
      }
      msg->data[msg->actualLen++] = byte;

      // if got all the data bytes, checksum next
      return (msg->actualLen == msg->length) ? expectChkH : expectData;

    case expectChkH:
      msg->checksum = byte << 8;
      return expectChkL;

    case expectChkL:
      msg->checksum |= byte;
      msg->actualChecksum = checksumForMessage(*msg);
      return msgComplete;
  }

  return expectStart;        // invalid state, start over
}

Message PoolPumpRS485::makeMessage(const MsgActions action,
                                   const uint8_t length,
                                   const uint8_t* data) const {
  Message msg;
  std::memset(&msg, 0, sizeof(msg));

  msg.prefix = mPrefixA5;
  msg.protocolRev = mProtocolRev0;
  msg.dest   = PumpId;        // we only make messages to the pump
  msg.source = CtlrId;        // we only make messages for the controller
  msg.action = action;
  msg.length = length;
  msg.actualLen = length;     // may be 0

  for (int i=0; i<length; i++) {
    msg.data[i] = data[i];
  }

  msg.checksum = checksumForMessage(msg);
  msg.actualChecksum = msg.checksum;
  return msg;
}

//
// sendMessage -- send message via RS-485 serial to the pump
//
void PoolPumpRS485::sendMessage(const Message& msg) {
  // send data to the UART for RS-485 transmission to pump
  write_array(msgPreamble, sizeof(msgPreamble));

  write(static_cast<uint8_t>(msg.prefix));
  write(static_cast<uint8_t>(msg.protocolRev));
  write(static_cast<uint8_t>(msg.dest));
  write(static_cast<uint8_t>(msg.source));
  write(static_cast<uint8_t>(msg.action));
  write(msg.length);

  if (msg.length > 0) {
    write_array(msg.data, msg.length);
  }

  write(msg.checksum >> 8);       // MSB
  write(msg.checksum & 0xFF);     // LSB
}

void PoolPumpRS485::printMessage(const Message& msg) {
  char str[255];

  if (msg.source == PumpId && msg.dest == CtlrId) {
    strcpy(str, "<<<<< RS-485: P-->C");
  } else if (msg.source == CtlrId && msg.dest == PumpId) {
    strcpy(str, ">>>>> RS-485: C-->P");
  } else {
    sprintf(str,"????? RS-485: x%02X->x%02X", msg.source, msg.dest);
  }

  char* nextP = str + strlen(str);
  switch (msg.action) {
    case rqstSetSpeed:
      sprintf(nextP, " 1.SetSpeed");         break;
    case rqstEquipStatus:
      sprintf(nextP, " 2.EquipStat");        break;
    case lockDisplay:
      sprintf(nextP, " 4.Panel On/Off");     break;
    case rqstMode:
      sprintf(nextP, " 5.Time Bcst");        break;
    case turnOnOff:
      sprintf(nextP, " 6.Pump On/Off");      break;
    case rqstStatus:
      sprintf(nextP, " 7.Status");           break;
    case runAtGPM:
      sprintf(nextP, " 9.Run @GPM");         break;
    default:
      sprintf(nextP, " Action: %u(x%02X)", msg.action, msg.action);
  }
  sprintf(str+strlen(str), " (#%u)", msg.length);

  for (int i=0; i<msg.length; i++) {
    sprintf(str+strlen(str), " %02X", msg.data[i]);
  }

  if (msg.checksum == msg.actualChecksum) {
    sprintf(str+strlen(str), " (x%04X) ✅", msg.checksum);
  } else {
    sprintf(str+strlen(str), " (x%04X != %04X) ⛔️ERR", msg.checksum, msg.actualChecksum);
  }
  ESP_LOGD("custom", str);
}

//
// computeChecksum for message
//
uint16_t PoolPumpRS485::checksumForMessage(const Message& msg) const {
  uint16_t checksum = msg.prefix;
  checksum += uint16_t(msg.protocolRev);
  checksum += uint16_t(msg.dest);
  checksum += uint16_t(msg.source);
  checksum += uint16_t(msg.action);
  checksum += uint16_t(msg.length);

  for (int i=0; i<msg.length; i++) {
    checksum += uint16_t(msg.data[i]);
  }
  return checksum;
}

bool PoolPumpRS485::isValidMessage(const Message& msg) const {
  if (msg.source == CtlrId && msg.dest == CtlrId && msg.action == invalidAction) {
    return false;           // this message has been zeroed out due to timeout error
  }

  // verify checksum is correct
  return msg.checksum == msg.actualChecksum;
}

//-----------------------------------------------------
// handlePumpStatusReply - got status from pump
//-----------------------------------------------------
void PoolPumpRS485::handlePumpStatusReply(const Message& msg) {
  //---------------------------
  // extract Pump RPM and Watts
  //---------------------------
  uint16_t curPumpWatts = uint16_t((msg.data[3] << 8) | msg.data[4]);
  uint16_t curPumpRPM   = uint16_t((msg.data[5] << 8) | msg.data[6]);
  uint8_t curPumpFlow  = msg.data[7];       // in gallons/minute
  uint8_t curPumpPower = msg.data[8];       // percentage of max

  //---------------------------
  // publish RPM and Watts sensor data
  //---------------------------
  if (curPumpRPM != lastPumpRPM) {
    // RPM changed, publish an update
    rpmSensor_->publish_state(curPumpRPM);
    lastPumpRPM = curPumpRPM;
  }
  if (abs(curPumpWatts-lastPumpWatts) >= 5) {
    // Watts changed, publish an update
    wattsSensor_->publish_state(curPumpWatts);
    lastPumpWatts = curPumpWatts;
  }
  if (curPumpFlow != lastPumpFlow) {
    // Flow changed, publish an update
    flowSensor_->publish_state(curPumpFlow);
    lastPumpFlow = curPumpFlow;
  }
  if (curPumpPower != lastPumpPower) {
    // Power changed, publish an update
    powerSensor_->publish_state(curPumpPower);
    lastPumpPower = curPumpPower;
  }

  //---------------------------
  // update pump start time tracker -- needs to know when pumping
  updatePumpStartTime(curPumpRPM);

  //---------------------------
  // compute how long the pump has been running for this polling loop
  //---------------------------
  MilliSec msNow = millis();

  msLastPumpStatusReply = msNow;  // start next period
}

void PoolPumpRS485::updatePumpStartTime(const float pumpRPM) {
  // track pump turn on/off time
  const bool isPumpSpeedValid = !std::isnan(pumpRPM);
  const bool isPumpRunning = isPumpSpeedValid && (pumpRPM >= 1000);

  if (isPumpRunning) {
    // pump is RUNNING
    if (msPumpStartTime == 0) {
      // we have no pump start time, so it must have
      // been OFF previously, and it has just started up,
      // record the start time
      msPumpStartTime = millis();
    }
  } else {
    // pump is OFF, we have no start time
    msPumpStartTime = 0;
  }
}

bool PoolPumpRS485::isPipeTempValid() const {
  if (msPumpStartTime == 0) {
    return false;       // pump is not running
  }
  const MilliSec msElapsed = millis() - msPumpStartTime;
  const float secsPumpOn = msElapsed / 1000.0;

  return secsPumpOn >= PIPE_TEMP_VALID_INTERVAL_S;
}

// called from yaml in a lambda, as desired to debug hours data
void PoolPumpRS485::printDebugInfo() const {
  ESP_LOGD("custom","printDebugInfo()");
}

}  // namespace pool_pump_rs485
}  // namespace esphome
