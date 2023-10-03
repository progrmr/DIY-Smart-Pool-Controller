#include "esphome.h"

#define MaxMsgData 255
#define PumpPollIntervalMS 15000
#define PumpId 0x60
#define CtlrId 0x10
#define ReplyTimeOutMS 1000

class PoolPumpRS485 : public Component, public UARTDevice {
 public:
  PoolPumpRS485(UARTComponent *parent) : UARTDevice(parent) {}
 
  typedef unsigned long MilliSec;

  Sensor* rpmSensor = new Sensor();		// pool pump RPM
  Sensor* wattsSensor = new Sensor();		// pool pump power consumption
  Sensor* peakCurrentSensor = new Sensor();	// solar diverter valve actuator motor current draw
  Sensor* actuationTimeSensor = new Sensor();	// solar diverter valve actuation time

  static PoolPumpRS485* instance;		// singleton instance

  // RS-485 Message Sequencing States
  enum MsgSequencingStates {
    sendSolarSpeedOn,	// send solarSpeedOn message (if applicable)
    waitSolarSpeedOn,	// wait for reply to solarSpeedOn message
    sendSolarSpeedOff,  // send solarSpeedOff message (if applicable)
    waitSolarSpeedOff,  // wait for reply to solarSpeedOff message
    sendStatusRequest,  // send pump status request message 
    waitStatusReply,    // wait for reply to pump status request
    waitForNextPollInterval,	// wait for next polling interval to start
  };

  //
  // RS-485 Message Parsing States for Pump and Controller
  // 
  enum MsgStates { 
    expectStart,	// message starts with FF
    expectA5,		// got FF, wait for A5
    expect00,		// got A5, wait for 00
    expectSrc,		// expect 60 (pump) or 10 (controller)
    expectDst,		// expect 60 (pump) or 10 (controller)
    expectCmd,		// various
    expectLen,		// number of data bytes to follow
    expectData,		// data bytes
    expectChkH,		// checksum hi byte
    expectChkL, 	// checksum lo byte
    msgComplete	
  };

  //
  // RS-485 Message Actions
  //
  enum MsgActions {
    noAction = 0,
    requestSetSpeed = 1,
    requestStatus = 7,
  };

  //
  // RS-485 Message Data Structure, not including header
  //
  struct Message {
    uint8_t dest;
    uint8_t source;
    uint8_t action;
    uint8_t length;		// expected data length
    uint8_t actualLen;		// actual received length
    uint8_t data[MaxMsgData];	// received data
    uint16_t checksum;		// expected checksum
    uint16_t actualChecksum;	// computed checksum
  }; 

  const uint8_t msgHeader[4] = { 0x00, 0xFF, 0xA5, 0x00 };	// starts a message transmittion
  const uint8_t msgDestination[2] = { PumpId, CtlrId };		// destination and source of message

  // format: action, length, data, CRChi, CRClo 
  const uint8_t pumpStatusRequest[4] = { requestStatus, 0x00, 0x01, 0x1C };
  const uint8_t pumpSolarSpeedOn [8] = { requestSetSpeed, 0x04, 0x03, 0x21, 0x00, 0x20, 0x01, 0x5E };
  const uint8_t pumpSolarSpeedOff[8] = { requestSetSpeed, 0x04, 0x03, 0x21, 0x00, 0x00, 0x01, 0x3E };

  bool shouldRequestSolarSpeedOn = false;
  bool shouldRequestSolarSpeedOff = false;

  MilliSec msLastPumpPoll = 0;

  uint16_t lastPumpRPM = 9999;
  uint16_t lastPumpWatts = 9999;

  float peakActuatorCurrent = 0.0;
  MilliSec msActuatorStartTime = 0;
  float actuationTime = 0.0;		// time in seconds

  // 
  // setup() -- one time setup
  //
  void setup() override {
    // not much to do here
    msLastPumpPoll = millis();		// init time of "previous" polling interval
  }

  //
  // loop() -- main loop, called about every 16 milliseconds
  //
  void loop() override {
    static MsgSequencingStates msgSequenceState = waitForNextPollInterval;
    static MsgStates msgState = expectStart;
    static MilliSec msReplyWaitStart = 0;
    static Message msg;

    switch (msgSequenceState) {
       case waitSolarSpeedOn:
       case waitSolarSpeedOff:
       case waitStatusReply:
         //-------------------------------
	 // waiting for incoming message
         //-------------------------------
	 while (available()) {
	   uint8_t byte = read();
	   msgState = gotMessageByte(byte, msgState, &msg);

	   if (msgState == msgComplete) {
	     if (checkReceivedMessage(&msg)) {
	       // we received a reply message
	       switch (msg.action) {
	         case requestSetSpeed:
	           // NOT IMPLEMENTED YET -- verify the pump did what we requested, 
	           // if it didn't there's not much we can do just report errors
	           break;

	         case requestStatus:
	           handlePumpStatus(&msg);
                   break;
                 
                 default:
	           // invalid or not of interest message
	           ESP_LOGD("custom","***** WARNING: unexpected RS-485 message received");
	       }
	     }
	   }
         }
         if (msgState != msgComplete) {
           MilliSec elapsed = millis() - msReplyWaitStart;
           if (elapsed >= ReplyTimeOutMS) {
             msgState = msgComplete;		// timed out, give up, call it done and move on
             ESP_LOGD("custom","***** WARNING: timed out waiting for RS-485 message (seq:%d)", msgSequenceState);
           }
         }
         break;

       default:
         break;
    }

    switch (msgSequenceState) {
       case waitForNextPollInterval:
    	 if (isPumpPollingTime()) {
           msgSequenceState = sendSolarSpeedOn;
           msLastPumpPoll = millis();
         }
         break;   

       case sendSolarSpeedOn:
         if (shouldRequestSolarSpeedOn) {
           // transmit a request for pump to switch on to solar speed
           sendMessage(pumpSolarSpeedOn, sizeof(pumpSolarSpeedOn));
           msgSequenceState = waitSolarSpeedOn;
           msgState = expectStart;
           msReplyWaitStart = millis();
         } else {
           msgSequenceState = sendSolarSpeedOff;
         }
         break;

       case sendSolarSpeedOff:
         if (shouldRequestSolarSpeedOff) {
           // transmit a request for pump to switch off solar speed
           sendMessage(pumpSolarSpeedOff, sizeof(pumpSolarSpeedOff));
           msgSequenceState = waitSolarSpeedOff;
           msgState = expectStart;
           msReplyWaitStart = millis();
         } else {
           msgSequenceState = sendStatusRequest;
         }
         break;

       case sendStatusRequest:
         // transmit a pump status request via RS-485 serial
         sendMessage(pumpStatusRequest, sizeof(pumpStatusRequest));
         msgSequenceState = waitStatusReply;
         msgState = expectStart;
         msReplyWaitStart = millis();
         break;
         
       case waitSolarSpeedOn:
         if (msgState == msgComplete) {
           msgSequenceState = sendStatusRequest;		// next sequence state
         }
         break;

       case waitSolarSpeedOff:
         if (msgState == msgComplete) {
           msgSequenceState = sendStatusRequest;		// next sequence state
         }
         break;

       case waitStatusReply:
         if (msgState == msgComplete) {
           msgSequenceState = waitForNextPollInterval;		// next sequence state
         }
         break;
    }
  }

  //
  // setPumpSpeed -- needs to check if the pump is running, turn it on
  //                 if needed, then set it to the desired RPM.
  //
  void setPumpSpeed(long speed) {
    if (speed == 2400) {
      // request pump revert to normal programmed speed
      shouldRequestSolarSpeedOn  = false;
      shouldRequestSolarSpeedOff = true;

    } else if (speed == 2700) {
      // request pump switch to solar on speed
      shouldRequestSolarSpeedOn  = true;
      shouldRequestSolarSpeedOff = false;

    } else {
      // no requests, stop sending these messages
      shouldRequestSolarSpeedOn  = false;
      shouldRequestSolarSpeedOff = false;
    }
  }

  //
  // setActuatorCurrent
  //
  void setActuatorCurrent(float amps) {
    if (amps > 0 && msActuatorStartTime == 0) {
      msActuatorStartTime = millis();
    }

    if (amps > peakActuatorCurrent) {
      peakActuatorCurrent = amps;
      peakCurrentSensor->publish_state(amps);
    }

    if (msActuatorStartTime != 0) {
      MilliSec elapsed = millis() - msActuatorStartTime;
      actuationTimeSensor->publish_state(elapsed / 1000.0);

      if (amps == 0) {
        msActuatorStartTime = 0;	// done, reset start clock
      }
    }  
  } 

  // 
  // isPumpPollingTime -- determines if we should send pump messages now
  //
  const bool isPumpPollingTime() {
    const unsigned long msElapsed = millis() - msLastPumpPoll;

    return msElapsed >= PumpPollIntervalMS;
  }

  //
  // sendMessage -- send message via RS-485 serial to the pump
  //
  const void sendMessage(const uint8_t* message, const size_t length) {
    write_array(msgHeader, sizeof(msgHeader));
    write_array(msgDestination, sizeof(msgDestination));
    write_array(message, length);
  }

  //
  // gotMessageByte -- updates received message data according to state
  //
  const MsgStates gotMessageByte(const uint8_t byte, const MsgStates msgState, Message* msg) {
    switch (msgState) {
      case expectStart:
        return (byte == 0xff) ? expectA5 : expectStart;

      case expectA5:
        return (byte == 0xa5) ? expect00 : expectStart;

      case expect00:
        return (byte == 0x00) ? expectDst : expectStart;

      case expectDst:
        if (byte == PumpId || byte == CtlrId) {
          msg->dest = byte;
          return expectSrc;
        }
        break;
 
      case expectSrc:
        if (byte == PumpId || byte == CtlrId) {
          msg->source = byte;
          return expectCmd;
        }
        break;

      case expectCmd:
        msg->action = byte;
        return expectLen;

      case expectLen:
        msg->length = byte;		// #bytes expected in message
        msg->actualLen = 0;		// #bytes actually recieved
        // if no data then checksum is next
        return (msg->length == 0) ? expectChkH : expectData;
   
      case expectData:
        if (msg->actualLen > msg->length || msg->actualLen >= MaxMsgData) {
          return expectStart;		// invalid length, start over
        }

        msg->data[msg->actualLen++] = byte;

        // if got all the data bytes, checksum next
	return (msg->actualLen == msg->length) ? expectChkH : expectData;

      case expectChkH:
        msg->checksum = byte << 8;
        return expectChkL;

      case expectChkL:
        msg->checksum |= byte;
        msg->actualChecksum = computeChecksum(msg);
        return msgComplete;
    }

    return expectStart;		// invalid state, start over
  }

  //
  // computeChecksum for message
  // 
  const uint16_t computeChecksum(const Message* msg) {
    uint16_t result = 0xA5;
    result += uint16_t(msg->dest);
    result += uint16_t(msg->source);
    result += uint16_t(msg->action);
    result += uint16_t(msg->length);

    for (int i=0; i<msg->actualLen; i++) {
      result += uint16_t(msg->data[i]);
    } 
    return result;
  }

  //
  // checkReceivedMessage -- checks (and debug prints) received RS-485 message 
  // 
  // Returns: true if message is valid and of interest
  //
  bool checkReceivedMessage(const Message* msg) {
    // ignore certain messages for now
    switch (msg->action) {
      case 2:
      case 4:
        return false;		// may be valid, but it's not of interest
    }

    const bool msgValid = (msg->actualChecksum == msg->checksum);

    char str[255];
    if (msg->source == 0x60 && msg->dest == 0x10) {
      strcpy(str, "----- RS-485: Pump->Ctlr");
    } else if (msg->source == 0x10 && msg->dest == 0x60) {
      strcpy(str, "----- RS-485: Ctlr->Pump");
    } else {
      sprintf(str, "********** RS-485: %02X->%02X", msg->source, msg->dest);
    }
   
    switch (msg->action) {
      case 1:  sprintf(str+strlen(str), " 1.Set Speed");        break;
      case 2:  sprintf(str+strlen(str), " 2.Equip Status");     break;
      case 4:  sprintf(str+strlen(str), " 4.Panel On/Off");     break;
      case 5:  sprintf(str+strlen(str), " 5.Time Bcst");        break;
      case 6:  sprintf(str+strlen(str), " 6.Pump On/Off");      break;
      case 7:  sprintf(str+strlen(str), " 7.Status");           break;
      case 9:  sprintf(str+strlen(str), " 9.Run @GPM");         break;
      default: sprintf(str+strlen(str), " %u. (x%02X)", msg->action, msg->action);
    }
    sprintf(str + strlen(str), " #%u: ", msg->length);

    switch (msg->action) {
      case requestStatus:	// pump status
      case requestSetSpeed:	// pump speed
        for (int i=0; i<msg->actualLen; i++) {
          sprintf(str+strlen(str), "%02X", msg->data[i]);
        }
        break;

      default:
        for (int i = 0; i < msg->actualLen; ++i) {
          sprintf(str+strlen(str), " %02X", msg->data[i]);
        }
    }

    if (msgValid) {
      sprintf(str+strlen(str), " (OK)");
    } else {
      sprintf(str+strlen(str), " (x%04X != %04X) <<<ERR", msg->checksum, msg->actualChecksum);
    }
    ESP_LOGD("custom", str);

    return msgValid;
  }

  void handlePumpStatus(const Message* msg) {
    //---------------------------
    // Save Pump RPM and Watts
    //---------------------------
    lastPumpWatts = (msg->data[3] << 8) | msg->data[4];
    lastPumpRPM   = (msg->data[5] << 8) | msg->data[6];

    //---------------------------
    // publish sensor data
    //---------------------------
    rpmSensor->publish_state(lastPumpRPM);
    wattsSensor->publish_state(lastPumpWatts);
  }

};

PoolPumpRS485* PoolPumpRS485::instance = 0;
