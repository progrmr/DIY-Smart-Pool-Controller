#include <sstream>
#include <iomanip>
#include "esphome.h"

#define MAXMSGDATA 255

class PoolPumpRS485 : public Component, public UARTDevice {
 public:
  PoolPumpRS485(UARTComponent *parent) : UARTDevice(parent) {}
 
  Sensor* rpmSensor = new Sensor();
  Sensor* wattsSensor = new Sensor();

  static PoolPumpRS485* instance;		// singleton instance

  const uint8_t pumpStatusRequest[10] = { 0x00, 0xFF, 0xA5, 0x00, 0x60, 0x10, 0x07, 0x00, 0x01, 0x1C };

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
    expectChkL  	// checksum lo byte
  };

  //
  // RS-485 Message Data Structure, not including header
  struct Message {
    uint8_t dest;
    uint8_t source;
    uint8_t action;
    uint8_t length;		// expected data length
    uint8_t actualLen;		// actual received length
    uint8_t data[MAXMSGDATA];	// received data
    uint16_t checksum;		// expected checksum
    uint16_t actualChecksum;	// computed checksum
  }; 

  bool shouldPollPumpStatus = true;
  unsigned long msLastStatusPoll = 0;

  unsigned long msLastPumpStatusRcvd = 0; 
  uint16_t lastPumpRPM = 9999;
  uint16_t lastPumpWatts = 9999;

  MsgStates msgState = expectStart;
  Message msg;

  // 
  // setup() -- one time setup
  //
  void setup() override {
    // not much to do here
  }

  //
  // loop() -- main loop, called about every 16 milliseconds
  //
  void loop() override {
    if (shouldPollPumpStatus && isPumpStatusTime(msgState)) {
      //
      // transmit a pump status request via RS-485 serial
      //
      write_array(pumpStatusRequest, sizeof(pumpStatusRequest));
    }

    //
    // recieve RS-485 serial data, handle incoming messages
    //
    while (available()) {
      uint8_t byte = read();
      gotMessageByte(byte);
    }
  }

  //
  // setPumpSpeed -- needs to check if the pump is running, turn it on
  //                 if needed, then set it to the desired RPM.
  //
  void setPumpSpeed(long speed) {
    // NOT IMPLEMENTED YET
    ESP_LOGD("custom","Set Pump Speed: %ld (Not Implemented Yet)", speed);
  }

  // 
  // isPumpStatusTime -- determines if we should request pump status now
  //
  bool isPumpStatusTime(const MsgStates msgState) {
    const unsigned long msNow = millis();

    if (msLastStatusPoll == 0) {
      msLastStatusPoll = msNow;
      return false;	// first time called
    }
 
    if (msNow < msLastStatusPoll) {
      // millis wraps around (every 50 days)
      msLastStatusPoll = msNow;
      return false;
    }

    const unsigned long msElapsed = msNow - msLastStatusPoll;
    if (msElapsed >= 15000 && msgState == expectStart) {
      // 15 seconds has elapsed, no message incoming now, request pump status 
      msLastStatusPoll = msNow;
      return true;
    }
    return false;
  }

  //
  // gotMessageByte -- updates received message data according to state
  //
  void gotMessageByte(uint8_t byte) {
    switch (msgState) {
      case expectStart:
        if (byte == 0xff) {
          msgState = expectA5;
        } // else no state change
        break;

      case expectA5:
        if (byte == 0xa5) {
          msgState = expect00;
        } else {
          msgState = expectStart;	// invalid, start over
        }
        break;

      case expect00:
        if (byte == 0x00) {
          msgState = expectDst;
        } else {
          msgState = expectStart;	// invalid, start over
        }
        break;

      case expectDst:
        if (byte == 0x60 || byte == 0x10) {
          msg.dest = byte;
          msgState = expectSrc;
        } else {
          msgState = expectStart;	// invalid, start over
        }
        break;
 
      case expectSrc:
        if (byte == 0x60 || byte == 0x10) {
          msg.source = byte;
          msgState = expectCmd;
        } else {
          msgState = expectStart;	// invalid, start over
        }
        break;

      case expectCmd:
        msg.action = byte;
        msgState = expectLen;
        break;

      case expectLen:
        msg.length = byte;		// #bytes expected in message
        msg.actualLen = 0;		// #bytes actually recieved
        if (msg.length == 0) {
          msgState = expectChkH;	// no data expected, checksum next
        } else {
          msgState = expectData;	// expect data
        }
        break;
   
      case expectData:
        if (msg.actualLen > msg.length || msg.actualLen >= MAXMSGDATA) {
          msgState = expectStart;	// invalid state, start over
          break;
        }
        msg.data[msg.actualLen++] = byte;
        if (msg.actualLen == msg.length) {
          msgState = expectChkH;	// got all the data bytes, checksum next
        } 
        break;

      case expectChkH:
        msg.checksum = byte << 8;
        msgState = expectChkL;
        break;

      case expectChkL:
        msg.checksum |= byte;
        msg.actualChecksum = computeChecksum(&msg);
        processMsg(&msg);
        msgState = expectStart;
        break;
    } // end switch
  }

  //
  // computeChecksum for message
  // 
  uint16_t computeChecksum(const Message* msg) {
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
  // processMsg -- handles and prints out received RS-485 message to debug output
  //
  void processMsg(const Message* msg) {
    // ignore certain messages for now
    switch (msg->action) {
      case 2:
      case 4:
        return;
    }

    const bool msgValid = (msg->actualChecksum == msg->checksum);

    char str[255];
    if (msg->source == 0x60 && msg->dest == 0x10) {
      strcpy(str, "RS-485: Pump->Ctlr");
    } else if (msg->source == 0x10 && msg->dest == 0x60) {
      strcpy(str, "RS-485: Ctlr->Pump");
    } else {
      sprintf(str, "RS-485: %02X->%02X", msg->source, msg->dest);
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
      case 7:	// pump status
        for (int i=0; i<msg->actualLen; i++) {
         sprintf(str+strlen(str), "%02X", msg->data[i]);
        }
        if (msgValid && msg->actualLen >= 7) {
          //---------------------------
          // Save Pump RPM and Watts
          //---------------------------
          msLastPumpStatusRcvd = millis();
	  lastPumpWatts = (msg->data[3] << 8) | msg->data[4];
	  lastPumpRPM   = (msg->data[5] << 8) | msg->data[6];
	  sprintf(str+strlen(str), " %uw %urpm", lastPumpWatts, lastPumpRPM);
	
          //---------------------------
          // publish sensor data
          //---------------------------
	  instance->rpmSensor->publish_state(lastPumpRPM);
          instance->wattsSensor->publish_state(lastPumpWatts);
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
  }

};

PoolPumpRS485* PoolPumpRS485::instance = 0;
