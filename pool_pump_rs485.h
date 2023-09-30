#include <sstream>
#include <iomanip>
#include "esphome.h"

#define MAXMSGDATA 255

class PoolPumpRS485 : public Component, public UARTDevice {
 public:
  PoolPumpRS485(UARTComponent *parent) : UARTDevice(parent) {}
 
  Sensor* rpmSensor = new Sensor();
  Sensor* wattsSensor = new Sensor();

  const uint8_t pumpStatusRequest[10] = { 0x00, 0xFF, 0xA5, 0x00, 0x60, 0x10, 0x07, 0x00, 0x01, 0x1C };

  // 
  // setup() -- one time setup
  //
  void setup() override {
    // nothing to do here
  }

  //
  // loop() -- main loop, called about every 16 milliseconds
  //
  void loop() override {
    if (isPumpStatusTime()) {
      write_array(pumpStatusRequest, sizeof(pumpStatusRequest));
    }

    while (available()) {
      const uint8_t byte = read();
      handleReceivedByte(byte);
    }
  }

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
    expectChkH,		// Checksum hi byte
    expectChkL  	// Checksum lo byte
  };

  // 
  // RS-485 Received Message Data
  //
  MsgStates msgState = expectStart;
  uint8_t msgSrc = 0;
  uint8_t msgDst = 0;
  uint8_t msgCmd = 0;
  uint8_t msgLen = 0;
  uint8_t msgDataRcvd = 0;
  uint8_t msgData[MAXMSGDATA];
  uint16_t msgChecksum = 0;

  void setPumpSpeed(int speed) {
    ESP_LOGD("custom","Set Pump Speed: %d", speed);
  }

  // 
  // isPumpStatusTime -- determines if we should request pump status now
  //
  bool isPumpStatusTime() {
    static unsigned long msLastPumpStatus = 0;
    const unsigned long msNow = millis();

    if (msLastPumpStatus == 0) {
      msLastPumpStatus = msNow;
      return false;	// first time called
    }
 
    if (msNow < msLastPumpStatus) {
      // millis wraps around (every 50 days)
      msLastPumpStatus = msNow;
      return false;
    }

    const unsigned long msElapsed = msNow - msLastPumpStatus;
    if (msElapsed >= 15000 && msgState == expectStart) {
      // 15 seconds has elapsed, no message incoming now, request pump status 
      msLastPumpStatus = msNow;
      return true;
    }
    return false;
  }

  //
  // printMsg -- prints out received RS-485 message to debug output
  //
  void printMsg() {
    static uint16_t pumpWatts = 9999;
    static uint16_t pumpRPM   = 9999;

    // ignore certain messages for now
    switch (msgCmd) {
      case 2:
      case 4:
        return;
    }

    const uint16_t expChecksum = expectedChecksum();
    const bool checksumGood = (expChecksum == msgChecksum);

    char str[255];
    if (msgSrc == 0x60 && msgDst == 0x10) {
      strcpy(str, "RS-485: Pump->Ctlr");
    } else if (msgSrc == 0x10 && msgDst == 0x60) {
      strcpy(str, "RS-485: Ctlr->Pump");
    } else {
      sprintf(str, "RS-485: %02X->%02X", msgSrc, msgDst);
    }
   
    switch (msgCmd) {
      case 1:  sprintf(str+strlen(str), " 1.Set Speed");        break;
      case 2:  sprintf(str+strlen(str), " 2.Equip Status");     break;
      case 4:  sprintf(str+strlen(str), " 4.Panel On/Off");     break;
      case 5:  sprintf(str+strlen(str), " 5.Time Bcst");        break;
      case 6:  sprintf(str+strlen(str), " 6.Pump On/Off");      break;
      case 7:  sprintf(str+strlen(str), " 7.Status");           break;
      case 9:  sprintf(str+strlen(str), " 9.Run @GPM");         break;
      default: sprintf(str+strlen(str), " %u. (x%02X)", msgCmd, msgCmd);
    }
    sprintf(str + strlen(str), " %2ub", msgLen);

    switch (msgCmd) {
      case 7:	// pump status
        for (int i=0; i<msgLen; i++) {
          sprintf(str+strlen(str), "%02X", msgData[i]);
        }
        if (checksumGood && msgLen >= 7) {
	  uint16_t watts = (msgData[3] << 8) | msgData[4];
	  uint16_t rpm   = (msgData[5] << 8) | msgData[6];
	  sprintf(str+strlen(str), " %uw %urpm", watts, rpm);
	
          if (rpm != pumpRPM) {
            // RPM changed, publish new value
            pumpRPM = rpm;
	    rpmSensor->publish_state(rpm);
          }
          if (watts != pumpWatts) {
            // Watts changed, publish new value
            pumpWatts = watts;
            wattsSensor->publish_state(watts);
          }
        }
        break;

      default:
        for (int i = 0; i < msgLen; ++i) {
          sprintf(str+strlen(str), " %02X", msgData[i]);
        }
    }

    if (checksumGood) {
      sprintf(str+strlen(str), " (OK)");
    } else {
      sprintf(str+strlen(str), " (x%04X != %04X) <<<ERR", msgChecksum, expChecksum);
    }

    ESP_LOGD("custom", str);
  }

  uint16_t expectedChecksum() {
    uint16_t result = 0xA5;
    result += uint16_t(msgDst);
    result += uint16_t(msgSrc);
    result += uint16_t(msgCmd);
    result += uint16_t(msgLen);

    for (int i=0; i<msgLen; i++) {
      result += uint16_t(msgData[i]);
    } 
    return result;
  }

  //
  // handleReceivedByte -- updates received message data according to state
  //
  void handleReceivedByte(uint8_t byte) {
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
          msgDst = byte;
          msgState = expectSrc;
        } else {
          msgState = expectStart;	// invalid, start over
        }
        break;
 
      case expectSrc:
        if (byte == 0x60 || byte == 0x10) {
          msgSrc = byte;
          msgState = expectCmd;
        } else {
          msgState = expectStart;	// invalid, start over
        }
        break;

      case expectCmd:
        msgCmd = byte;
        msgState = expectLen;
        break;

      case expectLen:
        msgLen = byte;
        msgDataRcvd = 0;
        if (msgLen == 0) {
          msgState = expectChkH;
        } else {
          msgState = expectData;
        }
        break;
   
      case expectData:
        if (msgDataRcvd > msgLen || msgDataRcvd == MAXMSGDATA) {
          msgState = expectStart;	// invalid state, start over
          break;
        }
        msgData[msgDataRcvd++] = byte;
        if (msgDataRcvd == msgLen) {
          msgState = expectChkH;
        } 
        break;

      case expectChkH:
        msgChecksum = byte << 8;
        msgState = expectChkL;
        break;

      case expectChkL:
        msgChecksum |= byte;
        msgState = expectStart;
        printMsg();
        break;
    } // end switch
  }

};
