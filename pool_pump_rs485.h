#include <sstream>
#include <iomanip>
#include "esphome.h"

#define MAXMSGDATA 255

class PoolPumpRS485 : public Component, public UARTDevice {
 public:
  PoolPumpRS485(UARTComponent *parent) : UARTDevice(parent) {}

  void setup() override {
    // nothing to do here
  }


  // typical message: 00 FF A5 00 60 10 02 02 03 96 01 B2
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

  MsgStates msgState = expectStart;
  uint8_t msgSrc = 0;
  uint8_t msgDst = 0;
  uint8_t msgCmd = 0;
  uint8_t msgLen = 0;
  uint8_t msgDataRcvd = 0;
  uint8_t msgData[MAXMSGDATA];
  uint16_t msgChecksum = 0;



  const size_t MaxBytes = 8;
  unsigned long totalRead = 0;

  void loop() override {
    while (available()) {
      uint8_t byte = read();
      parseMsgByte(byte);
    }

/*
    if (available()) {
      uint8_t bytes[MaxBytes];
      unsigned long nRead = 0;

      while (available()) {
        bytes[nRead++] = read();
	totalRead++;

	if (nRead >= MaxBytes)  {
          printBytes(bytes, nRead, totalRead);
          nRead = 0;
        }
      }

      if (nRead > 0) {
        printBytes(bytes, nRead, totalRead);
      }
    }
*/
  }

  void printMsg() {
    char str[255];
    uint16_t watts;
    uint16_t rpm;

    // ignore certain messages for now
    switch (msgCmd) {
      case 2:
      case 4:
        return;
    }

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

      default: 
	sprintf(str + strlen(str), " Cmd=%u (x%02X)", msgCmd, msgCmd);
    }
    sprintf(str + strlen(str), " Len=%2u(x%02X)", msgLen, msgLen);

    switch (msgCmd) {
      case 7:
        if (msgLen >= 7) {
          sprintf(str+strlen(str), msgData[0] == 0x0A ? " ON" : " OFF");
	  watts = (msgData[3] << 8) | msgData[4];
	  rpm = (msgData[5] << 8) | msgData[6];
	  sprintf(str+strlen(str), " watts: %u, rpm: %u", watts, rpm);
        }
        break;

      default:
        for (int i = 0; i < msgLen; ++i) {
          sprintf(str + strlen(str), " %02X", msgData[i]);
        }
    }

    sprintf(str + strlen(str), " (x%04X)", msgChecksum);

    ESP_LOGD("custom", str);
  }

  void printBytes(uint8_t* bytes, unsigned long count, unsigned long totalRead) {
      char str[120];
      str[0] = 0;

      for (size_t i = 0; i < count; ++i) {
          sprintf(str + strlen(str), " %02X", bytes[i]);
      }

      ESP_LOGD("custom", "RS-485: (%lu) %lu: %s", totalRead, count, str);
  }


  void parseMsgByte(uint8_t byte) {
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

