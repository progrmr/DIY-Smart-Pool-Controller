#include "esphome.h"

#define MaxMsgData 255
#define PumpPollIntervalMS 30000
#define ReplyTimeOutMS 1000
#define MinPumpSpeed 0
#define MaxPumpSpeed 3000
#define NDaysPumpHistory 3

// seconds, time it takes for thermister in pipe to get to correct water 
// temp after water flow starts (pump running)
#define PIPE_TEMP_VALID_INTERVAL (4*60) 

class PoolPumpRS485 : public Component, public UARTDevice {
public:
    PoolPumpRS485(UARTComponent *parent) : UARTDevice(parent) {}
    
    typedef unsigned long MilliSec;
    
    Sensor* rpmSensor = new Sensor();		// pool pump RPM
    Sensor* wattsSensor = new Sensor();	    // pool pump power consumption
    Sensor* flowSensor = new Sensor();      // pool pump flow rate gal/min
    Sensor* powerSensor = new Sensor();     // pool pump % of max power
    
    Sensor* runTimeSensor = new Sensor();   // today's total pump run time, in hours
    Sensor* targetRunHours = new Sensor();  // hours pump should run per day
    Sensor* runHoursDeficit = new Sensor(); // pump run hours deficit from past 2 days
    
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
    struct Message {
        MsgPrefixes prefix;         // marks message start (always 0xA5)
        MsgPrefixes protocolRev;    // protocol version (always 0 for my pump)
        MsgDeviceIds dest;          // message destination
        MsgDeviceIds source;        // message source
        MsgActions action;
        uint8_t length;		        // expected data length
        uint8_t actualLen;		    // actual received length
        uint8_t data[MaxMsgData];	// received data
        uint16_t checksum;		    // expected checksum
        uint16_t actualChecksum;	// computed checksum
    }; 
    
    const uint8_t msgPreamble[3] = { 0xFF, 0x00, 0xFF };             // preamble before a message 
    const MsgPrefixes msgPrefix[2] = { mPrefixA5, mProtocolRev0 };   // starts a message transmittion
    const MsgDeviceIds msgDestSource[2] = { PumpId, CtlrId };		 // destination and source of message
    
    // These messages hold the data portion of the Message struct 
    const uint8_t pumpExtProg4On[4] = { 
        (programRegister >> 8),     // programRegister MSB
        (programRegister & 0xFF),   // programRegister LSB
        0x00, 
        extProg4 };                 // turns on external program 4
    
    const uint8_t pumpExtProg4Off[4] = { 
        (programRegister >> 8),     // programRegister MSB 0x03
        (programRegister & 0xFF),   // programRegister LSB 0x21
        0x00, 
        noExtProg };                // turns off any external program running
    
    bool shouldRequestExtPgmOn = false;
    uint8_t shouldRequestExtPgmOff = 0;

    long pumpSpeedRequested = 0;
    
    MilliSec msLastPumpPoll = 0;                // track time between polling for status
    MilliSec msLastPumpStatusReply = 0;         // tracks total pump run time for this polling cycle
    MilliSec msPumpStartTime = 0;               // millis() time of when pump started

    float pumpRunHours[NDaysPumpHistory];       // past history of run time, hrs per day, [0]==today
    float pumpTargetRunHours = NAN;             // target hours per day to run the pump
    float pumpRunHoursDeficit = NAN;            // past 2 days hours deficit
    
    uint16_t lastPumpRPM   = 9999;
    uint16_t lastPumpWatts = 9999;
    uint8_t  lastPumpFlow  = 255;
    uint8_t  lastPumpPower = 255;
    
    // 
    // setup() -- one time setup
    //
    void setup() override {
        // init a few things
        msLastPumpPoll = millis();		// init time of "previous" polling interval
        
        // init states
        runTimeSensor->publish_state( pumpRunHours[0] );
        
        // set history to unknown (skip [0], leave today at 0.0)
        for (int i=1; i<NDaysPumpHistory; i++) {
            pumpRunHours[i] = NAN;      // history is unknown on reboot
        }
        pumpRunHours[0] = 0;            // today starts with 0 hours
    }
    
    //
    // loop() -- main loop, called about every 16 milliseconds
    //
    void loop() override {
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
                    const auto msg = makeMessage(rqstSetSpeed, 
                                                 sizeof(pumpExtProg4On), 
                                                 pumpExtProg4On);
                    printMessage(msg);
                    sendMessage(msg);

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
                    const auto msg = makeMessage(rqstSetSpeed, 
                                                 sizeof(pumpExtProg4On), 
                                                 pumpExtProg4Off);
                    printMessage(msg);
                    sendMessage(msg);

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
                    if (msReplyWait >= ReplyTimeOutMS) {
                        // if we timed out on the pump status message, then report RPM as unknown
                        if (msgSequenceState == waitStatusReply) {
                            rpmSensor->publish_state(NAN);
                            wattsSensor->publish_state(NAN);
                            flowSensor->publish_state(NAN);
                            powerSensor->publish_state(NAN);
                            
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
    
    const char* const seqImage(MsgSequencingStates state) {
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
    void requestPumpSpeed(long speed) {
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
    long pumpSpeedForSolar(bool solarHeatOn) {
        if (id(spa_mode).state) {
            return solarHeatOn ? pumpSpaSolarOn : pumpSpaSolarOff;
        } else {
            return solarHeatOn ? pumpPoolSolarOn : pumpPoolSolarOff;
        }
    }

    //
    // setNextPollToHappenIn -- fix polling to happen next in durationMS
    // if time to the next poll less than duration, no change made
    //
    void setNextPollToHappenIn(MilliSec duration) {
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
    const MsgStates gotMessageByte(const uint8_t byte, const MsgStates msgState, Message* msg) {
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
                msg->actualChecksum = checksumForMessage(*msg);
                return msgComplete;
        }
        
        return expectStart;		// invalid state, start over
    }
    
    Message makeMessage(MsgActions action,
                        uint8_t length,
                        const uint8_t* data) {
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
    const void sendMessage(const Message msg) {
        // send data to the UART for RS-485 transmission to pump
        write_array(msgPreamble, sizeof(msgPreamble));
        
        write(msg.prefix);
        write(msg.protocolRev);
        write(msg.dest);
        write(msg.source);
        write(msg.action);
        write(msg.length);
        
        if (msg.length > 0) {
            write_array(msg.data, msg.length);
        }
        
        write(msg.checksum >> 8);       // MSB
        write(msg.checksum & 0xFF);     // LSB
    }

    const void printMessage(const Message msg) {
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
    const uint16_t checksumForMessage(const Message msg) {
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
    
    bool isValidMessage(const Message msg) {
        if (msg.source == CtlrId && msg.dest == CtlrId && msg.action == invalidAction) {
            return false;           // this message has been zeroed out due to timeout error
        }
        
        // verify checksum is correct
        return msg.checksum == msg.actualChecksum;
    }
    
    //-----------------------------------------------------
    // handlePumpStatusReply - got status from pump
    //-----------------------------------------------------
    void handlePumpStatusReply(const Message msg) {
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
            rpmSensor->publish_state(curPumpRPM);
            lastPumpRPM = curPumpRPM;
        }
        if (abs(curPumpWatts-lastPumpWatts) >= 5) {
            // Watts changed, publish an update
            wattsSensor->publish_state(curPumpWatts);
            lastPumpWatts = curPumpWatts;
        }
        if (curPumpFlow != lastPumpFlow) {
            // Flow changed, publish an update
            flowSensor->publish_state(curPumpFlow);
            lastPumpFlow = curPumpFlow;
        }
        if (curPumpPower != lastPumpPower) {
            // Power changed, publish an update
            powerSensor->publish_state(curPumpPower);
            lastPumpPower = curPumpPower;
        }
        
        //---------------------------
        // update pump start time tracker -- needs to know when pumping
        updatePumpStartTime(curPumpRPM);
        
        //---------------------------
        // update pump run time target hours (if temp valid)
        updatePumpTargetHours();

        //---------------------------
        // update pump run hours deficit
        updatePumpHoursDeficit();
        
        //---------------------------
        // compute how long the pump has been running for this polling loop
        //---------------------------
        MilliSec msNow = millis();
        
        // if we have the time of the previous pump status message and
        // the pump is running, then count the elapsed run time
        if (msLastPumpStatusReply > 0) {
            if (curPumpRPM > 1000) {
                MilliSec msElapsed = msNow - msLastPumpStatusReply;
                
                // get 100% run time credit at 2400, more when faster, less when slower
                float timeCreditFactor = curPumpRPM / 2400.0;     
                
                // adjust msElapsed by the timeCreditFactor, convert to seconds
                float secCredit = (msElapsed / 1000.0) * timeCreditFactor;
                
                if (secCredit >= 0.5) {
                    // update total run time for today (in hours)
                    float hrsCredit = secCredit / 3600.0;
                    pumpRunHours[0] += hrsCredit;
                    runTimeSensor->publish_state( pumpRunHours[0] );
                }
            }
        }
        msLastPumpStatusReply = msNow;  // start next period
    }
    
    //-----------------------------------------------------
    // midnight reset of pump run time history
    // save run time totals for past few days, 
    // zero out today's run time and start counting again.
    //
    // NOTE: this is called at midnight from yaml in the sntp lambda
    //-----------------------------------------------------
    void resetTotalPumpRunTime() {
        // shift the past days history over by 1 into higher array slots,
        // [0]==today, [1]==1 day ago, [2]==2 days ago, ... [6]==6 days ago
        //
        int day = NDaysPumpHistory-1;       // start with last index
        while (day >= 1) {
            pumpRunHours[day] = pumpRunHours[day-1];
            day--;
        }
        // zero out run time, a new day is starting
        pumpRunHours[0] = 0.0; 
    }

    void updatePumpStartTime(float pumpRPM) {
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
    
    void updatePumpHoursDeficit() {
        if (std::isnan(pumpTargetRunHours)) {
            return;     // target run hours is unknown
        }
        
        int nDaysHistory = 0;
        float pastDaysHours = 0;
        float desiredPastDaysHours = 0;
        
        for (int i=1; i<NDaysPumpHistory; i++) {
            if (!std::isnan(pumpRunHours[i])) {
                nDaysHistory++;
                pastDaysHours += pumpRunHours[i];
                desiredPastDaysHours += pumpTargetRunHours;
            }
        }
        
        if (nDaysHistory == 0) {
            return;     // no history hours
        }
        
        float newDeficitHours = desiredPastDaysHours - pastDaysHours;
        
        ESP_LOGD("custom","----- Run Hours Deficit: %0.1f (%0.1f actual, %0.1f desired)",
                 newDeficitHours, pastDaysHours, desiredPastDaysHours);

        // should we publish an update?
        float difference = fabs(newDeficitHours - pumpRunHoursDeficit);
        
        if (std::isnan(pumpRunHoursDeficit) || difference >= 0.1) {
            runHoursDeficit->publish_state(newDeficitHours);
            pumpRunHoursDeficit = newDeficitHours;
        }
    }
    
    float CtoF(float centigrade) {
        return (centigrade * 1.8) + 32.0;
    }

    void updatePumpTargetHours() {        
        const bool spaMode = id(spa_mode).state;
        if (spaMode || !isPipeTempValid()) {
            return;     // this only applies to pool temperature
        }
        
        auto waterTempC = id(water_temperature);
        float waterTempF = waterTempC.has_state() ? CtoF(waterTempC.state) : NAN;
        
        if (std::isnan(waterTempF)) {
            return;     // can't do this without water temp
        }

        float newTargetHrs = pumpHoursForWaterTempF(waterTempF);
        
        if (newTargetHrs != pumpTargetRunHours) {
            targetRunHours->publish_state(newTargetHrs);
            pumpTargetRunHours = newTargetHrs;
        }

        // for days with no pump run history (value == nan),
        // set the run history to match the current target run hours.
        // Simplifies deficit calc.
        for (int i=1; i<NDaysPumpHistory; i++) {
            if (std::isnan(pumpRunHours[i])) {
                pumpRunHours[i] = newTargetHrs;
            }
        }
    }
    
    float pumpHoursForWaterTempF(float waterTempF) {
        if (waterTempF >= 95) {
            return 12;      // 95+
        } else if (waterTempF >= 90) {
            return 11;      // 90-95
        } else if (waterTempF >= 85) {
            return 10;      // 85-90
        } else if (waterTempF >= 80) {
            return 9;       // 80-85
        } else if (waterTempF >= 75) {
            return 8;       // 75-80
        } else if (waterTempF >= 70) {
            return 7;       // 70-75
        } else if (waterTempF >= 60) { 
            return 6;       // 60-70
        } else {
            return 5;
        }
    }
    
    const bool isPipeTempValid() {
        if (msPumpStartTime == 0) {
            return false;       // pump is not running
        }
        const MilliSec msElapsed = millis() - msPumpStartTime;
        const float secsPumpOn = msElapsed / 1000.0;
        
        return secsPumpOn >= PIPE_TEMP_VALID_INTERVAL;
    }
    
    // called from yaml in a lambda, as desired
    void printDebugInfo() {
        char str[256] = {0};
        for (int i=0; i<NDaysPumpHistory; i++) {
            sprintf(str+strlen(str), "%0.3f ", pumpRunHours[i]);
        }
        ESP_LOGD("custom","----- Pump hours/day: %s", str);

        void updatePumpHoursDeficit();
    }
    

};

PoolPumpRS485* PoolPumpRS485::instance = 0;
