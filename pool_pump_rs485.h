#include "esphome.h"

#define MaxMsgData 255
#define PumpPollIntervalMS 15000
#define PumpId 0x60
#define CtlrId 0x10
#define InvalidAction 0xFF
#define ReplyTimeOutMS 1000
#define MinPumpSpeed 0
#define MaxPumpSpeed 3000
#define NDaysPumpHistory 7

// seconds, time it takes for thermister in pipe to get to correct water 
// temp after water flow starts (pump running)
#define PIPE_TEMP_VALID_INTERVAL (4*60) 

class PoolPumpRS485 : public Component, public UARTDevice {
public:
    PoolPumpRS485(UARTComponent *parent) : UARTDevice(parent) {}
    
    typedef unsigned long MilliSec;
    
    Sensor* rpmSensor = new Sensor();		// pool pump RPM
    Sensor* wattsSensor = new Sensor();	    // pool pump power consumption
    Sensor* runTimeSensor = new Sensor();   // today's total pump run time, in hours
    Sensor* targetRunHours = new Sensor();  // hours pump should run per day
    
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
    // RS-485 Message Actions (codes used by Intelliflo pump)
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
    long pumpSpeedRequested = 0;
    
    MilliSec msLastPumpPoll = 0;                // track time between polling for status
    
    MilliSec msLastPumpStatusReply = 0;         // tracks total pump run time for this polling cycle
    float hrsTotalRunTimeToday = 0;             // hours of total run time today
    float pumpRunHours[NDaysPumpHistory] = {0.0};  // 7 day history of run time, hrs per day, [0]==today
    
    uint16_t lastPumpRPM = 9999;
    uint16_t lastPumpWatts = 9999;
    
    // track pump startup time
    MilliSec msPumpStartTime = 0;        // millis() time of when pump started up
    
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
                if (shouldRequestSolarSpeedOn) {
                    // transmit a request for pump to switch on to solar speed
                    sendMessage(pumpSolarSpeedOn, sizeof(pumpSolarSpeedOn));
                    msgState = expectStart;
                    msgSequenceState = waitSolarSpeedOn;
                    msReplyWaitStart = msNow;
                } else {
                    msgSequenceState = sendSolarSpeedOff;
                }
                break;
                
            case sendSolarSpeedOff:
                if (shouldRequestSolarSpeedOff) {
                    // transmit a request for pump to switch off solar speed
                    sendMessage(pumpSolarSpeedOff, sizeof(pumpSolarSpeedOff));
                    msgState = expectStart;
                    msgSequenceState = waitSolarSpeedOff;
                    msReplyWaitStart = msNow;
                } else {
                    msgSequenceState = sendStatusRequest;
                }
                break;
                
            case sendStatusRequest:
                // transmit a pump status request via RS-485 serial
                sendMessage(pumpStatusRequest, sizeof(pumpStatusRequest));
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
                            // reset the total run time counter for this polling cycle
                            msLastPumpStatusReply = 0;
                        }
                        msgState = msgComplete;        // timed out, give up, call it done and move on
                        ESP_LOGD("custom","***** WARNING: timed out waiting for RS-485 message (seq:%d)", msgSequenceState);
                        msg.source = CtlrId;
                        msg.dest   = CtlrId;
                        msg.action = InvalidAction;     // force it to fail the check later
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
                            ESP_LOGD("custom","***** WARNING: RS-485 read() + gotMB() took %lu ms", msElapsed);
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
                    // we received a complete reply message (or timed out)
                    switch (msg.action) {
                        case requestSetSpeed:
                            // NOT IMPLEMENTED YET -- verify the pump did what we requested, 
                            // if it didn't there's not much we can do just report errors
                            break;
                            
                        case requestStatus:
                            // message won't be valid if we timed out earlier while waiting for it
                            if (checkReceivedMessage(&msg)) {
                                handlePumpStatusReply(&msg);
                            }
//                        {
//                            // ********************  DEBUG CODE -- REMOVE AFTER TESTING
//                            // ********************  DEBUG CODE -- REMOVE AFTER TESTING
//                            // ********************  DEBUG CODE -- REMOVE AFTER TESTING
//                            // ********************  DEBUG CODE -- REMOVE AFTER TESTING
//                            auto sntp = id(local_sntp_time);
//                            ESPTime time = sntp.now(); 
//                            // https://github.com/esphome/esphome/blob/c77a9ad3630802376ab65d79d73d5663c79bf6a4/esphome/core/time.h
//                            auto utc = sntp.utcnow();
//                            std::string tz = sntp.get_timezone();
//                            
//                            int utcHour = utc.hour;
//                            int utcMin = utc.minute;
//                            
//                            int hour = time.hour;
//                            int minute = time.minute;
//                            int second = time.second;
//                            ESP_LOGD("custom","---------- TIME: %d-%d-%d %02d:%02d:%02d %02d:%02d (TZ: %s)", 
//                                     time.year, time.month, time.day_of_month, 
//                                     time.hour, time.minute, time.second, 
//                                     utc.hour, utc.minute, 
//                                     tz.c_str());
//                        }
                            break;
                            
                        default:
                            // invalid or not of interest message
                            ESP_LOGD("custom","***** WARNING: unexpected RS-485 message received");
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
        if (speed > MaxPumpSpeed) { speed = MaxPumpSpeed; }
        if (speed < MinPumpSpeed) { speed = MinPumpSpeed; }
        
        ESP_LOGD("custom","----- requestPumpSpeed: %ld", speed);
        
        shouldRequestSolarSpeedOn  = (speed == 2701);
        shouldRequestSolarSpeedOff = (speed == 2401);
        
        if (shouldRequestSolarSpeedOn || shouldRequestSolarSpeedOff) {
            // adjust last pump poll time so that next poll occurs in 1 second
            setNextPollToHappenIn(1000);        // 1 sec in future
        }
        
        pumpSpeedRequested = speed;
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
        if (msg->source == CtlrId && msg->dest == CtlrId && msg->action == InvalidAction) {
            return false;           // this message has been zeroed out due to timeout error
        }
        
        // ignore certain messages for now
        switch (msg->action) {
            case 2:
            case 4:
                return false;		// may be valid, but it's not of interest
        }
        
        const bool msgValid = (msg->actualChecksum == msg->checksum);
        
        char str[255];
        if (msg->source == PumpId && msg->dest == CtlrId) {
            strcpy(str, "----- RS-485: Pump->Ctlr");
        } else if (msg->source == CtlrId && msg->dest == PumpId) {
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
    
    //-----------------------------------------------------
    // handlePumpStatusReply - got status from pump
    //-----------------------------------------------------
    void handlePumpStatusReply(const Message* msg) {
        //---------------------------
        // extract Pump RPM and Watts
        //---------------------------
        float curPumpWatts = uint16_t((msg->data[3] << 8) | msg->data[4]);
        float curPumpRPM   = uint16_t((msg->data[5] << 8) | msg->data[6]);
        
        //---------------------------
        // publish RPM and Watts sensor data
        //---------------------------
        if (fabs(curPumpRPM-lastPumpRPM) > 0.4) {
            // RPM changed, publish an update
            rpmSensor->publish_state(curPumpRPM);
            lastPumpRPM = curPumpRPM;
        }
        if (fabs(curPumpWatts-lastPumpWatts) >= 5) {
            // Watts changed, publish an update
            wattsSensor->publish_state(curPumpWatts);
            lastPumpWatts = curPumpWatts;
        }
        
        //---------------------------
        // update pump start time tracker -- needs to know when pumping
        //---------------------------
        updatePumpStartTime(curPumpRPM);
        
        //---------------------------
        // update pump run time target hours (if temp valid)
        //---------------------------
        updatePumpTargetHours();
        
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

        static int prevTargetHours = 0;
        int targetHours = 0;
        
        // we have a valid water temp, choose the run time
        if (waterTempF >= 90) {
            targetHours = 11;
        } else if (waterTempF >= 85) {
            targetHours = 10;
        } else if (waterTempF >= 80) {
            targetHours = 9;
        } else if (waterTempF >= 75) {
            targetHours = 8;
        } else if (waterTempF >= 70) {
            targetHours = 7;
        } else if (waterTempF >= 60) {
            targetHours = 6;
        } else {
            targetHours = 5;
        }
        
        if (targetHours != prevTargetHours) {
            id(pump_target_hours).publish_state(float(targetHours));
            prevTargetHours = targetHours;
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
            sprintf(str+strlen(str), "%0.1f ", pumpRunHours[i]);
        }
        ESP_LOGD("custom","----- Pump hours/day: %s", str);
    }
};

PoolPumpRS485* PoolPumpRS485::instance = 0;
