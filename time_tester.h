//
//  time_tester.h
//  
//
//  Created by Gary Morris on 2023-10-15.
//
typedef unsigned long ULong;

// MS = milliseconds
#define MS_DELAY 5

#define MS_REPORT_INTERVAL 60000        // 60 seconds

// US = microseconds = 𝜇Sec
#define US_PER_BIN 2000
#define N_BINS 50
#define US_MAX_BINNED (US_PER_BIN * N_BINS)

class TimeTester : public PollingComponent, public TextSensor {
  public:
    static TimeTester* instance;     // singleton instance
    
    TimeTester() : PollingComponent(200) {}        // call update() periodically
    
    ULong elapsedBins[N_BINS] = {0};    // counters
    ULong msBinsPublishedState = 0;     // millis time of last publish_state
    
    void setup() override {
        msBinsPublishedState = millis();
    }
    
    void update() override {
//        const ULong msStart = millis();
//        
//        // artificially delay the update() method
//        for (int i=0; i<MS_DELAY; i++) {
//            delay(1);
//        }
//        
//        // how much time was spend in the update() method?
//        const ULong msAfter = millis();
//        const ULong msElapsed = msAfter - msStart;
//        // subtract expected MS_DELAY time, convert to uSec
//        const ULong usElapsed = (msElapsed - MS_DELAY) * 1000;
//        
//        int binIndex = usElapsed / ULong(US_PER_BIN);
//        if (binIndex >= N_BINS) {
//            binIndex = N_BINS-1;
//        }
//        
//        // increment the counter in the bin for this elapsed time
//        elapsedBins[binIndex]++;
//        
//        // track total elapsed time
//        const ULong msSincePublish = msStart - msBinsPublishedState;
//        if (msSincePublish >= MS_REPORT_INTERVAL) {
//            // it's time to publish state again
//            // NOT IMPLEMENTED YET
//            char str[256] = {0};
//            for (int i=0; i<N_BINS; i++) {
//                int len = strlen(str);
//                if (len > 0) {
//                    sprintf(str+len,",");
//                    len++;  // we added one char
//                }
//                if (elapsedBins[i] > 0) {
//                    sprintf(str+len, "%lu", elapsedBins[i]);
//                }
//            }
//            publish_state(str);            
//            msBinsPublishedState = msStart;
//        }
    }
    
};

class TimeTest2 : public Component, public Sensor {
public:
    static TimeTest2* instance;     // singleton instance

    ULong elapsedBins[N_BINS] = {0};    // counters
    ULong msBinsPublishedState = 0;     // millis time of last publish_state
    
    void setup() override {
        msBinsPublishedState = millis();
    }
    
    void loop() override {
        const ULong msBegin = millis();
        
        // artificially delay the update() method
        for (int i=0; i<MS_DELAY; i++) {
            delay(1);
        }
        
        // report how much time was spent between the millis() calls
        const ULong msEnd = millis();
        reportElapsed(msBegin, msEnd, MS_DELAY);
    }
    
    void reportElapsed(ULong msStart, ULong msFinish, ULong msExpected) {
        const ULong msElapsed = msFinish - msStart;
        const ULong usElapsed = (msElapsed - msExpected) * 1000;
        
        int binIndex = usElapsed / ULong(US_PER_BIN);
        if (binIndex >= N_BINS) {
            binIndex = N_BINS-1;
        }
        
        // increment the counter in the bin for this elapsed time
        elapsedBins[binIndex]++;
        
        // track total elapsed time
        const ULong msSincePublish = msStart - msBinsPublishedState;
        if (msSincePublish >= MS_REPORT_INTERVAL) {
            // it's time to publish state again
            // NOT IMPLEMENTED YET
            char str[256] = {0};
            for (int bin=0; bin<N_BINS; bin++) {
                int len = strlen(str);
                if (elapsedBins[bin] > 0) {
                    if (len > 0) {
                        sprintf(str+len,",");
                        len++;  // we added one char
                    }
                    sprintf(str+len, "%lu(%d)", elapsedBins[bin], bin);
                }
            }
            TimeTester::instance->publish_state(str);            
            msBinsPublishedState = msStart;
        }
    }
    
};

TimeTester* TimeTester::instance = 0;
TimeTest2* TimeTest2::instance = 0;
