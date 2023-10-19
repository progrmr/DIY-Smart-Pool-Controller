//
//  time_tester.h
//  
//
//  Created by Gary Morris on 2023-10-15.
//
typedef unsigned long ULong;

// MS = milliseconds
#define MS_DELAY 5

#define MS_REPORT_INTERVAL 120000        // 120 seconds

// US = microseconds = 𝜇Sec
#define US_PER_BIN 2000
#define N_BINS 50
#define US_MAX_BINNED (US_PER_BIN * N_BINS)

class TimeTester : public PollingComponent, public TextSensor {
  public:
    static TimeTester* instance;     // singleton instance
    
    TimeTester() : PollingComponent(60000) {}        // call update() every 60s
    
    ULong elapsedBins[N_BINS] = {0};    // counters
    ULong msBinsPublishedState = 0;     // millis time of last publish_state
    
    void setup() override {
        msBinsPublishedState = millis();
    }
    
    void update() override {
        const ULong msStart = millis();
        
        // artificially delay the update() method
        for (int i=0; i<MS_DELAY; i++) {
            delay(1);
        }
        
        // how much time was spend in the update() method?
        const ULong msAfter = millis();
        const ULong msElapsed = msAfter - msStart;
        const ULong usElapsed = (msElapsed - MS_DELAY) * 1000;
        
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
            for (int i=0; i<N_BINS; i++) {
                //sprintf(str, )
            }
            
            msBinsPublishedState = msStart;
        }
    }
    
};

TimeTester* TimeTester::instance = 0;
