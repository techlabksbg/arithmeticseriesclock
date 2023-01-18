#include <Arduino.h>
#include "dcf77simple.h"

/**
 * @brief Decoding of the DCF77 signal, inspired by
 *  https://www.az-delivery.de/en/blogs/azdelivery-blog-fur-arduino-und-raspberry-pi/todo_florian_alles-hat-eine-genaue-zeit-dcf77-mit-dem-arduino
 * Completely rewritten, using Interrupts. No dependencies.
 * 
 * Copyleft Ivo Blöchliger
 * 
 */

DCF77Simple DCF77;

void IRAM_ATTR pinchange() {
    unsigned long now = millis();
    if (digitalRead(DCF77.pin)) {// HIGH
        long lowtime = now-DCF77.lastLow;        
        //Serial.println(lowtime);

        if (lowtime<600) { // this corrects minor flukes
            DCF77.ignoreNext = true;
        } else {
            DCF77.ignoreNext = false;
                
            if (lowtime>1700) {
                if (DCF77.nr>=58) {
                    memcpy((void*)DCF77.info, (void*)DCF77.mem, 8);
                    DCF77.lastData = millis();
                }
                DCF77.nr = 0;
            } else if (lowtime>730 && lowtime<830 && DCF77.nr!=-1) { // Bit 1
                DCF77.mem[DCF77.nr/8] |= (1<<(DCF77.nr%8));
                DCF77.nr++;
            } else if (lowtime>=830 && lowtime<950 && DCF77.nr!=-1) { // Bit 0
                DCF77.mem[DCF77.nr/8] &= ~(1<<(DCF77.nr%8));
                DCF77.nr++;
            } else {
                DCF77.nr=-1;
            }
            DCF77.lastHigh = now;
        }
    } else if (!DCF77.ignoreNext) {
        DCF77.lastLow = now;
    }
}

void DCF77Simple::begin(int _pin) {
    pin = _pin;
    pinMode(pin, INPUT);
    lastHigh = millis();
    lastLow = millis();
    attachInterrupt(pin, pinchange, CHANGE);
}

void DCF77Simple::status2Serial() {
    Serial.printf("LastLow %lu, LastHigh %lu, LastData %lu, nr %d\n", lastLow, lastHigh, lastData, nr);
}

int DCF77Simple::bcd(int start, int len, bool check) {
    int coeff[]={1,2,4,8,10,20,40,80};
    int s = 0;
    int parity = 0;
    for (int i=0; i<len; i++) {
        int bit = getBit(start+i);
        s += coeff[i]*bit;
        parity ^= bit;
    }
    if (!check || getBit(start+len)==parity) {
        return s;
    }
    Serial.printf("parity check failed at start=%d, len=%d\n",start, len);
    return -1;
}

bool DCF77Simple::decode() {
    if (lastData==0) {
        Serial.println("No complete transmission yet");
        return false;
    }
    if (getBit(0)!=0) {
        Serial.println("Startbit is not zero");
        return false;
    }
    if (getBit(20)!=1) {
        Serial.println("Startbit of date info is not 1");
        return false;
    }
    timeInfo.daylightSavingChangeAtEndOfHour = getBit(16);
    // See https://de.wikipedia.org/wiki/DCF77#Signal
    timeInfo.cest = getBit(17);
    timeInfo.mez = getBit(18);
    timeInfo.daylightSaving = -1;
    if (timeInfo.cest==0 && timeInfo.mez==1) {
        timeInfo.daylightSaving = 0;
    } else if (timeInfo.cest==1 && timeInfo.mez==0) {
        timeInfo.daylightSaving = 1;
    }
    timeInfo.swtch = getBit(19);
    timeInfo.minute = bcd(21,7, true);
    if (timeInfo.minute<0) {
        return false;
    }
    timeInfo.hour = bcd(29,6, true);
    if (timeInfo.hour<0) {
        return false;
    }
    timeInfo.day = bcd(36,6);
    timeInfo.weekday = bcd(42,3);
    timeInfo.month = bcd(45,5);
    timeInfo.year = 2000+bcd(50,8);
    long unsigned ms = millis()-lastData;
    // Set ms and seconds, correct for overflow
    timeInfo.ms = ms%1000;
    ms/=1000;
    timeInfo.second = ms%60;  // Ha ha, wrong in case of leap second :-/
    ms = ms/60+timeInfo.minute;
    timeInfo.minute = ms%60;
    timeInfo.hour = ms/60+timeInfo.hour;
    // TODO: No overflow of days... expect spooky behaviour at midnight.
    // TODO check parity for date information
    return true;
}

void DCF77Simple::showData() {

    if (lastData==0) {
        Serial.println("No data yet, still waiting to synchronize...");
        return;
    }
    if (!decode()) {
        Serial.println("Current Data is not valid!");
        return;
    }
    printf("changeAtEndOfHour: %d, CEST: %d, MEZ: %d, daylightSaving: %d, leapSecond: %d\n %02d:%02d:%02d.%03d %04d-%02d-%02d wday=%d\n",
    timeInfo.daylightSavingChangeAtEndOfHour, timeInfo.cest, timeInfo.mez, timeInfo.daylightSaving, timeInfo.swtch,
    timeInfo.hour, timeInfo.minute, timeInfo.second, timeInfo.ms,
    timeInfo.year, timeInfo.month, timeInfo.day, timeInfo.weekday
    );
}

int DCF77Simple::getBit(int nr) {
    return (info[nr/8] >> (nr % 8)) & 1;
}
