#pragma once

/**
 * @brief Decoding of the DCF77 signal, inspired by
 *  https://www.az-delivery.de/en/blogs/azdelivery-blog-fur-arduino-und-raspberry-pi/todo_florian_alles-hat-eine-genaue-zeit-dcf77-mit-dem-arduino
 * Completely rewritten, using Interrupts. No dependencies.
 * 
 * Copyleft Ivo Blöchliger
 * 
 */

struct DCF77Simple {
    int pin;
    void begin(int pin);
    void status2Serial();
    bool decode();
    void showData();

    struct {
       int cest;
       int mez;
       int change;
       int swtch; 
       int ms;
       int second;
       int minute;
       int hour;
       int day;
       int weekday;
       int month;
       int year;
    } timeInfo;

    char mem[8];
    char info[8];
    int getBit(int nr);
    
    volatile unsigned long lastLow = 0;  // Last change to Low
    volatile unsigned long lastHigh = 0; // Last change to High
    volatile unsigned long lastData = 0;
    volatile bool ignoreNext = false;
    volatile int nr = -1;
    
    int bcd(int start, int len, bool check=false);

};

extern DCF77Simple DCF77;
