#include <Arduino.h>

#include <time.h> 
#include "WiFi.h"
#include <Adafruit_NeoPixel.h>
#include "esp_adc_cal.h"



int black = 0;
const uint16_t PixelCount = 150; 
float brightness = 0.3;
int state = -1;


unsigned int eventpos = 0;
int events[][3] = {
  {7,40,45},
  {8,25,9},
  {8,34,45},
  {9,19,9},
  {9,28,45},
  {10,13,17},
  {10,30,45},
  {11,15,9},
  {11,24,45},
  {12,9,5},
  {12,14,45},
  {12,59,5},
  {13,04,45},
  {13,49,6}, 
  {13,55,45},  
  {14,40,9},
  {14,49,45},  
  {15,34,9},
  {15,43,45},  
  {16,28,5},
  {16,33,45},  
  {17,18,5},   // hours, minutes, duration
  {17,23,45},
  /*
  {18,8,5},
  {18,13,45},
  {18,58,5},
  {19,03,45},
  {19,48,5},
  {20,53,45},
  {21,38,5} */
};
  


#define PIN 13
Adafruit_NeoPixel strip(PixelCount, PIN, NEO_RGB + NEO_KHZ800);

float hues[PixelCount];
float vh[PixelCount];

void printTime() {
    time_t now;
    time(&now);
    struct tm  info;
    localtime_r(&now, &info);
    Serial.println(String("Day=")+info.tm_mday+", "+info.tm_hour+":"+info.tm_min+":"+info.tm_sec);
}


void timeKeeping(int delayMS) {

  brightness = analogRead(34)/4096.0*0.9+0.1;
  Serial.println(brightness);
  strip.setBrightness(255*brightness);

  unsigned long start = millis()+delayMS;
  while (start>millis()) {    
    delay(1);
  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for serial attach


  Serial.println();
  Serial.println("Initializing LEDS");
  Serial.flush();

  // this resets all the neopixels to an off state
  strip.begin();
  strip.show();
  strip.setBrightness(255*brightness);
    
    //WiFi.begin("stopbuepf", "stopbuepf");
    WiFi.begin("St.Galler Wireless", "");

    Serial.println();
    Serial.println();
    Serial.print("Wait for WiFi... ");

    int i=0;
    while(WiFi.status()!= WL_CONNECTED) {
        Serial.print(WiFi.status());        
        strip.setPixelColor(i,strip.ColorHSV((i%10)*6553,255,255));
        i++;
        strip.show();
        timeKeeping(100);
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("SSID ");
    Serial.println(WiFi.SSID());
    printTime();
    Serial.println("NTP sync start.");
    // Daylightsaving time included
    configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org");

    delay(1000);
    printTime();
 
    strip.clear();

}


long startms;
long duration;

void checkEvent() {
    time_t now;
    time(&now);
    struct tm  info;
    localtime_r(&now, &info);

  int h = info.tm_hour;
  int m = info.tm_min;
  int s = info.tm_sec;
  if (state==-1) { // Init
    for(eventpos=0;eventpos<sizeof(events);eventpos++) {
      int hh = events[eventpos][0];
      int mm = events[eventpos][1];
      if (hh*60+mm>h*60+m) {
        Serial.println(String("Found time ")+hh+":"+mm+"  now "+h+":"+m);
        if (eventpos==0) {
          state=0;
        } else {
          hh = events[eventpos-1][0];
          mm = events[eventpos-1][1];
          Serial.println(String("Last event time ")+hh+":"+mm+"  now "+h+":"+m);
          startms = millis()-1000L*(3600L*h+m*60+s-(3600L*hh+60*mm));
          duration = events[eventpos-1][2]*60L*1000L;
          state = 2;
        }
        break;
      }
    }
  } else {
    if (events[eventpos][0]==h && events[eventpos][1]==m) {
      startms = millis();
      duration = 1000L*60L*events[eventpos][2];
      state = 2;
      eventpos++;
      if (eventpos==sizeof(events)) {
        eventpos = 0;
      }
    }
  }
}

void smoothout(int s) {
  for (int i=PixelCount-1; i>=s; i--) {
    float d = hues[i-1]-hues[i];
    if (d>0.5) d=-(1-d);
    if (d<-0.5) d=1+d;
    vh[i] = 0.95*vh[i]+d/1000.0;
    if (vh[i]>0.01) vh[i]=0.01;
    if (vh[i]<-0.01) vh[i]=-0.01;
    hues[i]+=vh[i];
    if (hues[i]>1.0) hues[i]-=1.0;
    if (hues[i]<-1.0) hues[i]+=1.0;
    strip.setPixelColor(i,strip.ColorHSV((int)(0xffff*hues[i])),255,255);
  }
  strip.show();
}





long waitForIt(long step, long steps) {
  long localms = duration*1.0*step/steps;
  long waituntil = startms+localms;
  waituntil -= millis();
  if (waituntil<0) waituntil=0;
  return waituntil;
}

float getBr(int j) {
  float r = brightness*(5-j)/5;
  return r*r;
}

void showMustGoOn() {
  Serial.println(String("showMustGoOn start=")+startms+" duration="+duration+"   millis()="+millis());
  long steps = 150*151/2;
  long step = 0;
  state = 1;
  for (int last=PixelCount; last>0; last--) {
    if (state!=1) break;
    hues[last-1] = random(10000)/10000.0;
    vh[last-1] = 0.0;
    int c = strip.ColorHSV(0xffff*hues[last-1], 255, 255);
    if (waitForIt(step+last, steps)==0) {
      strip.setPixelColor(last-1, c);
      step+=last;
      Serial.println(String("Fast forward to step=")+step+" at last="+last+"  of steps="+steps);
    } else {
      Serial.println(String("Firing at last=")+last+" at step="+step+"  of steps="+steps);
      for (int i=0; i<last; i++) {
        if (state!=1) break;
        step++;
        strip.setPixelColor(i,c);
        for (int j=1; j<=5; j++) {
          if (i-j>=0) {
            int c2 = strip.ColorHSV((int)(hues[last-1]*0xffff), 255,255);
            strip.setPixelColor(i-j,c2);
          } else if (last<PixelCount) {
            int c2 = strip.ColorHSV((int)(hues[last]*0xffff), 255,getBr(j)*255);
            strip.setPixelColor(last+i-j,c2);
          }
        }
        if (i>4) {
          strip.setPixelColor(i-5,black);
        }
        strip.show();
        for (int j=0; j<1; j++) {
          if (last<PixelCount-1) {
            smoothout(last+1);
          }
          timeKeeping(waitForIt(step, steps));
        }
      }
    }
  }
  for (int i=0; i<PixelCount; i++) {
    strip.setPixelColor(i,black);
  }
  strip.show();
  
}

void loop() {
  state=-1; // Force initialisation
  checkEvent();
  if (state == 0) strip.clear();
  if (state==2) { 
    showMustGoOn();
    Serial.println(String("Show finished with state=")+state);
  }  
  timeKeeping(200);
}
