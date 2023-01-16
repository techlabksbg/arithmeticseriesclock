#include <Arduino.h>

#include <time.h> 
#include "dcf77simple.h"
#include "NeoPixelBus.h"
#include "esp_adc_cal.h"



RgbColor black(0,0,0);
const uint16_t PixelCount = 150; 
float brightness = 0.3;
int state = -1;


unsigned int eventpos = 0;
int events[][3] = {
    {7,31,9},
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
//Adafruit_NeoPixel strip(PixelCount, PIN, NEO_RGB + NEO_KHZ800);
// Uses GPIO2 alias D4
//NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, 42); // pin is ignored
NeoPixelBus<NeoRgbFeature, NeoEsp32I2s1800KbpsMethod> strip(PixelCount, PIN);


float hues[PixelCount];
float power[PixelCount];
float vh[PixelCount];

struct slider {
    int last = 0;  // Index of target LED + 1
    float hue = 0.0;
    bool active = false;
    void paint(float pos) {  // (pos from -1 to range+5)
        if (active) {
            if (pos>last+5) {
                active = false;
                return;
            }
            for (int x=int(pos)-6; x<pos+2; x++) {
                if (x>=0 && x<last) {
                    float l=0.0;
                    if (x>=pos-5 && x<pos) {
                        l = 0.5*(x-pos+5)/5.0;
                    } else if (x>=pos) {
                        l = 0.5*((pos+1)-x);
                        if (l<0.0) { l=0.0; }
                    }
                    l = 4*l*l*l;
                    strip.SetPixelColor(x, HslColor(hue, 1.0, l));
                }
            }
        }
    }
};



void printTime() {
    time_t now;
    time(&now);
    struct tm  info;
    localtime_r(&now, &info);
    Serial.println(String("Day=")+info.tm_mday+", "+info.tm_hour+":"+info.tm_min+":"+info.tm_sec);
}

void smoothout(int s);

void timeKeeping(int delayMS, int last) {

  brightness = analogRead(34)/4096.0*0.95+0.05;
  //Serial.println(brightness);
  //strip.setBrightness(255*brightness);

  unsigned long start = millis()+delayMS;
  while (start>millis()) {
    if (last<PixelCount) {
      smoothout(last+1);
    }
    delay(5);  // Time to update strip
  }
}

void dcf2esp() {
  if (DCF77.decode()) {
    tm local;
    local.tm_year = DCF77.timeInfo.year - 1900;
    local.tm_mon = DCF77.timeInfo.month-1;
    local.tm_hour = DCF77.timeInfo.hour;
    local.tm_min = DCF77.timeInfo.minute;
    local.tm_sec = DCF77.timeInfo.second;
    const time_t sec = mktime(&local);
    localtime(&sec); //set time
  }
}

void waitForTimeFix()
{
  int pos = 0;
  while (DCF77.lastData == 0 || !DCF77.decode()) {
    if (digitalRead(DCF77.pin)) { // HIGH
      if (pos > 50) {
        strip.ClearTo(black);
        pos = 0;
        DCF77.status2Serial();
      }
      if (DCF77.nr != -1) {
        strip.SetPixelColor(pos, RgbColor(128, 0, 0));
      } else {
        strip.SetPixelColor(pos, RgbColor(64, 64, 0));
      }
    } else { // LOW
      if (DCF77.nr != -1) {
        strip.SetPixelColor(pos, RgbColor(0, 128, 0));
      } else {
        strip.SetPixelColor(pos, RgbColor(0, 0, 64));
      }
    }
    pos++;
    strip.Show();
    delay(15);
  }
  dcf2esp();
  strip.ClearTo(black);
  strip.Show();
  delay(15);
}


struct iterPos {
  int iter;
  float pos;
};


// compute the fractional position number, given the fraction [0,1] of a full period
iterPos iterAndPos(float t) {
  float steps = (PixelCount+1)*(PixelCount)/2*t; // How many total steps to advance
  int s = int(steps);   // Floor of total steps
  int x = 2*PixelCount+1;    // helpful constant
  int n = (int)(0.5*(x)+sqrt(x*x/4-2*s));   // How many interations in [0..PixelCount-1])
  float pos = steps-(x-n)*n/2;       // How many steps after start (i.e. position of the dot)
  return {n,pos};
}

float hsvInterpolate(float h1, float h2, float t) {
  if (abs(h1-h2)<=0.5) {
    return (1-t)*h1+t*h2;
  }
  float h;
  if (h1<h2) {
    h = (1-t)*(h1+1) + t*h2;
  } else {
    h = (1-t)*h1 + t*(h2+1);
  }
  if (h<1.0) return h;
  return h-1.0;
}

// Diffusing colors 
void diffuse(int n) {
  for (int i=n+1; i<150; i++) {
    float t = 0.99;
    hues[i] = hsvInterpolate(hues[i], hues[i-1], t);
  }
}

void sliderTest() {
  unsigned long start = millis();
  slider s;
  s.last=150;
  s.hue = 0.4;
  s.active = true;
  while (s.active) {
    s.hue = (millis()-start)/100.0/150;
    s.paint((millis()-start)/100.0);
    strip.Show();
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for serial attach


  Serial.println();
  Serial.println("Initializing LEDS");
  Serial.flush();

  // this resets all the neopixels to an off state
  strip.Begin();
  strip.ClearTo(black);
  strip.SetPixelColor(0,RgbColor(255,0,0));
  strip.SetPixelColor(1,RgbColor(0,255,0));
  strip.SetPixelColor(2,RgbColor(0,0,255));
  strip.Show();

  DCF77.begin(5);
  sliderTest();
  // waitForTimeFix();
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
    if (d>0.5) d=-1.0+d;
    if (d<-0.5) d=1+d;
    vh[i] = 0.95*vh[i]+d/10000.0;
    if (vh[i]>0.01) vh[i]=0.01;
    if (vh[i]<-0.01) vh[i]=-0.01;
    hues[i]+=vh[i];
    if (hues[i]>1.0) hues[i]-=1.0;
    if (hues[i]<0.0) hues[i]+=1.0;
    strip.SetPixelColor(i,HslColor(hues[i], 1.0f,0.5f*brightness));
  }
  strip.Show();
}





long waitForIt(long step, long steps) {
  long localms = duration*1.0*step/steps;
  long waituntil = startms+localms;
  waituntil -= millis();
  if (waituntil<0) waituntil=0;
  return waituntil;
}

float getBr(int j) {
  float r = (5-j)/5.0f;
  return r*r;
}

void showMustGoOn() {
  Serial.println(String("showMustGoOn start=")+startms+" duration="+duration+"   millis()="+millis());
  long steps = 150*151/2;
  long step = 0;
  state = 1;
  for (int last=PixelCount; last>0; last--) {
    if (state!=1) break;
    hues[last-1] = random(10000)/10000.0f;
    vh[last-1] = 0.0;
    HslColor c(hues[last-1], 1.0f, 0.5f*brightness);
    if (waitForIt(step+last, steps)==0) {
      strip.SetPixelColor(last-1, c);
      step+=last;
      Serial.println(String("Fast forward to step=")+step+" at last="+last+"  of steps="+steps);
    } else {
      Serial.println(String("Firing at last=")+last+" at step="+step+"  of steps="+steps);
      for (int i=0; i<last; i++) {
        if (state!=1) break;
        step++;
        strip.SetPixelColor(i,c);
        for (int j=1; j<=5; j++) {
          if (i-j>=0) {
            HslColor c2 (hues[last-1], 1.0f, getBr(j)*0.5f*brightness);
            strip.SetPixelColor(i-j,c2);
          } else if (last<PixelCount) {
            HslColor c2(hues[last], 1.0f, getBr(j)*0.5f*brightness);
            strip.SetPixelColor(last+i-j,c2);
          }
        }
        if (i>4) {
          strip.SetPixelColor(i-5,black);
        }
        strip.Show();
        timeKeeping(waitForIt(step, steps), last);
      }
    }
  }
  for (int i=0; i<PixelCount; i++) {
    strip.SetPixelColor(i,black);
  }
  strip.Show();
  
}

void loop() {
  state=-1; // Force initialisation
  checkEvent();
  if (state == 0) strip.ClearTo(black);
  if (state==2) { 
    showMustGoOn();
    Serial.println(String("Show finished with state=")+state);
  }  
  timeKeeping(200,PixelCount);
}
