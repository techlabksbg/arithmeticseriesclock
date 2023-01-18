#include <Arduino.h>

#include <time.h> 
#include <sys/time.h>

#include "dcf77simple.h"
#include "NeoPixelBus.h"
#include "esp_adc_cal.h"

#define LED_PIN 13
#define DCF77_PIN 22
#define BRIGHTNESS_PIN 34

// DCF77 pins are: VCC, GND, SIGNAL, EN (tie to GND)

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
    {18,8,5},
    {18,13,45},
    {18,58,5},
    {19,03,45},
    {19,48,5},
    {19,53,45},
    {20,38,5},
};
  


//Adafruit_NeoPixel strip(PixelCount, PIN, NEO_RGB + NEO_KHZ800);
// Uses GPIO2 alias D4
//NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, 42); // pin is ignored
NeoPixelBus<NeoRgbFeature, NeoEsp32I2s1800KbpsMethod> strip(PixelCount, LED_PIN);


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
                    l = 4*l*l*l*brightness;
                    strip.SetPixelColor(x, HslColor(hue, 1.0, l));
                    hues[x] = hue;
                }
            }
        }
    }
};

void printLocalTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

unsigned long lastData = 0;

void dcf2esp() {
  if (DCF77.decode()) {
    DCF77.showData();
    lastData = DCF77.lastData;
    tm local;
    local.tm_year = DCF77.timeInfo.year - 1900;
    local.tm_mon = DCF77.timeInfo.month-1;
    local.tm_mday = DCF77.timeInfo.day;
    local.tm_hour = DCF77.timeInfo.hour;
    local.tm_min = DCF77.timeInfo.minute;
    local.tm_sec = DCF77.timeInfo.second;
    const time_t sec = mktime(&local)+3600;
    timeval tv;
    tv.tv_sec = sec;
    Serial.printf("Have sec=%ld\n", sec);
    tv.tv_usec = 1000*DCF77.timeInfo.ms;
    settimeofday(&tv, NULL);
    printLocalTime();
  }
}

void fakeTime() {
    tm local;
    local.tm_year = 123;
    local.tm_mon = 0;
    local.tm_mday = 16;
    
    local.tm_hour = 20;
    local.tm_min = 42; // 41;
    local.tm_sec = 50;
    const time_t sec = mktime(&local);
    timeval tv;//= {tv_sec=sec, tv_usec=0};
    tv.tv_sec = sec;
    tv.tv_usec = 0;

    Serial.printf("Have sec=%ld\n", sec);
    settimeofday(&tv, NULL);
    printLocalTime();
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
        strip.SetPixelColor(pos, RgbColor(12, 0, 0));
      } else {
        strip.SetPixelColor(pos, RgbColor(6, 6, 0));
      }
    } else { // LOW
      if (DCF77.nr != -1) {
        strip.SetPixelColor(pos, RgbColor(0, 12, 0));
      } else {
        strip.SetPixelColor(pos, RgbColor(0, 0, 6));
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
  int s = (int)(steps);   // Floor of total steps
  int x = 2*PixelCount+1;    // helpful constant
  int n = (int)(0.5*(x-sqrt(x*x-8*s)));   // How many interations in [0..PixelCount-1])
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
    float t = 0.01;
    hues[i] = hsvInterpolate(hues[i], hues[i-1], t);
    strip.SetPixelColor(i, HslColor(hues[i], 1.0, 0.5*brightness));
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

// Get t in [0,1] of the current period (or -1 if none applies)
float getCurrentT() {
  time_t now;
  time(&now);
  struct tm  info;
  localtime_r(&now, &info);
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  
  int daymins = info.tm_hour*60+info.tm_min;
  for(int i=0;i<sizeof(events)/12;i++) {
    int mins = events[i][0]*60+events[i][1];
    if (daymins>=mins && daymins<mins+events[i][2]) {
      return ((info.tm_hour*60+info.tm_min-mins)*60+info.tm_sec+tv_now.tv_usec/1e6)/(events[i][2]*60);
    }
  }
  return -1.0;
}

slider sliders[2];

void initHues() {
  for (int i=0; i<PixelCount; i++) {
      hues[i] = random(10000)/10000.0f;
   }
  for (int i=0; i<2; i++) {
    sliders[i].active = false;
  }
}

void paintStrip() {
  float t = getCurrentT();
  //Serial.printf("Got t=%f  ",t);
  if (t<0) {
    strip.ClearTo(black);
    strip.Show();
  } else {
    iterPos a = iterAndPos(t);
    //Serial.printf("a.iter=%d, a.pos=%f\n", a.iter, a.pos);
    if (a.iter==0) { // Reset if next period
      for (int i=0; i<2; i++) {
        if (sliders[i].last<PixelCount) {
          sliders[i].active = false;
        }
      }
    }
    if (!sliders[0].active) {   // Slider 0 not active
      if (sliders[1].active) {   // Either move slider 1 to 0
        sliders[0] = sliders[1];
        sliders[1].active = false;
      } else {                   // Or make a new slider
        sliders[0].active = true;
        sliders[0].hue = random(10000)/10000.0f;
        sliders[0].last = PixelCount - a.iter;
      }
    } else if (sliders[0].last!=PixelCount - a.iter) {  // Slider is active, but not current any more
      if (!sliders[1].active) {   // Slider 1 is inactive
        sliders[1].active = true;
        sliders[1].hue = random(10000)/10000.0f;
        sliders[1].last = PixelCount - a.iter;
      } // both active? That's enough ;-)
    }
    for (int i=0; i<2; i++) {
      if (sliders[i].active) {
        sliders[i].hue+=0.0001;
        if (sliders[i].hue>=1.0) {
          sliders[i].hue = 0.0;
        }
        if (sliders[i].last==PixelCount - a.iter) {
          sliders[i].paint(a.pos);
        } else {
          sliders[i].paint(a.pos+sliders[i].last);
        }
      }
    }
    //Serial.println(PixelCount-1 - a.iter);
    diffuse(PixelCount-1 - a.iter);
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

  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/ 3", 1); // https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
  tzset();

  DCF77.begin(DCF77_PIN);
  //sliderTest();
  waitForTimeFix();
  //fakeTime();
  initHues();
}

void loop() {
  brightness = 0.98*brightness + 0.02*(analogRead(BRIGHTNESS_PIN)/4096.0*0.95+0.05);
  if (DCF77.lastData!=lastData) {
    dcf2esp();
    lastData = DCF77.lastData;
  }
  paintStrip();
  strip.Show();
  delay(5);
}