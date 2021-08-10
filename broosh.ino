
// Import external libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Geometry.h>
#include <U8g2lib.h>
#include <AsyncDelay.h>
#include "logo.h"

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();
AsyncDelay tick;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

/**
 * Constants that should be given their own .h file in due course.
 */

// Set baud rate
const int BAUD_RATE = 9600;

// Sensor specific 3-point calibration parameters
const float X_AT_REST = 0.15;
const float X_POSITIVE_G = 10.57;
const float X_NEGATIVE_G = -10.45;

const float Y_AT_REST = -0.02;
const float Y_POSITIVE_G = 10.25;
const float Y_NEGATIVE_G = -10.51;

const float Z_AT_REST = -1.92;
const float Z_POSITIVE_G = 8.44;
const float Z_NEGATIVE_G = -11.51;

// OLED specific parameters
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 32;
const int FPS = 1000 / 30;
const int X_SHIFT = 3;

// History of sample taken for moving averages
const int SAMPLE_LENGTH = 16; 

// Target brush duration for each type of stroke
const int STROKE_DURATION = 10;
const int SAMPLE_FREQUENCY = 20;

// Number of unique brush strokes
const int NUM_UNIQUE_STROKES = 6;

// Sine values for FFT
const byte sine_data [91]=
 {
0,  
4,    9,    13,   18,   22,   27,   31,   35,   40,   44, 
49,   53,   57,   62,   66,   70,   75,   79,   83,   87, 
91,   96,   100,  104,  108,  112,  116,  120,  124,  127,  
131,  135,  139,  143,  146,  150,  153,  157,  160,  164,  
167,  171,  174,  177,  180,  183,  186,  189,  192,  195,
198,  201,  204,  206,  209,  211,  214,  216,  219,  221,  
223,  225,  227,  229,  231,  233,  235,  236,  238,  240,  
241,  243,  244,  245,  246,  247,  248,  249,  250,  251,  
252,  253,  253,  254,  254,  254,  255,  255,  255,  255
  };

/**
 * Variable declarations that should be given their own .h file in due course.
 */

// Storage and accessibility of accelerometer calibration parameters
float X_PARAMS[3] = {X_AT_REST, X_POSITIVE_G, X_NEGATIVE_G};
float Y_PARAMS[3] = {Y_AT_REST, Y_POSITIVE_G, Y_NEGATIVE_G};
float Z_PARAMS[3] = {Z_AT_REST, Z_POSITIVE_G, Z_NEGATIVE_G};
float *CAL_PARAMS[3] = {X_PARAMS, Y_PARAMS, Z_PARAMS};

// Top 5 frequencies peaks in descending order for FFT
float f_peaks[5]; 

// Anchoring position of LED screem
int pos_x = 0;
int pos_y = 0;

// Create arrays for storing accelerometer history
int cur_hist[3];
int hist[3][SAMPLE_LENGTH];
int hist_fill = 0;

// Create arrays for storing frequency history
int cur_freq[3];
int freq[3][SAMPLE_LENGTH];
int freq_fill = 0;

// Declaration of moving averages orientation
double orientation[3];

// Declaration of stroke duration and stroke tracker variables
int stroke_time[NUM_UNIQUE_STROKES + 1];
int last_stroke = 0;
int current_stroke = 0;
unsigned long start_stroke_time, end_stroke_time;

// Starting value assumed to be 100
unsigned long period = 100;

void setup(void) {

    Serial.begin(BAUD_RATE);  
    u8g2.begin();
    tick.start(FPS, AsyncDelay::MILLIS);

    // Check if accelerometer is detected
    if(!accel.begin()) {
        Serial.println("Accelerometer not detected.");
        while(true);
    }
}

void loop() {

    // Start time of each sample
    unsigned long start = millis();

    // Take accelerometer reading
    sensors_event_t event; 
    accel.getEvent(&event);
    cur_hist[0] = (int) (calibrate(event.acceleration.x, X_AT_REST, X_POSITIVE_G, X_NEGATIVE_G) * 100);
    cur_hist[1] = (int) (calibrate(event.acceleration.y, Y_AT_REST, Y_POSITIVE_G, Y_NEGATIVE_G) * 100);
    cur_hist[2] = (int) (calibrate(event.acceleration.z, Z_AT_REST, Z_POSITIVE_G, Z_NEGATIVE_G) * 100);
    
    // Push accelerometer reading to array
    push(hist, &hist_fill, cur_hist);

    // Only attempt to calculate frequency if array of accelerometer readings is filled
    if (hist_fill == SAMPLE_LENGTH) {

        for (int j = 0; j < 3; j++) {

            // Use Fast Fourier Transform to calculate frequency
            FFT(hist[j], SAMPLE_LENGTH, (int) (1000 / period));

            // Frequency range must be above a certain threshold to be classified as non-stationary
            if (minmax(hist[j]) < 200) {
                cur_freq[j] = 0;
            } else {
                cur_freq[j] = (int) (f_peaks[0] * 100);
            }
            
        }

        // Push frequency calculated to array
        push(freq, &freq_fill, cur_freq);

        // Calculate moving average of accelerometer readings to get orientation
        for (int j = 0; j < 3; j++) {
            orientation[j] = average(hist[j]);
        }

        // Determine which stroke brush on this sample
        current_stroke = check_stroke(orientation, cur_freq);
        if (last_stroke != current_stroke) {
            // End of the current stroke. Add total time to stroke times 
            if (last_stroke != 0) {
                end_stroke_time = millis();
                stroke_time[last_stroke] += end_stroke_time - start_stroke_time;
            }
            // Beginning of a new stroke
            if (current_stroke != 0) {
                start_stroke_time = millis();
            }
            // Update strokes
            last_stroke = current_stroke;
        }

        // Print diagnostics
//        Serial.print("Accel X: "); Serial.print((double) hist[0][SAMPLE_LENGTH - 1] / 100); Serial.print("  ");
//        Serial.print("Accel Y: "); Serial.print((double) hist[1][SAMPLE_LENGTH - 1] / 100); Serial.print("  ");
//        Serial.print("Accel Z: "); Serial.print((double) hist[2][SAMPLE_LENGTH - 1] / 100); Serial.println("  ");
//        Serial.print("Orient X: "); Serial.print(orientation[0]); Serial.print("  ");
//        Serial.print("Orient Y: "); Serial.print(orientation[1]); Serial.print("  ");
//        Serial.print("Orient Z: "); Serial.print(orientation[2]); Serial.println("  ");
//        Serial.print("Freq X: "); Serial.print((double) freq[0][SAMPLE_LENGTH - 1] / 100); Serial.print("  ");
//        Serial.print("Freq Y: "); Serial.print((double) freq[1][SAMPLE_LENGTH - 1] / 100); Serial.print("  ");
//        Serial.print("Freq Z: "); Serial.print((double) freq[2][SAMPLE_LENGTH - 1] / 100); Serial.println("  ");

        for (int j = 1; j < 7; j++) {
            Serial.print("(" + j + "): " + strokes[j] + " ");
        } Serial.println();
        
        period = millis() - start;
        // Serial.print("Period: "); Serial.print(period); Serial.println(" ");
    }
    
    // Update OLED after every tick
    if (tick.isExpired()) {
        drawAnimation();
        display_text(strokes);
        pos_x = pos_x + X_SHIFT;
        if (pos_x > OLED_WIDTH + logo_width) {
            pos_x = -logo_width;
        }
        tick.repeat();
    }

}

/**
 * Three point calibration for each axis.
 */
double calibrate(double val, double val_at_rest, double val_positive_G, double val_negative_G) {
  if (val > val_at_rest){
    return (val - val_at_rest) * 9.81 / (val_positive_G - val_at_rest);
  } else {
    return (val - val_at_rest) * 9.81 / (val_at_rest - val_negative_G);
  }
}

/**
 * Pushes new elements into the array and accounts for overflow.
 */
void push(int hist[3][SAMPLE_LENGTH], int * fill, int cur[3]) {
    if (*fill == SAMPLE_LENGTH) {
        for (int i = 0; i < SAMPLE_LENGTH - 1; i++) {
            for (int j = 0; j < 3; j++) {
              hist[j][i] = hist[j][i+1];
            }
        }
        for (int j = 0; j < 3; j++) {
            hist[j][SAMPLE_LENGTH - 1] = cur[j];
        }       
    } else {
        for (int j = 0; j < 3; j++) {
            hist[j][*fill] = cur[j];
        } 
        (*fill)++;
    }
}

/**
 * Calculates average of SAMPLE_LENGTH.
 */
double average(int hist[SAMPLE_LENGTH]) {
    double sum = 0;
    for (int i = 0; i < SAMPLE_LENGTH; i++) {
        sum += (double) hist[i] / 100;
    }
    return sum / SAMPLE_LENGTH;
}

/**
 * Calculates the range of the data in the array.
 */
double minmax(int hist[SAMPLE_LENGTH]) {
    double minimum = hist[0];
    double maximum = hist[0];
    for (int i = 1; i < SAMPLE_LENGTH; i++) {
        if (hist[i] > maximum) {
            maximum = hist[i];
        }
        if (hist[i] < minimum) {
            minimum = hist[i];
        }
    }
    return maximum - minimum;
}

/**
 * Fast Fourier Transform to quickly find frequency of brushing.
 */
float FFT(int in[],int N,float Frequency) {

    unsigned int data[13]={1,2,4,8,16,32,64,128,256,512,1024,2048};
    int a,c1,f,o,x;
    a=N;                                       
    for(int i=0;i<12;i++)                 //calculating the levels
        { if(data[i]<=a){o=i;} }
          
    int in_ps[data[o]]={};     //input for sequencing
    float out_r[data[o]]={};   //real part of transform
    float out_im[data[o]]={};  //imaginory part of transform               
    x=0;  
    for(int b=0;b<o;b++) {                   // bit reversal
        c1=data[b];
        f=data[o]/(c1+c1);
        for(int j=0;j<c1;j++) { 
            x=x+1;
            in_ps[x]=in_ps[j]+f;
        }
    }     
    for(int i=0;i<data[o];i++) {           // update input array as per bit reverse order
        if(in_ps[i]<a)
        {out_r[i]=in[in_ps[i]];}
        if(in_ps[i]>a)
        {out_r[i]=in[in_ps[i]-a];}      
    }    
    int i10,i11,n1;
    float e,c,s,tr,ti;
    
    for(int i=0;i<o;i++) {                                   //fft
        i10=data[i];              // overall values of sine/cosine:
        i11=data[o]/data[i+1];    // loop with similar sine cosine:
        e=360/data[i+1];
        e=0-e;
        n1=0;
    
        for(int j=0;j<i10;j++) {
            c=cosine(e*j);
            s=sine(e*j);    
            n1=j;
              
            for(int k=0;k<i11;k++){
                tr=c*out_r[i10+n1]-s*out_im[i10+n1];
                ti=s*out_r[i10+n1]+c*out_im[i10+n1];
                out_r[n1+i10]=out_r[n1]-tr;
                out_r[n1]=out_r[n1]+tr;
                out_im[n1+i10]=out_im[n1]-ti;
                out_im[n1]=out_im[n1]+ti;          
                n1=n1+i10+i10;
            }       
        }
    }

    for(int i=0;i<data[o-1];i++) {              // getting amplitude from compex number
        out_r[i]=sqrt(out_r[i]*out_r[i]+out_im[i]*out_im[i]); // to  increase the speed delete sqrt
        out_im[i]=i*Frequency/N; 
    }
    x=0;       // peak detection
    for(int i=1;i<data[o-1]-1;i++) {
        if(out_r[i]>out_r[i-1] && out_r[i]>out_r[i+1]) {
            in_ps[x]=i;    //in_ps array used for storage of peak number
            x=x+1;
        }    
    }
    s=0;
    c=0;
    for(int i=0;i<x;i++) {            // re arraange as per magnitude
        for(int j=c;j<x;j++) {
            if(out_r[in_ps[i]]<out_r[in_ps[j]]) {
                s=in_ps[i];
                in_ps[i]=in_ps[j];
                in_ps[j]=s;
            }
        }
        c=c+1;
    }
    for(int i=0;i<5;i++) {     // updating f_peak array (global variable)with descending order
        f_peaks[i]=out_im[in_ps[i]];
    }
}

/**
 * Assigns each orientation to a pre-determined stroke
 * and counts it in only if frequency is above a certain
 * threshold.
 */
int check_stroke(double cur_hist[3], int cur_freq[3]) {

    int MIN_THRESH = 5.00;
    int MAX_THRESH = 7.00;
    int FREQ_THRESH = 600;
  
    if (abs(cur_hist[1]) < MIN_THRESH && abs(cur_hist[2]) < MIN_THRESH && (cur_freq[0] + cur_freq[1] + cur_freq[2] > FREQ_THRESH)){
        if (cur_hist[0] >= MAX_THRESH) {
            // Serial.println("1");
            return 1;
        } else if (cur_hist[0] <= -MAX_THRESH) {
            // Serial.println("2");
            return 2;
        }
    } else if (abs(cur_hist[0]) < MIN_THRESH && abs(cur_hist[2]) < MIN_THRESH && (cur_freq[0] + cur_freq[1] + cur_freq[2] > FREQ_THRESH)) {
        if (cur_hist[1] >= MAX_THRESH) {
            // Serial.println("3");
            return 3;
        } else if (cur_hist[1] <= -MAX_THRESH) {
            // Serial.println("4");
            return 4;
        }
    } else if (abs(cur_hist[0]) < MIN_THRESH && abs(cur_hist[1]) < MIN_THRESH && (cur_freq[0] + cur_freq[1] + cur_freq[2] > FREQ_THRESH)) {
        if (cur_hist[2] >= MAX_THRESH) {
            // Serial.println("5");
            return 5;
        } else if (cur_hist[2] <= -MAX_THRESH) {
            // Serial.println("6");
            return 6;
        }
    } else {
        return 0;
    }
}

void drawAnimation() {
  u8g2.firstPage();
  do {
    u8g2.drawXBMP(pos_x, pos_y, logo_width, logo_height, logo_right);
  } while ( u8g2.nextPage() );
}

void display_text(int strokes[7]) {

    String display_string;
    
    for (int j = 1; j < 7; j++) {
        if (strokes[j] > 10000) {
            display_string = display_string + j;
        }
    }

    char stroke[10];
    display_string.toCharArray(stroke, 10);
    
    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(2, 15, stroke);
    } while ( u8g2.nextPage() );
}
    

float sine(int i) {
  int j=i;
  float out;
  while(j<0){j=j+360;}
  while(j>360){j=j-360;}
  if(j>-1   && j<91){out= sine_data[j];}
  else if(j>90  && j<181){out= sine_data[180-j];}
  else if(j>180 && j<271){out= -sine_data[j-180];}
  else if(j>270 && j<361){out= -sine_data[360-j];}
  return (out/255);
}

float cosine(int i) {
  int j=i;
  float out;
  while(j<0){j=j+360;}
  while(j>360){j=j-360;}
  if(j>-1   && j<91){out= sine_data[90-j];}
  else if(j>90  && j<181){out= -sine_data[j-90];}
  else if(j>180 && j<271){out= -sine_data[270-j];}
  else if(j>270 && j<361){out= sine_data[j-270];}
  return (out/255);
}
