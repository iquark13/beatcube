// Audio Spectrum Display
// Copyright 2013 Tony DiCola (tony@tonydicola.com)

// This code is part of the guide at http://learn.adafruit.com/fft-fun-with-fourier-transforms/

#define ARM_MATH_CM4          //Fast Math Functions for Teensy
#include <arm_math.h>         //Fast Math Functions for Teensy (DSP 1.5.3, CMSISv5.3)
#include <Adafruit_DotStar.h> //LED Library
#include <SPI.h>              //LED Coms



////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION 
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

int SAMPLE_RATE_HZ = 30000;             // Sample rate of the audio in hertz.
int LEDS_ENABLED = 1;                  // Control if the LED's should display the spectrum or not.  1 is true, 0 is false.
                                       // Useful for turning the LED display on and off with commands from the serial port.
const int FFT_SIZE = 1024;              // Size of the FFT.  Realistically can only be at most 256 
                                       // without running out of memory for buffers and other state.
const int AUDIO_INPUT_PIN = A2;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 12; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 4;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).

const int MAX_CHARS = 65;              // Max size of the input command buffer


////////////////////////////
// MEL Filter Initialization
////////////////////////////

const int nfilt = 30;
volatile float fbank[nfilt][int(floor(FFT_SIZE/2 + 1))];
volatile float filter_banks;
volatile float mel_points[nfilt+2];
const int lowFreq = 40; //hz
const float low_freq_mel=2595.0 * log10(1.0+(lowFreq/700.0));
const float high_freq_mel = 2595.0 * log10(1.0+((SAMPLE_RATE_HZ/2.0)/700.0));
volatile float hz_points[nfilt+2];
volatile int bin[nfilt+2];
volatile float Xfilt[nfilt]={};

////////////////////////////////////
/////Deeper work with beat tracking
///////////////////////////////////
volatile float scaledLog[nfilt];
float maxLog,minLog;
float lambda = 1.0;
float XlogHistory[nfilt]={};
float ODF=0;
float ODFhistory[10];

byte test = 0;
long int timer1;

////////////////////////////
//Dotstar Stuff
///////////////////////////
#define NUMPIXELS 144
Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_RBG);
byte intensity[NUMPIXELS]={};

////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

IntervalTimer samplingTimer;
float32_t samples[FFT_SIZE];
float32_t magnitudes[FFT_SIZE/2+1]; //This is the meat and potatoes = holds the 1024 FFT Bins
float32_t output[FFT_SIZE+2];
int sampleCounter = 0;
char commandBuffer[MAX_CHARS];

/////Try some pointers?

float32_t *samp = samples;
float32_t *mag = output;


////////////////////////////////////////////////////////////////////////////////
// MAIN SKETCH FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set up serial port.
  Serial.begin(115200);
  Serial.println("Test printing");
  
  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);
  analogReference(0);
  
  // Turn on the power indicator LED.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);
  
  
  // Clear the input command buffer
  memset(commandBuffer, 0, sizeof(commandBuffer));
  
  //Setup Mel Filters
  genMelFilter();

  //Setup led strip
  strip.begin();
  strip.show();
  ledRainbow();

  
  // Begin sampling audio
  samplingBegin();
}

void loop() {
  // Calculate FFT if a full sample is available.
  if (samplingIsDone()) {
    timer1=micros();
    // Run FFT on sample data.
//    arm_cfft_radix4_instance_f32 fft_inst;
//    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
//    arm_cfft_radix4_f32(&fft_inst, samples);
//    // Calculate magnitude of complex numbers output by the FFT.
//    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

    //new faster real FFT from the CMSIS-DSP v5.3 (v1.5.2 dsp)
    arm_rfft_fast_instance_f32 fft_inst;
    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    arm_rfft_fast_f32(&fft_inst, samp,mag,0);

    output[FFT_SIZE]=output[1];
    output[1]=output[FFT_SIZE+1]=0;

    arm_cmplx_mag_f32(output,magnitudes,FFT_SIZE/2+1);

    //Set the DC offset value to 0 because it is useless
    magnitudes[0]=0;

    //get the melfiltered data into Xfilt
    applyMelFilter();

    //now scale and set max/min
    scaleMaxMel();
    
  
    if (LEDS_ENABLED == 1)
    {
      ledIntensity();
    }
    // Restart audio sampling.
    timer1=micros()-timer1;
    samplingBegin();
  }
    
  // Parse any pending commands.
  parserLoop();

//  if (millis()-timer1 >= 1000){
//  if (test == 0){
//    digitalWrite(POWER_LED_PIN,HIGH);
//    test =1;
//    timer1=millis();
//  }
//  else {
//    digitalWrite(POWER_LED_PIN,LOW);
//    test =0;
//    //Serial.println(magnitudes[1]);
//    timer1=millis();
//  }
//  }

  //Serial.println(analogRead(10));
  
}


////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////







////////////////////////////////////////////////////////////////////////////////
// SPECTRUM DISPLAY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 1;
  if (sampleCounter >= FFT_SIZE) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE;
}


////////////////////////////////////////////////////////////////////////////////
// COMMAND PARSING FUNCTIONS
// These functions allow parsing simple commands input on the serial port.
// Commands allow reading and writing variables that control the device.
//
// All commands must end with a semicolon character.
// 
// Example commands are:
// GET SAMPLE_RATE_HZ;
// - Get the sample rate of the device.
// SET SAMPLE_RATE_HZ 400;
// - Set the sample rate of the device to 400 hertz.
// 
////////////////////////////////////////////////////////////////////////////////

void parserLoop() {
  // Process any incoming characters from the serial port
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Add any characters that aren't the end of a command (semicolon) to the input buffer.
    if (c != ';') {
      c = toupper(c);
      strncat(commandBuffer, &c, 1);
    }
    else
    {
      // Parse the command because an end of command token was encountered.
      parseCommand(commandBuffer);
      // Clear the input buffer
      memset(commandBuffer, 0, sizeof(commandBuffer));
    }
  }
}

// Macro used in parseCommand function to simplify parsing get and set commands for a variable
#define GET_AND_SET(variableName) \
  else if (strcmp(command, "GET " #variableName) == 0) { \
    Serial.println(variableName); \
  } \
  else if (strstr(command, "SET " #variableName " ") != NULL) { \
    variableName = (typeof(variableName)) atof(command+(sizeof("SET " #variableName " ")-1)); \
  }

void parseCommand(char* command) {
  if (strcmp(command, "GET MAGNITUDES") == 0) {
    for (int i = 0; i < FFT_SIZE/2+1; ++i) {
      Serial.println(magnitudes[i]);
    }
  }
  else if (strcmp(command, "GET SAMPLES") == 0) {
    for (int i = 0; i < FFT_SIZE; i+=1) {
      Serial.println(samples[i]);
    }
  }
  else if (strcmp(command, "GET FFT_SIZE") == 0) {
    Serial.println(FFT_SIZE);
  }
  else if (strcmp(command, "GET FBANK") == 0) {
    for (int i = 0; i < FFT_SIZE/2+1; ++i) {
      Serial.println(fbank[9][i]);
    }
  }
  else if (strcmp(command, "GET MEL") == 0) {
    Serial.println(low_freq_mel);
    Serial.println(high_freq_mel);
  }
  else if (strcmp(command, "GET XFILT") ==0 ) {
    for (int i=0;i<nfilt;++i){
      Serial.println(Xfilt[i]);
    }
  }
  else if (strcmp(command, "GET BIN") ==0 ) {
    for (int i=0;i<nfilt+2;++i){
      Serial.println(bin[i]);
    }
  }
  else if (strcmp(command, "GET HZ") ==0 ) {
    for (int i=0;i<nfilt+2;++i){
      Serial.println(hz_points[i]);
    }
  }
  else if (strcmp(command, "GET SCALEDLOG") ==0 ) {
    for (int i=0;i<nfilt;++i){
      Serial.println(scaledLog[i]);
    }
  }
  else if (strcmp(command, "GET TIME") == 0) {
    Serial.println(timer1);
  }
  GET_AND_SET(SAMPLE_RATE_HZ)
  GET_AND_SET(LEDS_ENABLED)

  
  // Update spectrum display values if sample rate was changed.
  if (strstr(command, "SET SAMPLE_RATE_HZ ") != NULL) {
    //spectrumSetup();
  }
  
  // Turn off the LEDs if the state changed.

}

void genMelFilter() {
    mel_points[0]=low_freq_mel;
    mel_points[nfilt+2-1]=high_freq_mel;
    for (int i = 1; i < nfilt+2-1; ++i) {
      mel_points[i]=((high_freq_mel-low_freq_mel)/(nfilt+2-1))+mel_points[i-1];
    }


    //inefficient, but this is just setup
    for (int i = 0; i< nfilt+2; ++i) {
      hz_points[i]=700.0 * (pow(10.0,(mel_points[i]/2595.0))-1.0);
    }

    for (int i = 0; i<nfilt+2; ++i) {
      bin[i]=int(floor((FFT_SIZE+1)*hz_points[i]/SAMPLE_RATE_HZ));
    }
    int f_m_minus,f_m,f_m_plus;
    for (int m = 1; m<nfilt+2-1; ++m){
      f_m_minus = int(bin[m-1]);
      f_m = int(bin[m]);
      f_m_plus = int(bin[m+1]);

      for (int k = f_m_minus; k<f_m;k++){
        fbank[m-1][k]=float((k-bin[m-1])) / (bin[m] - bin[m-1]);
      }
      for (int k = f_m;k<f_m_plus; k++){
        fbank[m - 1][k] = float((bin[m + 1] - k)) / (bin[m + 1] - bin[m]);
      }
      }
    } 

void applyMelFilter() {
  //data is in magnitudes[FFT_SIZE/2+1]
  //Process is to dot product fbank[i].magnitudes[i] and store in new vector Xfilt[nfilt+2]
  //use the fast ARM process arm_dot_prod_f32

  for (int i=0;i<nfilt; ++i){
    Xfilt[i]=0.0;
    float32_t *filter=fbank[i];
    arm_dot_prod_f32(filter,magnitudes,FFT_SIZE/2+1,&Xfilt[i]);
    
  }

  
}

void scaleMaxMel() {

  //log scale everything for human hearing
  //scale to maximum on whatever box is highest
  // store to scaledLog[]
  // First FOR is doing logarithimic scaling, and storing the highest and lowest value
  // Second FOR is storing the history of the highest value
  // Next we find the max across the last logHistory, and store in logHistoryMax
  // Last FOR sets the range of brightnesses we want and scales everything.
  
  minLog=10000000;
  maxLog=0;

  //Equation #1 applied to the Mel Filtered signal
  for (int i =0;i<nfilt;++i){
    scaledLog[i]=log10(lambda*Xfilt[i] + 1.0);
    maxLog = scaledLog[i]>maxLog ? scaledLog[i] : maxLog;
    minLog = scaledLog[i]<minLog ? scaledLog[i] : minLog;
  }

  //Now apply the half wave rectifier fuction on the history change
  ODF=0;
  for (int i = 0; i<nfilt;++i){
    //H(x) = (x+ abs(x))/2
    ODF+= ((scaledLog[i]-XlogHistory[i])+fabs(scaledLog[i]-XlogHistory[i]))/2.0;
    XlogHistory[i]=scaledLog[i];
  }
  
  
  
  
}

void ledIntensity(){
  //sets the intensity of LED's based on scaledLog currently
  //intensities are held in the 'intensity' byte array

  float gamma = .99; //exponential filtering
  //scaledBrightness= ODF< 0.0 ? 0.0 : scaledBrightness;
  ODF-=5;
  ODF = ODF < 0 ? 0 : ODF*5;
  ODFhistory[1]=ODFhistory[0];
  ODFhistory[0]=ODF;
  ODF=(gamma*ODF)+(1-gamma)*ODFhistory[1];
  strip.setBrightness(int(ODF));
  strip.show();

  
}

void ledRainbow() {
  //sets the strip to a rainbow set of colors
  for (int i=0;i<NUMPIXELS;++i){
    strip.setPixelColor(i,strip.ColorHSV(65536/NUMPIXELS*i,255,255));
  }
  strip.show();
}

    

  
