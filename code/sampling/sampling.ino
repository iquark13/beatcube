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

int SAMPLE_RATE_HZ = 20000;             // Sample rate of the audio in hertz.
int LEDS_ENABLED = 1;                  // Control if the LED's should display the spectrum or not.  1 is true, 0 is false.
                                       // Useful for turning the LED display on and off with commands from the serial port.
const int FFT_SIZE = 1024;              // Size of the FFT.  Realistically can only be at most 256 
                                       // without running out of memory for buffers and other state.
const int AUDIO_INPUT_PIN = A13;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 12; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 4;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).

const int MAX_CHARS = 65;              // Max size of the input command buffer
bool sampleSetReady = false;          //primary full sample set ready
bool sampleHalfReady = false;         //half sample set ready


////////////////////////////
// MEL Filter Initialization
////////////////////////////

const int nfilt = 30;                                           //Number of mel filter banks (20-40)
float fbank[nfilt][int(floor(FFT_SIZE/2 + 1))];                 //Filterbank array, need nfilt X fft size +1
float filter_banks;                                             //holder for filterbank calculations
float mel_points[nfilt+2];                                      //individual mel_points (only used in setup, could hardcode if out of memory in progmem)
const int lowFreq = 100;                                        //hz --> frequency at which first melbank starts 100 is old, 200 is new
const float low_freq_mel=2595.0 * log10(1.0+(lowFreq/700.0));   //low frequency mel point.
const float high_freq_mel = 2595.0 * log10(1.0+((SAMPLE_RATE_HZ/2.0)/700.0));   //high frequency mel_point
float hz_points[nfilt+2];                                       //holder for frequency equivalents for mel points
int bin[nfilt+2];                                               //bins for the mel points
float Xfilt[nfilt]={};                                          //output for mel filtered FFT

float hannCoeff[FFT_SIZE];                                      //holds the coefficients for the hann window function.

////////////////////////////////////
/////Deeper work with beat tracking
///////////////////////////////////
float scaledLog[nfilt];             //This is a seperate array to hold the log filtered data --> you could do this in Xfilt instead if you wanted
float maxLog,minLog;                //Old tracking holders, not necessary anymore
float lambda = 5;                   //Lambda multiplier function for the scaledLog filter (range 1-20)
float gam = 1.0;                    //sized for 150hz cutoff with 30kHz sampleing rate DO NOT USE THIS RIGHT NOW.
float XlogHistory[nfilt]={0};       //Log history for calculating the spectral flux
float ODF=0;                        //Onset Detection Function output
float ODFhistory[2];                //History for the onset detection function
const int historyCount=1000;        //How many ODF's to keep in the history
float History[historyCount];        //Used for plotting the ODF history, and used in the 3 conditions of peak detection

///////////////////////////////////
/////Some debug stuff
//////////////////////////////////
byte test = 0;
long int timer1;
long int timer2;
long int timer3;

//////////////////////////////////////////
/////// Onset Detection Parameters
//////////////////////////////////////////

byte w1 = 3; //max frames
byte w3 = 12; //mean frames
byte w5 = 6; //how many frames between onsets
float delta = .25;
float ODFCondition1 = 0;
float ODFCondition2 = 0;
int nlastonset = 0;
int nlastHistory = 0;
bool frameOnset = false;
bool bigHit = false;
////////////////////////////
//LED Stuff
///////////////////////////
#define NUMPIXELS 144
Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_RBG);
byte intensity[NUMPIXELS]={};
float bright;

////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

IntervalTimer samplingTimer;
volatile float32_t sampleHolder[FFT_SIZE]={0};
volatile float32_t samples[FFT_SIZE]={0};
volatile float32_t finalSamples[FFT_SIZE]={0};
volatile float32_t magnitudes[FFT_SIZE/2+1]; //This is the meat and potatoes = holds the 1024 FFT Bins
volatile float32_t output[FFT_SIZE+2];
int sampleCounter = 0;
char commandBuffer[MAX_CHARS];

/////Try some pointers?

float32_t *samp = finalSamples;
float32_t *mag = output;


////////////////////////////////////////////////////////////////////////////////
// Animation Stuff
////////////////////////////////////////////////////////////////////////////////

int hues[NUMPIXELS]={0};
const byte tail_size = 10;
long int animation_timer = 0;
byte animation_speed = 10;
byte head_led = 1;
byte min_increment = 30;

bool rainbow_cylon = true;



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
  ledRainbow(100);


  
  // Begin sampling audio
  samplingBegin();

  //Setup Animation Parameters:
  if (rainbow_cylon){
  for (int i=0;i<NUMPIXELS;i++){
    strip.setPixelColor(i,0,0,0);
  }
  strip.show();

  //Start Amination timer
  animation_timer = millis();
  }
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
    memcpy(finalSamples,samples,FFT_SIZE*sizeof(*samples)); //grab the latest set of samples from the buffer
    arm_dot_prod_f32(finalSamples,hannCoeff,FFT_SIZE,finalSamples);//dot product the hann window with the samples
    arm_rfft_fast_f32(&fft_inst, samp,mag,0);

    output[FFT_SIZE]=output[1]; //Packed nyquist frequency moved to end
    output[1]=output[FFT_SIZE+1]=0;

    arm_cmplx_mag_f32(output,magnitudes,FFT_SIZE/2+1);

    //Set the sample set to not-ready again.
    sampleSetReady = false;
    

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
  }

  // Parse any pending commands.
  parserLoop();

  //Animation Work
  if(rainbow_cylon){
    if ((millis()-animation_timer)>animation_speed){

      increment_cylon();

      if (bright>min_increment){
        if (head_led < NUMPIXELS+tail_size){
          head_led++;
        }
        else{
          head_led = 1;
        }
      }
    animation_timer=millis();
    }
  }
  
  

  
  
}



////////////////////////////////////////////////////////////////////////////////
//
//
// SAMPLING FUNCTIONS
//
//
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  sampleHolder[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 1;
  //We want to sample at 100hz for FFT, so need to copy over and loop every 300 samples
  if (sampleCounter >= 300){
    //need to shift the samples down 300 spaces
    samplingTimer.end();
    memmove(samples,samples+(300),(FFT_SIZE-300) * sizeof(*samples)); //moves sample #300 to samples[0]
    memcpy(samples+(FFT_SIZE-300),sampleHolder,300 * sizeof(*samples));
    sampleSetReady = true;
    sampleCounter = 0;
    samplingBegin();

  }
  if (sampleCounter >= FFT_SIZE) {
    samplingTimer.end();
    memcpy(samples,sampleHolder,sizeof(sampleHolder));
    sampleSetReady = true;
    sampleHalfReady = false;
    samplingBegin();

  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleSetReady;
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
    Serial.println(timer3);
  }
  else if (strcmp(command, "GET ODF") ==0 ) {
    for (int i=0;i<historyCount;++i){
      Serial.println(History[i]);
    }
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
//    maxLog = scaledLog[i]>maxLog ? scaledLog[i] : maxLog;
//    minLog = scaledLog[i]<minLog ? scaledLog[i] : minLog;
  }

  //Now apply the half wave rectifier fuction on the history change
  ODF=0;
  for (int i = 0; i<nfilt;++i){
    //H(x) = (x+ abs(x))/2
    ODF+= ((scaledLog[i]-XlogHistory[i])+fabs(scaledLog[i]-XlogHistory[i]))/2.0;
    XlogHistory[i]=scaledLog[i];
  }
  ODFBuffer(ODF); //store in History[] for use on peak detection

  //Now run through the 3 rules to call it an onset
  ODFCondition1 = 0;
  for (int i=0;i<w1;++i){
    if (History[i+1]>ODFCondition1){
      ODFCondition1=History[i+1];
    }
  }
  
  if (ODF > ODFCondition1){
    
    ODFCondition2 = 0;
    
    for (int i=0; i<w3; ++i) {
      ODFCondition2 += History[i+1];
    }
    
    ODFCondition2 /= w3;

    if (ODF > ODFCondition2+delta){
    //Serial.println(ODF);
    //Serial.println();
    //Serial.println(ODFCondition2+delta);
      if (nlastonset > w5){

        if (maxLog < ODF){
          maxLog = ODF;
          bigHit = true;
        }
        else{
          maxLog *=.99;
        }
        frameOnset = true;
        nlastHistory=nlastonset;
        nlastonset=0;
        
      }
      
    }
    
  }
  
  nlastonset++;
  
  
  
}

void ledIntensity(){
  //sets the intensity of LED's based on scaledLog currently
  //intensities are held in the 'intensity' byte array

//  ODF-=3;
//  ODF = ODF < 0 ? 0 : ODF*20;
//  ODF = ODF > 255 ? 255 : ODF;
//  ODFhistory[1]=ODFhistory[0];
//  if (ODF < .5* ODFhistory[1]){
//    ODF=.5*ODFhistory[1];
//    //FastLED.setBrightness(int(ODF));
//    strip.setBrightness(byte(ODF));
//  }
//  else {
//    ODF=(gam*ODF)+(1-gam)*ODFhistory[1];
//    //FastLED.setBrightness(int(ODF));
//    strip.setBrightness(byte(ODF));
//  }
//  ODFhistory[0]=ODF;
//  strip.show();
//  //Serial.println(byte(ODF));

    if (frameOnset) {
      if (bigHit){
        bright = 100;
        bigHit = false;
        
      }
      else{
        //bright = nlastHistory>11 ? 100 : min(bright * 1.5,100);  //softening the
        bright = max(min(bright*1.5,100),50);   
      }
      
      //bright = 100;
      strip.setBrightness(bright);
      strip.show();
      frameOnset=false;
    }
    else{
      bright *= .9;
      strip.setBrightness(bright);
      strip.show();      
    }

  
}

void ledRainbow(byte bright) {
  //sets the strip to a rainbow set of colors
  
  for (int i=0;i<NUMPIXELS;++i){
    strip.setPixelColor(i,strip.ColorHSV(65535/NUMPIXELS*i,255,bright));
    hues[i]=65535/NUMPIXELS*i;
  }
  strip.show();

}

void ODFBuffer (float nval) {

  for (int i=historyCount-1;i>0;i=i-1){
    History[i]=History[i-1];
    //Serial.println(History[i]);
  }
  History[0]=nval;


  
}

void hannWindowCoeff(int size) {

  for (int i=0;i<size;++i){
    hannCoeff[i]=.5*(1- arm_cos_f32(2*M_PI*i/size));
  }
  
}

void increment_cylon(){
  //head_led should hold the head of the train
  //then you want to go back tail_size. Make sure not to go below 0, or above NUMPIXELS
  if ((head_led > tail_size)&&(head_led<NUMPIXELS+tail_size)){
    for (int i = head_led;i>=head_led-tail_size;i--){
      strip.setPixelColor(i,strip.ColorHSV(hues[i],255,bright));
    }
    strip.setPixelColor(head_led-tail_size,0,0,0);
  }

  strip.show();
}

void clear_strip(){
  //should clear the colors on the entire strip
  strip.fill(strip.Color(0,0,0),0,strip.numPixels());
}

//void triggerShift () {
//
//    memmove(samples,samples+(300),(FFT_SIZE-300) * sizeof(*samples)); //moves sample #300 to samples[0]
//    memcpy(samples+(FFT_SIZE-300),sampleHolder,300 * sizeof(*samples));
//    sampleSetReady = true;
//    sampleCounter = 0;
//  
//}
