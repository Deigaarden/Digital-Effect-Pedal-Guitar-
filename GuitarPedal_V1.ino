//This program was developed during the 7th semester of 2024-2025 and is the associated software for a guitar effect pedal featuring an ESP32 developed during the same period - JVDS

//I2C library
#include <Wire.h>

//Codec library
#include <SparkFun_WM8960_Arduino_Library.h>
WM8960 codec; 

// Include I2S driver
#include <driver/i2s.h>

//Flags
#define DELAY_FLAG_PIN 18
#define DISTORTION_FLAG_PIN 19

// LEDs
#define DELAY_LED_PIN 5
#define DISTORTION_LED_PIN 2

//Internal ADC li
#include <driver/adc.h>
#define BlendPot ADC1_CHANNEL_6      // GPIO34  (nr. 4) Control Blend Koefficient
#define FeedbackPot ADC1_CHANNEL_7   // GPIO35  (nr. 3) Control Feedback Koefficient
#define CrossoverPot ADC1_CHANNEL_4  // GPIO32  (nr. 2) Crossover for IIR-Filter
#define GainPot ADC1_CHANNEL_5       // GPIO33  (nr. 1) Gain for distortion 

// Use I2S peripheral number 0
#define I2S_PORT I2S_NUM_0

// Connections to I2S
#define I2S_WS 25
#define I2S_SD 17
#define I2S_SDO 4
#define I2S_SCK 16

// Define the DMA buffer length
#define bufferLen 64

//Buffers for processing the audio
int16_t sBuffer[bufferLen];            //DMA Buffer
int16_t AudioFloat[bufferLen];         //Float Audio
int16_t AudioInt[bufferLen];           //Integer Audio
float DistortedOutput[bufferLen];      //Distorted Audio
float IIRfilterOutput[bufferLen];      //IIR-Filter buffer
float DelayOutput[bufferLen];          //Delay-buffer


//Variables & buffers for FIR filter
const int FirTaps = 25;               // Number of filter taps

float FilterCoefficients[FirTaps] = {  0.008864098229,  0.01387221273, 0.0007424467476, -0.02567267977, -0.03081503138,
                                       0.004488705192,  0.03878715262, 0.009480092674, -0.06655570865, -0.07312383503,
                                       0.07640399784,   0.3033539653,   0.4136131704,   0.3033539653,  0.07640399784,
                                       -0.07312383503, -0.06655570865, 0.009480092674,  0.03878715262, 0.004488705192,
                                       -0.03081503138, -0.02567267977, 0.0007424467476,  0.01387221273, 0.008864098229
                                    };  // FIR FilterCoefficientsicients h[n]

float CircularBuffer[FirTaps] = {0};  // Circular buffer for FIR filter
int CircIndex = 0;                    // Pointer for circular buffer

// IIR filter
float y_prev = 0.0;



// Effect states  (Named Integer constants 0, 1, 2...)
enum EffectState {
  NoEffect,
  OnlyDelay,
  OnlyDistortion,
  DelayFirstThenDistortion,
  DistortionFirstThenDelay   
};

EffectState currentState = NoEffect;

// Bools:
volatile bool delayButtonPressed = false;
volatile bool distortionButtonPressed = false;

//Debounce variables:
unsigned long delayButtonTime = 0;
unsigned long distortionButtonTime = 0;


void onDelayButtonPress() {
  if (digitalRead(DELAY_FLAG_PIN) == LOW) {
    delayButtonPressed = true;
    delayButtonTime = millis();
  } else {
    delayButtonPressed = false;
  }
}

void onDistortionButtonPress() {
  if (digitalRead(DISTORTION_FLAG_PIN) == LOW) {
    distortionButtonPressed = true;
    distortionButtonTime = millis(); 
  } else {
    distortionButtonPressed = false;
  }
}


// Update state upon power up and in each loop.
void DetermineState() {
  // Determine state, then use switch-case to process
  if (digitalRead(DELAY_FLAG_PIN) == HIGH && digitalRead(DISTORTION_FLAG_PIN) == HIGH) {
    currentState = NoEffect;
  } 
  else if (digitalRead(DELAY_FLAG_PIN) == LOW && digitalRead(DISTORTION_FLAG_PIN) == HIGH) {
    currentState = OnlyDelay;
  } 
  else if (digitalRead(DELAY_FLAG_PIN) == HIGH && digitalRead(DISTORTION_FLAG_PIN) == LOW) {
    currentState = OnlyDistortion;
  } 
  else if (digitalRead(DELAY_FLAG_PIN) == LOW && digitalRead(DISTORTION_FLAG_PIN) == LOW) {
    currentState = DistortionFirstThenDelay;   // Since DelayThenDistortion sounds bad we dont enter this effect. 
  }
}


// Updates LEDs dependent on flags
void LED_Update() {
  if (digitalRead(DELAY_FLAG_PIN) == LOW) {
    digitalWrite(DELAY_LED_PIN, HIGH);
  } else {
    digitalWrite(DELAY_LED_PIN, LOW);
  }

  if (digitalRead(DISTORTION_FLAG_PIN) == LOW) {
    digitalWrite(DISTORTION_LED_PIN, HIGH);
  } else {
    digitalWrite(DISTORTION_LED_PIN, LOW);
  }
}



void setup() {

  // Set up I2C
  Wire.begin();

  // Configure pins as input
  pinMode(DELAY_FLAG_PIN, INPUT_PULLUP);       // Enable internal pull-up resistor
  pinMode(DISTORTION_FLAG_PIN, INPUT_PULLUP);  // Enable internal pull-up Resistor

  // Configure LED pins as outputs
  pinMode(DELAY_LED_PIN, OUTPUT);
  pinMode(DISTORTION_LED_PIN, OUTPUT);

  // Attach interrupts to pins for pressing
  attachInterrupt(digitalPinToInterrupt(DELAY_FLAG_PIN), onDelayButtonPress, FALLING);
  attachInterrupt(digitalPinToInterrupt(DISTORTION_FLAG_PIN), onDistortionButtonPress, FALLING);
  
  DetermineState();
  LED_Update();
 
  Serial.begin(115200);

  adc1_config_width(ADC_WIDTH_BIT_12); // Configure Internal ADC resolution to 12 bits

  // Configure each channel with attenuation for the full 0-3.3V range (11dB = 3.3V)
  adc1_config_channel_atten(BlendPot, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(FeedbackPot, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(CrossoverPot, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(GainPot, ADC_ATTEN_DB_11);

  if (codec.begin() == false) //Begin communication over I2C
  {
    Serial.println("The device did not respond. Please check wiring.");
    while (1); // Freeze
  }
  Serial.println("Device is connected properly.");

  codec_setup();

  // Set up I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
 

}

void loop() {

   DetermineState();  //Update State dependent on the flags. 

  // Handle delay button press
  if (delayButtonPressed && (millis() - delayButtonTime > 50)) {  // Debounce period (50 - 100 ms works well)
    delayButtonPressed = false;    //Reset the flag 

    if (currentState == OnlyDelay) {
      currentState = NoEffect;                // Reset if already active
    } else if (currentState == NoEffect) {
      currentState = OnlyDelay;
    } else if (currentState == OnlyDistortion) {
      currentState = DistortionFirstThenDelay; // Combine with distortion
    } else if (currentState == DelayFirstThenDistortion || currentState == DistortionFirstThenDelay) {

      if (digitalRead(DISTORTION_FLAG_PIN) == HIGH) { // Check whether distortion is disabled:
        currentState = OnlyDelay;
      }
      else if (digitalRead(DELAY_FLAG_PIN) == HIGH) // Check whether delay is disabled:
        currentState = OnlyDistortion;
    }
    else if (digitalRead(DISTORTION_FLAG_PIN && DELAY_FLAG_PIN) == HIGH) {
      currentState = NoEffect;
    }
  }





  // Handle distortion button press
  if (distortionButtonPressed && (millis() - distortionButtonTime > 50)) {  // Debounce period (50 - 100 ms works great)
    distortionButtonPressed = false;

    if (currentState == OnlyDistortion) {
      currentState = NoEffect;                // Reset if already active
    } else if (currentState == NoEffect) {
      currentState = OnlyDistortion;
    } else if (currentState == OnlyDelay) {
      currentState = DelayFirstThenDistortion; // Combine with delay
    } else if (currentState == DelayFirstThenDistortion || currentState == DistortionFirstThenDelay) {

      if (digitalRead(DISTORTION_FLAG_PIN) == HIGH) {  // Check whether distortion is disabled:
        currentState = OnlyDelay;
      }
      else if (digitalRead(DELAY_FLAG_PIN) == HIGH) { // Check whether delay is disabled:
        currentState = OnlyDistortion;
      }
      else if (digitalRead(DISTORTION_FLAG_PIN && DELAY_FLAG_PIN) == HIGH) {
        currentState = NoEffect;
      }
    }
  }

 // Serial.println(currentState);


  switch (currentState) {

    case NoEffect: {
      
        LED_Update();        // Nothing happens here - Codec is bypassed in hardware (TrueBypass)
      }

      break;
    case OnlyDelay: {


        LED_Update();

        size_t bytesIn = 0;
        size_t bytesOut = 0;

        // Read audio data from I2S
        esp_err_t result = i2s_read(I2S_PORT, sBuffer, bufferLen * sizeof(int16_t), &bytesIn, portMAX_DELAY);

        if (result == ESP_OK)
        {
          
          // Temporary buffer for float data
          float outputBufferFloats[bufferLen];
          ConvertToFloats(sBuffer, bufferLen, outputBufferFloats); // Conversion to float (-1 to 1)

          // Read the values from the potentiometer  // |k*G| < 1 to assure stability
          float f = readPotentiometer(CrossoverPot, 5000, 100); // Freq: 5kHz - 100 Hz
          float k = readPotentiometer(BlendPot, 1, 0);          // Blend Coefficient
          float G = readPotentiometer(FeedbackPot, -0.9, 0);    // Feedback Coefficient

          //Update Coefficients for IIR-Filter
          float T = 1.0 / 44100.0;
          float RC = 1.0 / (f * TWO_PI);
          float b = RC / (T + RC);
          float a = T / (T + RC);

          const int m = 10000;                   // Delay length (M) number of samples 
          static float Delayline[m] = {0};       // Delay buffer initialized to zeros
          static float FeedbackBuffer[m] = {0};  // Feedback buffer initialized to zeros
          static int delayIndex = 0;             // Index for circular delay buffer

          // Process each sample in the buffer
          for (int i = 0; i < bufferLen; i++)
          {
            // Fetch delayed sample (x[n-M] or x_h[n-M])
            float delayedSample = Delayline[delayIndex];           // x[n-M]
            float feedbackSample = FeedbackBuffer[delayIndex];     // x_h[n-M]

            // Update feedback x_h[n] (Compute Delay line output)
            float xh = outputBufferFloats[i] + G * feedbackSample;     // Compute x_h  (sum 1: x[n] + (x[n-m]*G))
            FeedbackBuffer[delayIndex] = xh;                           // xh[n-m] Feedback is stored in buffer.

            // Compute the difference equation:
            float y = outputBufferFloats[i] * (1.0 - k) + k * xh   ;  // Compute y[n] (Sum 2: (x[n]*(1-k) + k * x_h[n-M])

            // Update delay buffer (store delayed output sample in the circular buffer)
            Delayline[delayIndex] = outputBufferFloats[i];

            // Increment circular buffer index by 1 and check if it is out of bounds
            delayIndex = (delayIndex + 1) % m;

            // Store output
            outputBufferFloats[i] = y;

            //Computing difference equation: y[n] = a * x[n] + b * y[n-1]  // Fed feature + dæmper højfrekvent noget støj. Støj bliver ellers ført tilbage i systemet og repeated. Giver en mudret lyd..
            IIRfilterOutput[i] = a * outputBufferFloats[i] + b * y_prev;

            //Updating y[n-1]
            y_prev = IIRfilterOutput[i];

          }

          // Convert processed float data back to integer
          int16_t outputBufferInts[bufferLen];
          ConvertToInts(IIRfilterOutput, bufferLen, outputBufferInts);

          // Write audio data to I2S
          esp_err_t result_w = i2s_write(I2S_PORT, outputBufferInts, bytesIn, &bytesOut, portMAX_DELAY);
          if (result_w != ESP_OK) {
            Serial.println("I2S write error.");
          }
        }

        break;
      }

    case OnlyDistortion: {

        LED_Update();

        size_t bytesIn = 0;
        size_t bytesOut = 0;

        // Read audio data from I2S
        esp_err_t result = i2s_read(I2S_PORT, sBuffer, bufferLen * sizeof(int16_t), &bytesIn, portMAX_DELAY);

        if (result == ESP_OK)
        {

          // Temporary buffer for float data
          float outputBufferFloats[bufferLen];
          ConvertToFloats(sBuffer, bufferLen, outputBufferFloats); // Conversion to float (-1 to 1)


          // Read the blend value from the potentiometer
          float A = readPotentiometer(GainPot, 4, 0);              //Gain: 12 - 0 dB
          float f = readPotentiometer(CrossoverPot, 5000, 100);    //Freq: 5kHz - 100 Hz

          float T = 1.0 / 44100.0;
          float RC = 1.0 / (f * TWO_PI);
          float b = RC / (T + RC);
          float a = T / (T + RC);

          // Process each sample in the buffer
          for (int i = 0; i < bufferLen; i++)
          {

            //=================FIR-Filter======================

            // Add the current sample to the circular buffer
            CircularBuffer[CircIndex] = outputBufferFloats[i];

            // Compute FIR filter output
            float filterOutput = 0;
            for (int j = 0; j < FirTaps; j++) {
              int bufferIdx = (CircIndex - j + FirTaps) % FirTaps; // Circular buffer access with wrap-around
              filterOutput += FilterCoefficients[j] * CircularBuffer[bufferIdx];
            }

            // Update the circular buffer index
            CircIndex = (CircIndex + 1) % FirTaps;

            //==================Distortion======================

            // Scale the filtered audio
            float ScaledAudio = filterOutput * A;

            // Apply distortion
            DistortedOutput[i] = (1.0 - exp(-fabs(ScaledAudio)));

            //====================IIR-Filter====================

            //Computing difference equation: y[n] = a * x[n] + b * y[n-1]
            IIRfilterOutput[i] = a * DistortedOutput[i] + b * y_prev;

            //Updating y[n-1]
            y_prev = IIRfilterOutput[i];

          }

          // Convert distorted float data back to integer
          int16_t outputBufferInts[bufferLen];
          ConvertToInts(IIRfilterOutput, bufferLen, outputBufferInts);

          // Write audio data to I2S
          esp_err_t result_w = i2s_write(I2S_PORT, outputBufferInts, bytesIn, &bytesOut, portMAX_DELAY);
          if (result_w != ESP_OK) {
            Serial.println("I2S write error.");
          }
        }


        break;
      }
    case DelayFirstThenDistortion:


      LED_Update();

      //This effect Sounds Terrible...Therefore reduced to an LED blink-sequence. 
      //An alternative effect can be incoporated, if such, update "DetermineState" function og "buttonPress" functions
      //To achieve the correct state, thus entering this switchcase. 
      
      digitalWrite(DISTORTION_LED_PIN, HIGH);

      digitalWrite(DELAY_LED_PIN, HIGH);
      delay(100);
      digitalWrite(DELAY_LED_PIN, LOW);
      delay(100);
      digitalWrite(DELAY_LED_PIN, HIGH);
      delay(100);
      digitalWrite(DELAY_LED_PIN, LOW);
      delay(100);

      break;
    //===================================== Distortion -> Delay =======================================
    case DistortionFirstThenDelay: {

        LED_Update();

        size_t bytesIn = 0;
        size_t bytesOut = 0;

        // Read audio data from I2S
        esp_err_t result = i2s_read(I2S_PORT, sBuffer, bufferLen * sizeof(int16_t), &bytesIn, portMAX_DELAY);

        if (result == ESP_OK)
        {

          // Temporary buffer for float data
          float outputBufferFloats[bufferLen];
          ConvertToFloats(sBuffer, bufferLen, outputBufferFloats); // Conversion to float (-1 to 1)


          // Read the values from the potentiometer  // |k*G| < 1 to assure stability
          float A = readPotentiometer(GainPot, 4, 0);           // Gain: 12 - 0 dB
          float f = readPotentiometer(CrossoverPot, 5000, 100); // Freq: 5kHz - 100 Hz
          float k = readPotentiometer(BlendPot, 1, 0);          // Blend Coefficient
          float G = readPotentiometer(FeedbackPot, -0.9, 0);    // Feedback Coefficient

          float T = 1.0 / 44100.0;
          float RC = 1.0 / (f * TWO_PI);
          float b = RC / (T + RC);
          float a = T / (T + RC);

          const int m = 10000;                   // Delay length (M) number of samples u know /10k virker sygt.
          static float Delayline[m] = {0};       // Delay buffer initialized to zeros
          static float FeedbackBuffer[m] = {0};  // Delay buffer initialized to zeros
          static int delayIndex = 0;             // Index for circular delay buffer

          // Process each sample in the buffer
          for (int i = 0; i < bufferLen; i++)
          {

            //=================FIR-Filter======================

            // Add the current sample to the circular buffer
            CircularBuffer[CircIndex] = outputBufferFloats[i];

            // Compute FIR filter output
            float filterOutput = 0;
            for (int j = 0; j < FirTaps; j++) {
              int bufferIdx = (CircIndex - j + FirTaps) % FirTaps; // Circular buffer access with wrap-around
              filterOutput += FilterCoefficients[j] * CircularBuffer[bufferIdx];
            }

            // Update the circular buffer index
            CircIndex = (CircIndex + 1) % FirTaps;

            //==================Distortion======================

            // Scale the filtered audio
            float ScaledAudio = filterOutput * A;

            // Apply distortion
            DistortedOutput[i] = (1.0 - exp(-fabs(ScaledAudio)));

            //====================Delay====================

            // Fetch delayed sample (x[n-M] or x_h[n-M])
            float delayedSample = Delayline[delayIndex];           // x[n-M]
            float feedbackSample = FeedbackBuffer[delayIndex];     // x_h[n-M]

            // Update feedback x_h[n] (Compute Delay line output)
            float xh = DistortedOutput[i] + G * feedbackSample;    // Compute x_h  (sum 1: x[n] + (x[n-m]*G))
            FeedbackBuffer[delayIndex] = xh;                       // xh[n-m] Feedback is stored in buffer.

            // Compute the difference equation: 
            float y = DistortedOutput[i] * (1.0 - k) + k * xh   ;  // Compute y[n] (Sum 2: (x[n]*(1-k) + k * x_h[n-M])

            // Update delay buffer (store delayed output sample in the circular buffer)
            Delayline[delayIndex] = DistortedOutput[i];

            // Increment circular buffer index by 1 and check if it is out of bounds
            delayIndex = (delayIndex + 1) % m;

            // Store output in buffer
            outputBufferFloats[i] = y;

            //====================IIR_Filter====================

            //Computing difference equation: y[n] = a * x[n] + b * y[n-1]  // Fed feature + eliminere noget støj. Støj bliver ført tilbage i systemet og repeated. Giver en mudret lyd..
            IIRfilterOutput[i] = a * outputBufferFloats[i] + b * y_prev;

            //Updating y[n-1]
            y_prev = IIRfilterOutput[i];

          }

          // Convert distorted float data back to integer
          int16_t outputBufferInts[bufferLen];
          ConvertToInts(IIRfilterOutput, bufferLen, outputBufferInts);

          // Write audio data to I2S
          esp_err_t result_w = i2s_write(I2S_PORT, outputBufferInts, bytesIn, &bytesOut, portMAX_DELAY);
          if (result_w != ESP_OK) {
            Serial.println("I2S write error.");
          }
        }

        break;
      }

      delay(100); // Stability / Evil delay  
  }
}
//=================== Codec Configuration - Mono ================================

void codec_setup()  
{
  // General setup needed Updated only Right
  codec.enableVREF();
  codec.enableVMID();

  // Setup signal flow for RIGHT input
  codec.enableRMIC();               // Enable Right MIC
  codec.connectRMN1();              // Connect INPUT1 to RMN (inverting input of PGA)
  codec.disableRINMUTE();           // Disable mute for Right PGA input
  codec.setRINVOLDB(0.00);          // Set Right PGA volume to 0 dB
  codec.setRMICBOOST(WM8960_MIC_BOOST_GAIN_0DB); // Set Right MIC boost gain

  codec.connectRMIC2B();            // Route Right MIC (PGA output) to boost mixer
  codec.enableAINR();               // Enable Right Analog Input

  // Disable Left channel components
  codec.disableLMIC();              // Disable Left MIC
  codec.disableLINMUTE();           // Mute Left PGA input
  codec.setLINVOLDB(-17.25);        // Set Left PGA volume to minimum
  codec.disableAINL();              // Disable Left Analog Input

  // Disable analog bypass paths
  codec.disableLB2LO();             // Disable Left Boost Mixer to Output Mixer
  codec.disableRB2RO();             // Disable Right Boost Mixer to Output Mixer

  // Connect Right DAC output to Right Output Mixer
  codec.enableRD2RO();
  codec.setRB2ROVOL(WM8960_OUTPUT_MIXER_GAIN_NEG_21DB); // Set Right output mixer gain
  codec.enableROMIX();             // Enable Right Output Mixer

  // Clock setup for 44.1kHz
  codec.enablePLL();
  codec.setPLLPRESCALE(WM8960_PLLPRESCALE_DIV_2);
  codec.setSMD(WM8960_PLL_MODE_FRACTIONAL);
  codec.setCLKSEL(WM8960_CLKSEL_PLL);
  codec.setSYSCLKDIV(WM8960_SYSCLK_DIV_BY_2);
  codec.setBCLKDIV(4);
  codec.setDCLKDIV(WM8960_DCLKDIV_16);
  codec.setPLLN(7);
  codec.setPLLK(0x86, 0xC2, 0x26); // PLLK=86C226h
  codec.setWL(WM8960_WL_16BIT);

  codec.enablePeripheralMode();

  // Enable Right ADC and DAC
  codec.enableAdcRight();           // Enable Right ADC
  codec.disableAdcLeft();           // Disable Left ADC
  codec.enableDacRight();           // Enable Right DAC
  codec.disableDacLeft();           // Disable Left DAC
  codec.disableDacMute();

  codec.enableHeadphones();
  //codec.enableOUT3MIX();            // 1,65V Offset reference. 

  Serial.println("Volume set to +0dB");
  codec.setHeadphoneVolumeDB(0.00);

  Serial.println("Codec Setup complete. Mono operation on Right INPUT1 and Right OUTPUT.");
}


//===================I2S Mono Configuration!================================
void i2s_install() { 
  const i2s_driver_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
    .sample_rate = 44100,  // Ensure codec is configured for this rate
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // Mono, Right channel only!
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S_MSB), // Ensure codec matches this
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // Use safe interrupt level
    .dma_buf_count = 8, // Number of DMA buffers
    .dma_buf_len = bufferLen, // 64 - Latency dependent. 
    .use_apll = false,  // APLL is optional but increases clock accuracy
    .tx_desc_auto_clear = true, // Automatically clear TX descriptor on underflow
    .fixed_mclk = 0,  // Let ESP32 calculate MCLK
    .mclk_multiple = i2s_mclk_multiple_t(I2S_MCLK_MULTIPLE_512), // 512 * 44.1kHz = 22.5MHz 
    .bits_per_chan = I2S_BITS_PER_CHAN_16BIT // Match codec settings of 16 bitszz
  };

  esp_err_t ret = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (ret != ESP_OK) {
    Serial.printf("I2S Driver Install Failed: %d\n", ret);
    while (1);  // Halt execution to debug
  }
}

void i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE, // Set to GPIO for MCLK /Optional
    .bck_io_num = I2S_SCK,          // Bit Clock
    .ws_io_num = I2S_WS,            // Word Select
    .data_out_num = I2S_SDO,        // Data Out
    .data_in_num = I2S_SD           // Data In
  };

  esp_err_t ret = i2s_set_pin(I2S_PORT, &pin_config);
  if (ret != ESP_OK) {
    Serial.printf("I2S Pin Config Failed: %d\n", ret);
    while (1);  // Halt execution to debug
  }
}



//===================Conversion to Floats==================================
void ConvertToFloats(int16_t *AudioBuffer, int bufferLength, float * outputBuffer) {
  const int16_t maxValue = 32767;
  const int16_t minValue = -32768;

  const float maxOutputValue = 1.0;
  const float minOutputValue = -1.0;

  for (int i = 0; i < bufferLength; i++) {
    // Convert the int value to a float in the range [-1, 1]
    outputBuffer[i] = minOutputValue + ( float (AudioBuffer[i] - minValue ) / float ( maxValue - minValue )) * ( maxOutputValue - minOutputValue );
  }
}

//===================Conversion to integers==============================
void ConvertToInts(float *AudioBuffer, int bufferLength, int16_t *outputBuffer) {
  const int16_t maxValue = 32767;
  const int16_t minValue = -32768;

  const float maxOutputValue = 1.0;
  const float minOutputValue = -1.0;

  for (int i = 0; i < bufferLength; i++) {
    // Convert the float value to an integer in the range [-32768, 32767]
    outputBuffer[i] = minValue + (int16_t)((AudioBuffer[i] - minOutputValue) / (maxOutputValue - minOutputValue) * (maxValue - minValue));
  }
}

//===================Map ints to floats==================================
float floatMap ( float x, float in_min , float in_max , float out_min , float out_max ) {
  return (x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min ;
}


//=================== Update Gain + Crossover ================================
float readPotentiometer(adc1_channel_t potChannel, float minOutput, float maxOutput) {

  // Read raw value from the specified ADC channel
  int raw = adc1_get_raw(potChannel);

  // Map raw value to the desired range
  return floatMap(raw, 0, 4095, minOutput, maxOutput);
}
