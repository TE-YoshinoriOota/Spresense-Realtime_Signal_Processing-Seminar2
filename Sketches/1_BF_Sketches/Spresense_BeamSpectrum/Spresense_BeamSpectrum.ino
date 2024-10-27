#include <Audio.h>
#include <USBSerial.h>

/* Use CMSIS library */
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <arm_math.h>

/* need set 896MB for the memory allocation for the Main Core */
AudioClass *theAudio;
arm_rfft_fast_instance_f32 S;

#define SERIAL_BAUDRATE 115200
USBSerial UsbSerial;

const int sample_num = 1024;
const int mic_channel_num = AS_CHANNEL_4CH;
const float L = 0.1; // m
const float sonic_speed = 340.29; // m
const float frequency = 1602.0; // Hz
const float start_angle = 0; // degree
const float end_angle = 360; // degree
const float step_angle = 10; // degree
const int angle_points = int((end_angle-start_angle) / step_angle) + 1;
const float freq_resolution = float(AS_SAMPLINGRATE_48000)/float(sample_num);
const int target_bin = round(frequency/freq_resolution);
float stearing_real[mic_channel_num][angle_points];
float stearing_imag[mic_channel_num][angle_points];


void setup() {

  Serial.begin(115200);
  UsbSerial.begin(SERIAL_BAUDRATE);

  while (!Serial);

  Serial.println("Init Audio Library");
  theAudio = AudioClass::getInstance();
  theAudio->begin();

  Serial.println("Init Audio Recorder");
  /* Select input device as AMIC */
  //theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC, 210);
  theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC);

  /* Set PCM capture */
  theAudio->initRecorder(AS_CODECTYPE_PCM, "/mnt/sd0/BIN", AS_SAMPLINGRATE_48000, mic_channel_num);

  arm_rfft_1024_fast_init_f32(&S);
  //attachInterrupt(digitalPinToInterrupt(button_pin), rec_start, FALLING); 


  // making the stearing vector
  for (int ch = 0; ch < mic_channel_num; ++ch) {
    memset(&stearing_real[ch][0], 0, angle_points*sizeof(float));
    memset(&stearing_imag[ch][0], 0, angle_points*sizeof(float));
  }

  // build stearing vectors
  // static const float constant = frequency*L/sonic_speed;
  for (int mic = 0; mic < mic_channel_num; ++mic) {
    float angle = start_angle;
    for (int n = 0; n < angle_points; ++n, angle += step_angle) {
      float angle_rad = (float)angle*M_PI/180;
      float delay = 2.0*M_PI*(((mic)-(mic_channel_num-1)/2)*L*frequency/sonic_speed)*arm_sin_f32(angle_rad);
      // exp (j*delay) = cons (delay) + jsin(delay)
      stearing_real[mic][n] = arm_cos_f32(delay) / mic_channel_num;
      stearing_imag[mic][n] = arm_sin_f32(delay) / mic_channel_num;
    }
  }  
}

void loop() {

  /* Select mic channel number */
  static const int frames = 2;
  static const int channel_samples = sample_num*frames;
  static int16_t channel[mic_channel_num][channel_samples];

  // Sound buffer
  const int32_t buffer_sample = sample_num * mic_channel_num;
  const int32_t buffer_size = buffer_sample * sizeof(int16_t);
  static char  buffer[buffer_size];
  uint32_t read_size;

  // Sound Data Collection
  Serial.println("Rec start!");
  theAudio->startRecorder();

  int n = 0;
  while (n < channel_samples) {
    /* Read frames to record in buffer */
    int err = theAudio->readFrames(&buffer[0], buffer_size, &read_size);
    if (err != AUDIOLIB_ECODE_OK && err != AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA) {
      printf("Error err = %d\n", err);
      sleep(1);
      theAudio->stopRecorder();
      exit(1);
    }
    // Serial.println("read_size/buffer_size: " + String(read_size) + "/" + String(buffer_size));

    if ((read_size != 0) && (read_size == buffer_size)) {
      int16_t *buffer16 = (int16_t*)buffer;
      for (int i = 0; i < buffer_sample; i += mic_channel_num) {
        for (int j = 0; j < mic_channel_num; ++j) {
          channel[j][n] = buffer16[i+j];
        }
        ++n;
      }
    } else {
      usleep(20000);
    }
  }

  // stop recorder
  Serial.println("Rec Stop!");
  theAudio->stopRecorder();


  // Do Beamforming
  Serial.println("Start Beam!");
  uint32_t start_beam_time = millis();

  static float ch_beam[mic_channel_num][sample_num];
  static float result_beam[sample_num];

  for (int ch = 0; ch < mic_channel_num; ++ch) {
    memset(&ch_beam[ch][0], 0, sample_num*sizeof(float));
  }

  // calc the sound power of each angle_point
  UsbSerial.println("SPRS");
  Serial.println("SPRS");
  delay(10);
  float angle = start_angle;
  for (int n = 0; n < angle_points; ++n, angle += step_angle) {

    static float pSrc[sample_num];
    static float pDst[sample_num];

    for (int ch = 0; ch < mic_channel_num; ++ch) {

      // memclear
      memset(&pSrc[0], 0, sample_num*sizeof(float));
      memset(&pDst[0], 0, sample_num*sizeof(float));

      // do the fft
      arm_q15_to_float(&channel[ch][0], &pSrc[0], sample_num);
      arm_rfft_fast_f32(&S, &pSrc[0], &pDst[0], 0);

      // a+ib
      float a = stearing_real[ch][n]; // delay real
      float b = stearing_imag[ch][n]; // delay imag

      // multiply the stearing vector
      // (a+ib) x (c+id) = (ac–bd) + i(ad+bc)
      for (int i = 0; i < sample_num; i+=2) {
        float c = pDst[i+0]; // sound real
        float d = pDst[i+1]; // sound imag
        ch_beam[ch][i+0] = (a*c)-(b*d); // real
        ch_beam[ch][i+1] = (a*d)+(b*c); // imag
      }
    }

    // sum up of all mics
    memset(&result_beam[0], 0, sample_num*sizeof(float));
    for (int i = 0; i < sample_num; i+=2) {
      for (int ch = 0; ch < mic_channel_num; ++ch) {
        result_beam[i+0] += ch_beam[ch][i+0];  // real value
        result_beam[i+1] += ch_beam[ch][i+1];  // imag value
      }
    }

    float value;
    float peakFs = get_peak_frequency(&result_beam[0], sample_num, &value);

    //float output_real = result_beam[target_bin*2 + 0];
    //float output_imag = result_beam[target_bin*2 + 1];
    //float value = 10*log10(output_real*output_real + output_imag*output_imag);

    //printf("angle %3.1f, peak %f kHz, value %f\n", angle, peakFs/1000, value);
    //printf("%3.1f, %f, %f\n", angle, value, peakFs);
    printf("%03.1f, %f\n", angle, value);
    //printf("1.2, %f, -0.5\n", value);
    UsbSerial.printf("%f, %f\n", angle, value);

  }

  uint32_t duration = millis() - start_beam_time;
  Serial.println("duration: " + String(duration));
  Serial.println("End Beam!");
}


float get_peak_frequency(float *pData, int fftLen, float *value) {
  float g_fs = 48000.0f;
  uint32_t index;
  float maxValue;
  float delta;
  float peakFs;

  float *pTmp = (float*)malloc(fftLen*sizeof(float));
  memset(pTmp, 0, fftLen*sizeof(float));

  arm_cmplx_mag_f32(&pData[0], &pTmp[1], fftLen/2-1);
  pTmp[0] = pData[0];
  pTmp[fftLen/2] = pData[1];

  arm_max_f32(pTmp, fftLen/2, &maxValue, &index);

  delta = 0.5 * (pTmp[index-1] - pTmp[index+1])
    / (pTmp[index-1] + pTmp[index+1] - (2.0f*pTmp[index]));
  peakFs = (index + delta) * g_fs / (fftLen-1);

  memset(pTmp, 0, fftLen*sizeof(float));  
  free(pTmp);

  *value = maxValue;
  return peakFs;
}
