#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>

#define SAMPLE_SIZE (256)

//***/
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <cmsis/arm_math.h>
arm_rfft_fast_instance_f32 S;
//***/

FrontEnd *theFrontEnd;
OutputMixer *theMixer;

const int32_t channel_num = AS_CHANNEL_4CH;
const int32_t bit_length  = AS_BITLENGTH_16;
const int32_t sample_size = SAMPLE_SIZE;
const int32_t in_frame_size  = sample_size * (bit_length / 8) * channel_num;
const int32_t out_frame_size = sample_size * (bit_length / 8) * AS_CHANNEL_STEREO;
bool isErr = false;

#define BEAM_SW (0)
bool bBeamform = false;
void onoff_beam() {
  bBeamform = bBeamform ? false : true;
  digitalWrite(LED0, bBeamform);
}

// stearing vector parameters
const float L = 0.1; // m
const float sonic_speed = 340.39; // m
const float frequency = 1602.0; // Hz
const float target_angle_degree = 30.; // degree
const float freq_resolution = float(AS_SAMPLINGRATE_48000)/float(SAMPLE_SIZE);
const int target_bin = (int)round(frequency/freq_resolution);
float stearing_real[channel_num];
float stearing_imag[channel_num];

void frontend_attention_cb(const ErrorAttentionParam *param) {
  Serial.println("ERROR: Attention! Something happened at FrontEnd");
  if (param->error_code >= AS_ATTENTION_CODE_WARNING) isErr = true;
}

void mixer_attention_cb(const ErrorAttentionParam *param){
  Serial.println("ERROR: Attention! Something happened at Mixer");
  if (param->error_code >= AS_ATTENTION_CODE_WARNING) isErr = true;
}

static bool frontend_done_cb(AsMicFrontendEvent ev, uint32_t result, uint32_t detail){
  UNUSED(ev);  UNUSED(result);  UNUSED(detail);  
  return true;
}

static void outputmixer_done_cb(MsgQueId requester_dtq, MsgType reply_of, AsOutputMixDoneParam* done_param) {
  UNUSED(requester_dtq);  UNUSED(reply_of);  UNUSED(done_param);
  return;
}

static void outputmixer0_send_cb(int32_t identifier, bool is_end) {
  UNUSED(identifier);  UNUSED(is_end);
  return;
}

static void frontend_pcm_cb(AsPcmDataParam pcm) {
  static uint8_t sound_input[in_frame_size];
  static uint8_t stereo_output[out_frame_size];  
  static const bool time_measurement = false;
  if (time_measurement) {
    static uint32_t last_time = 0;
    uint32_t current_time = micros();
    uint32_t duration = current_time - last_time;
    last_time = current_time;
    Serial.println("duration = " + String(duration));
  }

  frontend_signal_input(pcm, sound_input, in_frame_size);
  signal_process((int16_t*)sound_input, (int16_t*)stereo_output, sample_size);
  mixer_stereo_output(stereo_output, out_frame_size);
  return;
}

void frontend_signal_input(AsPcmDataParam pcm, uint8_t* input, const uint32_t in_frame_size) {
  /* clean up the input buffer */
  memset(input, 0, in_frame_size);

  if (!pcm.is_valid) {
    Serial.println("WARNING: Invalid data! @frontend_signal_input");
    return;
  }
   
  if (pcm.size > in_frame_size) {
    Serial.print("WARNING: Captured size is too big! -");
    Serial.print(String(pcm.size));
    Serial.println("- @frontend_signal_input");
    pcm.size = in_frame_size;
  } 
  
  /* copy the signal to signal_input buffer */
  if (pcm.size != 0) {
    memcpy(input, pcm.mh.getPa(), pcm.size);
  } else {
    Serial.println("WARNING: Captured size is zero! @frontend_signal_input");
  }  
}

void signal_process(int16_t* sound_input, int16_t* stereo_output, const uint32_t sample_size) {
  // uint32_t start_time = micros();
  static float pTmp[SAMPLE_SIZE] = {0};
  static float pOut[SAMPLE_SIZE] = {0};
  static float pBuf[channel_num][SAMPLE_SIZE] = {0};
  static int16_t mic[channel_num][SAMPLE_SIZE] = {0};
  static int16_t result[SAMPLE_SIZE] = {0};

  // split mixed sound data to each channel
  for (int n = 0; n < sample_size; ++n) {
    for (int ch = 0; ch < channel_num; ++ch) {
      mic[ch][n] = sound_input[n*channel_num + ch];
    }
  }

  if (bBeamform) {
    // do fft to each channel
    for (int ch = 0; ch < channel_num; ++ch) {
      q15_t* q15_sound = (q15_t*)&mic[ch][0];
      arm_q15_to_float(&q15_sound[0], &pTmp[0], sample_size);
      arm_rfft_fast_f32(&S, &pTmp[0], &pBuf[ch][0], 0); 

      // do beam forming
      // multiply the stearing vector
      // (a+ib) x (c+id) = (ac–bd) + i(ad+bc)    
      float a = stearing_real[ch];
      float b = stearing_imag[ch];  
      memset(&pOut[0], 0, sample_size*sizeof(float));
      for (int n = 0; n < sample_size; n += 2) {
        float c = pBuf[ch][n+0];
        float d = pBuf[ch][n+1];
        pOut[n+0] += ((a*c)-(b*d))/channel_num;
        pOut[n+1] += ((a*d)+(b*c))/channel_num;
      }
    }

    // output spectrum power of the target frequency
    float output_real = pOut[target_bin*2+0];
    float output_imag = pOut[target_bin*2+1];
    float output = 10*log10(output_real*output_real+output_imag*output_imag);
    Serial.printf("-50, %f, -100\n", output);

    // do reverse fft for the beam forming result
    arm_rfft_fast_f32(&S, &pOut[0], &pTmp[0], 1); 
    arm_float_to_q15(&pTmp[0], (q15_t*)&result[0], sample_size);
    /* copy the signal to output buffer */
    for (int n = SAMPLE_SIZE-1; n >= 0; --n)  {
      stereo_output[n*2] = stereo_output[n*2+1] = result[n];
    }
  } else {
    /* copy the ch1 signal to output buffer */
    for (int n = SAMPLE_SIZE-1; n >= 0; --n)  {
      stereo_output[n*2] = stereo_output[n*2+1] = mic[1][n] ;
    }
  }


  return;
}

void mixer_stereo_output(uint8_t* stereo_output, uint32_t out_frame_size) {
  
  /* Alloc MemHandle */
  AsPcmDataParam pcm_param;
  if (pcm_param.mh.allocSeg(S0_REND_PCM_BUF_POOL, out_frame_size) != ERR_OK) {
    Serial.println("ERROR: Cannot allocate memory @mixer_stereo_output");
    isErr = false;
    return;
  }
  
  /* Set PCM parameters */
  pcm_param.is_end = false;
  pcm_param.identifier = OutputMixer0;
  pcm_param.callback = 0;
  pcm_param.bit_length = bit_length;
  pcm_param.size = out_frame_size;
  pcm_param.sample = sample_size*AS_CHANNEL_STEREO;
  pcm_param.is_valid = true;
  memcpy(pcm_param.mh.getPa(), stereo_output, pcm_param.size);
 
  int err = theMixer->sendData(OutputMixer0, outputmixer0_send_cb, pcm_param);
  if (err != OUTPUTMIXER_ECODE_OK) {
    Serial.println("ERROR: sendData -" + String(err) + "- @mixer_stereo_output");
    isErr = true;
  }
}

void setup() {
  attachInterrupt(BEAM_SW, onoff_beam, FALLING);
  Serial.begin(115200);
  //***/
  arm_rfft_fast_init_f32(&S, SAMPLE_SIZE);
  //***/

  /* build stearing vector */
  for (int mic = 0; mic < channel_num; ++mic) {
    const float angle_radian = target_angle_degree*M_PI/180.;
    float delay = 2.0*M_PI*(((mic)-(channel_num-1)/2)*L*frequency/sonic_speed)*arm_sin_f32(angle_radian);
    // exp (j*delay) = cons (delay) + j*sin(delay)
    stearing_real[mic] = arm_cos_f32(delay) / channel_num;
    stearing_imag[mic] = arm_sin_f32(delay) / channel_num;
  }  

  /* Initialize memory pools and message libs */
  initMemoryPools();
  createStaticPools(MEM_LAYOUT_RECORDINGPLAYER);

  /* setup FrontEnd and Mixer */
  theFrontEnd = FrontEnd::getInstance();
  theMixer = OutputMixer::getInstance();
  
  /* set clock mode */
  theFrontEnd->setCapturingClkMode(FRONTEND_CAPCLK_NORMAL);

  /* begin FrontEnd and OuputMixer */
  theFrontEnd->begin(frontend_attention_cb);
  theMixer->begin();
  Serial.println("Setup: FrontEnd and OutputMixer began");

  /* activate FrontEnd and Mixer */
  theFrontEnd->setMicGain(0);
  theFrontEnd->activate(frontend_done_cb);
  theMixer->create(mixer_attention_cb);
  theMixer->activate(OutputMixer0, outputmixer_done_cb);
  delay(100); /* waiting for Mic startup */
  Serial.println("Setup: FrontEnd and OutputMixer activated");

  /* Initialize FrontEnd */
  AsDataDest dst;
  dst.cb = frontend_pcm_cb;
  theFrontEnd->init(channel_num, bit_length, sample_size, AsDataPathCallback, dst);
  Serial.println("Setup: FrontEnd initialized");

  /* Set rendering volume */
  theMixer->setVolume(-10, -10, -10); /* -10dB */

  /* Unmute */
  board_external_amp_mute_control(false);
  theFrontEnd->start();
  Serial.println("Setup: FrontEnd started");
}

void loop() {
  if (isErr == true) {
    board_external_amp_mute_control(true); 
    theFrontEnd->stop();
    theFrontEnd->deactivate();
    theMixer->deactivate(OutputMixer0);
    theFrontEnd->end();
    theMixer->end();
    Serial.println("Capturing Process Terminated");
    while(1) {};
  }
}