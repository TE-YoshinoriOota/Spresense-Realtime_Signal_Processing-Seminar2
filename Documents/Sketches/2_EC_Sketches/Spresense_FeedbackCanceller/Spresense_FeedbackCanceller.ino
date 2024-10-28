#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>

#define SAMPLE_SIZE (1024)

//*****
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <cmsis/arm_math.h>


#define TAPS (7)
float h[TAPS];

#define DELAY_FRAME (60)
static float x_in[SAMPLE_SIZE];
static float d_out[SAMPLE_SIZE];
static float e_out[SAMPLE_SIZE];
static float e_buf[SAMPLE_SIZE*DELAY_FRAME];
//*****


//****
#define SW_PIN (0)
bool onoff = false;
void toggle_sw() {
  onoff = onoff ? false : true;
  digitalWrite(LED0, onoff);
}

//****

FrontEnd *theFrontEnd;
OutputMixer *theMixer;

const int32_t channel_num = AS_CHANNEL_MONO;
const int32_t bit_length  = AS_BITLENGTH_16;
const int32_t sample_size = SAMPLE_SIZE;
const int32_t in_frame_size  = sample_size * (bit_length / 8) * channel_num;
const int32_t out_frame_size = sample_size * (bit_length / 8) * AS_CHANNEL_STEREO;
bool isErr = false;
static uint8_t sound_input[in_frame_size];
static uint8_t stereo_output[out_frame_size];  

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


void signal_process(int16_t* sound_input, int16_t* stereo_output, uint32_t sample_size) {
//***/
  if (onoff) {
    float* u_in = &e_buf[SAMPLE_SIZE*(DELAY_FRAME-1)];
    arm_q15_to_float((q15_t*)&sound_input[0], &x_in[0], SAMPLE_SIZE);   

    // apply the adaptive FIR filter
    memset(&d_out[0], 0, sizeof(float)*SAMPLE_SIZE);
    for (int n = SAMPLE_SIZE-1; n >= 0; --n) {
      for (int i = 0; i < TAPS; ++i) {  
        d_out[n] += h[i]*u_in[n+i];  // predictive signal
      }
      e_out[n] = x_in[n] - d_out[n]; // error

      // update coeff of adaptive filter
      float mu = 1.0; // tentative value
      for (int i = 0; i < TAPS; ++i) {   
        h[i] = h[i] + mu*u_in[n+i]*e_out[n];
      }
    }    

    // convert float to int16_t and copy data    
    arm_float_to_q15(&u_in[0], (q15_t*)&sound_input[0], SAMPLE_SIZE);

    // buff shift and copy 
    memcpy(&e_buf[SAMPLE_SIZE], &e_buf[0], sizeof(float)*SAMPLE_SIZE*(DELAY_FRAME-1));
    memcpy(&e_buf[0], &e_out[0], sizeof(float)*SAMPLE_SIZE);
  } else {
    initializeFirAsLPF();    
  }
//***/

  /* copy the signal to output buffer */
  for (int n = SAMPLE_SIZE-1; n >= 0; --n)  {
    stereo_output[n*2] = stereo_output[n*2+1] = sound_input[n];
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


//***/
void initializeFirAsLPF() {
  const uint32_t CUTTOFF_FREQ_HZ = 2000; 
  float Fc = (float)CUTTOFF_FREQ_HZ/AS_SAMPLINGRATE_48000;
  const int H_TAPS = TAPS/2;
  int n = 0;
  for (int k = H_TAPS; k >= -H_TAPS; --k) {
    if (k == 0) h[n] = 2.*Fc;
    else {
      h[n] = 2.*Fc*arm_sin_f32(2.*PI*Fc*k)/(2.*PI*Fc*k);
    }
    ++n;
  }

  for (int m = 0; m < TAPS; ++m) {
    h[m] = (0.5 - 0.5*arm_cos_f32(2*PI*m/TAPS))*h[m];
  }
  
  //arm_fir_init_f32(&S, TAPS, &pCoeffs[0], &pState[0], SAMPLE_SIZE);
}
//***/

void setup() {
  Serial.begin(115200);

  //*** echo cancel sw
  attachInterrupt(SW_PIN, toggle_sw, FALLING);

  //*** initialize adaptive filter coeff
  initializeFirAsLPF();
  //***

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