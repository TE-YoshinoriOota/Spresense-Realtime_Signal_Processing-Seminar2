#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>

#define SAMPLE_SIZE (720)
#define TAPS (128)

//*****
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <cmsis/arm_math.h>

arm_fir_instance_f32 S;

static float pState[TAPS+SAMPLE_SIZE-1];
static float pCoeffs[TAPS] = {
   0.00941809,
  -0.01820304,
  -0.00937239,
   0.07003822,
  -0.02587963,
  -0.24966781,
  -0.23483267,
   0.19086214,
   0.72866059,
   1.00000000,
   0.84812888,
   0.42813546,
  -0.01028467,
  -0.29994516,
  -0.39136689,
  -0.35778258,
  -0.30026663,
  -0.28428818,
  -0.30448014,
  -0.30288178,
  -0.20961066,
  -0.06641856,
  -0.00482972,
  -0.06219846,
  -0.14694237,
  -0.18341893,
  -0.16973369,
  -0.13315783,
  -0.10315645,
  -0.08987415,
  -0.07489326,
  -0.03835165,
  -0.00682121,
  -0.01397112,
  -0.04172339,
  -0.04550848,
  -0.01441268,
   0.02347668,
   0.04452798,
   0.04384387,
   0.02059010,
  -0.02718332,
  -0.08746378,
  -0.13131127,
  -0.13829812,
  -0.11140609,
  -0.06600009,
  -0.02757181,
  -0.00535777,
   0.01440755,
   0.03761197,
   0.04992110,
   0.03856665,
   0.01824634,
   0.01639263,
   0.03907216,
   0.06467685,
   0.07322822,
   0.05791743,
   0.03321464,
   0.02361938,
   0.03231811,
   0.04221038,
   0.03738137,
   0.02517788,
   0.01866259,
   0.01935130,
   0.02364572,
   0.03402454,
   0.05129262,
   0.06712287,
   0.07459261,
   0.06977833,
   0.04981964,
   0.02424606,
   0.01522986,
   0.02390664,
   0.02511835,
   0.00262841,
  -0.02526038,
  -0.03028718,
  -0.01165092,
   0.01076034,
   0.02052405,
   0.02157670,
   0.03001041,
   0.05315924,
   0.07768696,
   0.07995006,
   0.05104236,
   0.01090015,
  -0.00705692,
   0.00996782,
   0.04201805,
   0.05839855,
   0.05048825,
   0.03962553,
   0.04471651,
   0.05365735,
   0.04235602,
   0.01611199,
   0.00437153,
   0.01821772,
   0.03639761,
   0.03477717,
   0.01876320,
   0.01167983,
   0.02173987,
   0.03813527,
   0.04968676,
   0.05289084,
   0.04932373,
   0.04281830,
   0.03477875,
   0.02506554,
   0.01278345,
  -0.00171378,
  -0.01131627,
  -0.00863681,
   0.00337151,
   0.01018698,
   0.00542573,
   0.00142058,
   0.00600110,
   0.01563541,
   0.02347080,
   0.02308142,
   0.01042748
};


#define H_TAPS (21)
float h[H_TAPS];

#define DELAY_FRAME (60)
int16_t ref_mic[SAMPLE_SIZE];
int16_t err_mic[SAMPLE_SIZE];
int16_t spk_out[SAMPLE_SIZE];
float x_in[SAMPLE_SIZE*2];  // reference mic
float e_in[SAMPLE_SIZE*2];    // error mic
float r_out[SAMPLE_SIZE*2];   // filtered ref-mic
float y_out[SAMPLE_SIZE];   // noise cancel signal
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

const int32_t channel_num = AS_CHANNEL_STEREO;
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


void signal_process(int16_t* sound_input, int16_t* stereo_output, const uint32_t sample_size) {
//***/
  memset(&spk_out[0], 0, SAMPLE_SIZE*sizeof(int16_t));
  int j = 0;
  for (int n = 0; n < SAMPLE_SIZE*2; n += 2) {
    ref_mic[j] = sound_input[n+0];  // MIC A for the reference mic
    err_mic[j] = sound_input[n+1];  // MIC B for the error mic
    ++j;
  }

  if (onoff) {
    memcpy(&x_in[SAMPLE_SIZE], &x_in[0], SAMPLE_SIZE*sizeof(float)); // shift the previous frame to apply the FIR filter
    memcpy(&e_in[SAMPLE_SIZE], &e_in[0], SAMPLE_SIZE*sizeof(float)); // shift the previous frame to apply the FIR filter
    memcpy(&r_out[SAMPLE_SIZE], &r_out[0], SAMPLE_SIZE*sizeof(float)); // shift the previous frame to apply the FIR filter
    float* x_in_last = &x_in[SAMPLE_SIZE];
    float* e_in_last = &e_in[SAMPLE_SIZE];
    float* r_out_last = &r_out[SAMPLE_SIZE];

    arm_q15_to_float((q15_t*)&ref_mic[0], &x_in[0], SAMPLE_SIZE);   
    arm_q15_to_float((q15_t*)&err_mic[0], &e_in[0], SAMPLE_SIZE);   
    arm_fir_f32(&S, &x_in[0], &r_out[0], SAMPLE_SIZE); // make filtered ref-mic signal

    // apply the adaptive FIR filter
    memset(&y_out[0], 0, sizeof(float)*SAMPLE_SIZE);
    for (int n = SAMPLE_SIZE-1; n >= 0; --n) {
      // update coeff of Filtered X LMS
      static const float mu = 0.3; // tentative value [first 0.1]
      for (int i = H_TAPS-1; i >= 0; --i) {
        h[i] = h[i] - mu*e_in_last[n]*r_out_last[n-i];
      }

      for (int i = H_TAPS-1; i >= 0; --i) {
        y_out[n] += h[i]*x_in_last[n-i];  // apply the FIR filter derived by X-LMS
      }
    }

    // convert float to int16_t    
    arm_float_to_q15(&y_out[0], (q15_t*)&spk_out[0], SAMPLE_SIZE);

  } else {
    for (int n = 0; n < SAMPLE_SIZE; ++n) {
      ref_mic[n] = -ref_mic[n];
    }
    memcpy(&spk_out[0], &ref_mic[0], SAMPLE_SIZE*sizeof(int16_t)); // make no sound.
    //initializeLMS();    
  }

//***/

  /* copy the signal to output buffer */
  for (int n = SAMPLE_SIZE-1; n >= 0; --n)  {
    stereo_output[n*2] = stereo_output[n*2+1] = spk_out[n];
  }

  return;
}

void mixer_stereo_output(uint8_t* stereo_output, const uint32_t out_frame_size) {
  
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
void initializeVirtualFir() {
  arm_fir_init_f32(&S, TAPS, &pCoeffs[0], &pState[0], SAMPLE_SIZE);
}

void initializeLMS() {
  const uint32_t CUTTOFF_FREQ_HZ = 2000; 
  float Fc = (float)CUTTOFF_FREQ_HZ/AS_SAMPLINGRATE_48000;
  const int HALF_TAPS = H_TAPS/2;
  int n = 0;
  for (int k = HALF_TAPS; k >= -HALF_TAPS; --k) {
    if (k == 0) h[n] = 2.*Fc;
    else {
      h[n] = 2.*Fc*arm_sin_f32(2.*PI*Fc*k)/(2.*PI*Fc*k);
    }
    ++n;
  }

  for (int m = 0; m < H_TAPS; ++m) {
    h[m] = (0.5 - 0.5*arm_cos_f32(2*PI*m/H_TAPS))*h[m];
  }
}
//***/

void setup() {
  Serial.begin(115200);

  //*** echo cancel sw
  attachInterrupt(SW_PIN, toggle_sw, FALLING);

  //*** initialize adaptive filter coeff
  initializeVirtualFir();
  initializeLMS();
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