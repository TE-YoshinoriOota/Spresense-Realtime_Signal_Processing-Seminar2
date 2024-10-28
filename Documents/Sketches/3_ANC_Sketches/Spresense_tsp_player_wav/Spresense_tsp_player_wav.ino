/*
 *  player_wav.ino - Simple sound player example application
 *  Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <SDHCI.h>
#include <Audio.h>

#define PLAYBACK_FILE_NAME "tsp_18.wav"
#define SW_BUTTON (0)
#define TRIGGER_OUT (28)
bool bPlay = false;
void play_on() { bPlay = true; digitalWrite(LED0, HIGH); }


SDClass theSD;
AudioClass *theAudio;
File myFile;
WavContainerFormatParser theParser;

const int32_t sc_buffer_size = 6144;
uint8_t s_buffer[sc_buffer_size];
uint32_t s_remain_size = 0;
bool ErrEnd = false;

static void audio_attention_cb(const ErrorAttentionParam *atprm) {
  puts("Attention!"); 
  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING) {
    ErrEnd = true;
  }
}

static const uint32_t sc_prestore_frames = 10;
 
void setup() {
  attachInterrupt(SW_BUTTON, play_on, FALLING);
  digitalWrite(TRIGGER_OUT, HIGH);
  Serial.begin(115200);
  while (!theSD.begin()) { Serial.println("Insert SD card"); }

  fmt_chunk_t fmt;
  handel_wav_parser_t *handle = (handel_wav_parser_t*)theParser.parseChunk("/mnt/sd0/" PLAYBACK_FILE_NAME, &fmt);
  if (handle == NULL) {
    Serial.println("Wav parser error");
    exit(1);
  }

  uint32_t data_offset = handle->data_offset;
  s_remain_size = handle->data_size;
  theParser.resetParser((handel_wav_parser *)handle);

  theAudio = AudioClass::getInstance();
  theAudio->begin(audio_attention_cb);

  puts("initialization Audio Library");
  theAudio->setRenderingClockMode((fmt.rate <= 48000) ? AS_CLKMODE_NORMAL : AS_CLKMODE_HIRES);
  theAudio->setPlayerMode(AS_SETPLAYER_OUTPUTDEVICE_SPHP, AS_SP_DRV_MODE_LINEOUT);
  err_t err = theAudio->initPlayer(AudioClass::Player0, AS_CODECTYPE_WAV, "/mnt/spif/BIN", fmt.rate, fmt.bit, fmt.channel);
  if (err != AUDIOLIB_ECODE_OK) {
    Serial.println("Player0 initialize error");
    exit(1);
  }

  myFile = theSD.open(PLAYBACK_FILE_NAME);
  if (!myFile) {
    Serial.printf("File open error: %s\n", PLAYBACK_FILE_NAME);
    exit(1);
  }
  Serial.printf("Open! %s\n", myFile.name());

  myFile.seek(data_offset);

  for (uint32_t i = 0; i < sc_prestore_frames; i++) {
    size_t supply_size = myFile.read(s_buffer, sizeof(s_buffer));
    s_remain_size -= supply_size;
    err = theAudio->writeFrames(AudioClass::Player0, s_buffer, supply_size);
    if (err != AUDIOLIB_ECODE_OK) { break; }
    if (s_remain_size == 0) { break; }
  }
  theAudio->setVolume(-160);

  while (bPlay == false) { delay(100); }
  digitalWrite(TRIGGER_OUT, LOW);

  theAudio->startPlayer(AudioClass::Player0);
  Serial.println("Play!");
}


static const uint32_t sc_store_frames = 10;

void loop() {
  static bool is_carry_over = false;
  static size_t supply_size = 0;

  for (uint32_t i = 0; i < sc_store_frames; i++) {
    if (!is_carry_over) {
      supply_size = myFile.read(s_buffer, (s_remain_size < sizeof(s_buffer)) ? s_remain_size : sizeof(s_buffer));
      s_remain_size -= supply_size;
    }
    is_carry_over = false;

    int err = theAudio->writeFrames(AudioClass::Player0, s_buffer, supply_size);

    if (err == AUDIOLIB_ECODE_SIMPLEFIFO_ERROR) {
      is_carry_over = true;
      break;
    }

    if (s_remain_size == 0) { goto stop_player; }
  }

  if (ErrEnd) {
    Serial.println("Error End");
    goto stop_player;
  }

  usleep(1000);
 
  return;

stop_player:
  theAudio->stopPlayer(AudioClass::Player0);
  myFile.close();
  theAudio->setReadyMode();
  theAudio->end();
  Serial.println("Exit player");
  digitalWrite(LED0, LOW);
  exit(1);
}
