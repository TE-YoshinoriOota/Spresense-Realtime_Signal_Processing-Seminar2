/*
 *  recorder_wav.ino - Recorder example application for WAV(PCM)
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
#include <arch/board/board.h>

#define RECORD_FILE_NAME "Response.wav"
#define TRIGGER_IN (0)
bool bRecording = false;
void recording_on() { bRecording = true; digitalWrite(LED0, HIGH); }

SDClass theSD;
AudioClass *theAudio;
File myFile;

bool ErrEnd = false;

static void audio_attention_cb(const ErrorAttentionParam *atprm) {
  puts("Attention!");
  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING) {
    ErrEnd = true;
  }
}

static const uint32_t recoding_sampling_rate = 48000;
static const uint8_t  recoding_channel_number = 1;
static const uint8_t  recoding_bit_length = 16;
static const uint32_t recoding_time_ms = 8200;
static const int32_t recoding_byte_per_second = recoding_sampling_rate *
                                                recoding_channel_number *
                                                recoding_bit_length / 8;

static const int32_t recoding_size = recoding_byte_per_second * recoding_time_ms/1000;

void setup() {
  attachInterrupt(TRIGGER_IN, recording_on, FALLING);
  Serial.begin(115200);
  while (!theSD.begin()) { Serial.println("Insert SD card."); }

  theAudio = AudioClass::getInstance();
  theAudio->begin(audio_attention_cb);

  Serial.println("initialization Audio Library");
  theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC);
  theAudio->initRecorder(AS_CODECTYPE_WAV, "/mnt/spif/BIN", recoding_sampling_rate
                       , recoding_bit_length, recoding_channel_number);

  Serial.println("Init Recorder!");
  if (theSD.exists(RECORD_FILE_NAME)) {
    printf("Remove existing file [%s].\n", RECORD_FILE_NAME);
    theSD.remove(RECORD_FILE_NAME);
  }

  if (theSD.exists(RECORD_FILE_NAME)) theSD.remove(RECORD_FILE_NAME);
  myFile = theSD.open(RECORD_FILE_NAME, FILE_WRITE);
  if (!myFile) {
    Serial.println("File open error");
    exit(1);
  }
  Serial.printf("Open! [%s]\n", RECORD_FILE_NAME);

  theAudio->writeWavHeader(myFile);
  Serial.println("Write Header!");

  while(bRecording == false) { delay(100); }

  theAudio->startRecorder();
  Serial.println("Recording Start!");
}

void loop() {
  err_t err;
  if (theAudio->getRecordingSize() > recoding_size) {
    theAudio->stopRecorder();
    sleep(1);
    err = theAudio->readFrames(myFile);
    goto exitRecording;
  }

  err = theAudio->readFrames(myFile);
  if (err != AUDIOLIB_ECODE_OK) {
    Serial.printf("File End! =%d\n", err);
    theAudio->stopRecorder();
    goto exitRecording;
  }

  if (ErrEnd) {
    Serial.println("Error End");
    theAudio->stopRecorder();
    goto exitRecording;
  }

  // usleep(10000);

  return;

exitRecording:
  theAudio->closeOutputFile(myFile);
  myFile.close();
  theAudio->setReadyMode();
  theAudio->end();
  puts("End Recording");
  digitalWrite(LED0, LOW);
  exit(1);
}
