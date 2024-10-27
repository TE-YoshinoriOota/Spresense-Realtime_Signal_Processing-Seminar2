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

//*****
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <cmsis/arm_math.h>

#define SAMPLE_SIZE 1024
#define TAPS 128
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
0.01042748,
/*
-0.01131512,
-0.02573062,
-0.01650300,
0.00902064,
0.02211103,
0.00805140,
-0.01669927,
-0.02594611,
-0.01523944,
-0.00457759,
-0.01047447,
-0.02567495,
-0.03403560,
-0.03147840,
-0.02617343,
-0.02474308,
-0.02609702,
-0.02535853,
-0.02020691,
-0.01164218,
-0.00281266,
-0.00040789,
-0.00719152,
-0.01711715,
-0.02018588,
-0.01481775,
-0.00896158,
-0.00898391,
-0.01242954,
-0.01102761,
0.00100751,
0.02013459,
0.03047801,
0.02113887,
0.00279197,
-0.00325560,
0.00620711,
0.00790892,
-0.01976454,
-0.06447883,
-0.08962498,
-0.07628893,
-0.04251431,
-0.02206403,
-0.02777269,
-0.04331745,
-0.04646942,
-0.03144125,
-0.01225290,
-0.00832330,
-0.02201243,
-0.03553831,
-0.03164722,
-0.01171335,
0.00580855,
0.00505598,
-0.01250333,
-0.03283379,
-0.04213339,
-0.03589949,
-0.02011857,
-0.00656530,
-0.00170339,
-0.00403128,
-0.00607610,
0.00048901,
0.01490687,
0.02618813,
0.02333395,
0.00949269,
-0.00049294,
0.00243409,
0.00950029,
0.00707138,
-0.00695571,
-0.02334556,
-0.03095250,
-0.02710428,
-0.01897547,
-0.01704981,
-0.02374345,
-0.02982731,
-0.02498321,
-0.01259749,
-0.00560521,
-0.01035656,
-0.01972841,
-0.02068597,
-0.01286879,
-0.00777543,
-0.01017824,
-0.01014054,
0.00119770,
0.01597326,
0.01829136,
0.00504667,
-0.00906916,
-0.01125752,
-0.00525531,
-0.00291071,
-0.00800535,
-0.01129188,
-0.00601361,
-0.00099445,
-0.00715724,
-0.02015370,
-0.02451277,
-0.01556866,
-0.00700925,
-0.01331509,
-0.02907391,
-0.03506323,
-0.02360139,
-0.00789560,
-0.00578357,
-0.01670454,
-0.02418226,
-0.01868573,
-0.00645715,
0.00094555,
-0.00070303,
-0.00586515,
-0.00822792,
-0.00716452,
-0.00447909,
-0.00156420,
0.00153715,
0.00347729
*/
};
//*****

#define TSP_FILE_NAME "tsp_18.wav"
#define OUT_FILE_NAME "tsp_res.wav"

SDClass theSD;

File tspFile;
File outFile;

float pSrc[SAMPLE_SIZE];
float pDst[SAMPLE_SIZE];

WavContainerFormatParser theParser;
WAVHEADER wav_format;

uint32_t s_remain_size = 0;
bool ErrEnd = false;



void setup() {

  Serial.begin(115200);
  /* Initialize FIR filter */
  arm_fir_init_f32(&S, TAPS, &pCoeffs[0], &pState[0], SAMPLE_SIZE);


  /* Initialize SD */
  while (!theSD.begin()) {
    /* wait until SD card is mounted. */
    Serial.println("Insert SD card.");
  }

  // Get wav file info
  fmt_chunk_t fmt;
  handel_wav_parser_t *handle = (handel_wav_parser_t *)theParser.parseChunk("/mnt/sd0/" TSP_FILE_NAME, &fmt);
  if (handle == NULL) {
    Serial.println("Wav parser error.");
    exit(1);
  }

  // Get data chunk info from wav format
  uint32_t data_offset = handle->data_offset;
  s_remain_size = handle->data_size;
  theParser.resetParser((handel_wav_parser *)handle);
  Serial.printf("data_size: %ld\n", s_remain_size);

  /* Open file placed on SD card */
  tspFile = theSD.open(TSP_FILE_NAME);
  /* Verify file open */
  if (!tspFile) {
    Serial.println("TSP File open error");
    exit(1);
  }
  Serial.printf("Read %s\n", tspFile.name());

  /* Open file placed on SD card */
  if (theSD.exists(OUT_FILE_NAME)) theSD.remove(OUT_FILE_NAME);
  outFile = theSD.open(OUT_FILE_NAME, FILE_WRITE);
  /* Verify file open */
  if (!outFile) {
    printf("Out File open error\n");
    exit(1);
  }
  Serial.printf("Write %s\n", outFile.name());

  uint8_t channel_number = 1;
  uint16_t sampling_rate = AS_SAMPLINGRATE_48000;
  uint8_t bit_length = 16;
  wav_format.riff     = CHUNKID_RIFF;
  wav_format.wave     = FORMAT_WAVE;
  wav_format.fmt      = SUBCHUNKID_FMT;
  wav_format.fmt_size = FMT_CHUNK_SIZE;
  wav_format.format   = FORMAT_ID_PCM;
  wav_format.channel  = channel_number;
  wav_format.rate     = sampling_rate;
  wav_format.avgbyte  = sampling_rate * channel_number * (bit_length / 8);
  wav_format.block    = channel_number * (bit_length / 8);
  wav_format.bit      = bit_length;
  wav_format.data     = SUBCHUNKID_DATA;
  wav_format.total_size = s_remain_size + sizeof(WAVHEADER) - 8;
  wav_format.data_size  = s_remain_size;
  outFile.write((uint8_t*)&wav_format, sizeof(WAVHEADER));


  /* Set file position to beginning of data */
  uint8_t wav_buffer[SAMPLE_SIZE*sizeof(int16_t)] = {0};
  uint8_t *s_buffer = wav_buffer; // to avoid a compilation error
  tspFile.seek(data_offset);
  uint32_t ave_duration = 0;
  uint32_t times = 0;
  while (s_remain_size > 0) {
    size_t supply_size = tspFile.read((uint8_t*)s_buffer, SAMPLE_SIZE*sizeof(int16_t));
    uint32_t start_time = millis();
    s_remain_size -= supply_size;
    q15_t* q15_mono = (q15_t*)s_buffer;
    arm_q15_to_float(&q15_mono[0], &pSrc[0], SAMPLE_SIZE);
    for (uint16_t n = 0; n < SAMPLE_SIZE; ++n) {
      pSrc[n] /= 8; // to avoid saturation of the response sound
    }
    arm_fir_f32(&S, &pSrc[0], &pDst[0], SAMPLE_SIZE); 
    arm_float_to_q15(&pDst[0], &q15_mono[0], SAMPLE_SIZE);
    s_buffer = (uint8_t*)q15_mono;
    uint32_t duration = millis() - start_time;
    ave_duration += duration;
    ++times;
    outFile.write((uint8_t*)s_buffer, supply_size);
    if (s_remain_size == 0) break;
  }
  ave_duration /= times;
  Serial.println("average_time: " + String(ave_duration));


  tspFile.close();
  outFile.close();

  Serial.println("End of the process");
    
  while(1);
}

void loop() {
}
