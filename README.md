# Spresense-Realtime_Signal_Processing-Seminar2

Spresense Realtime Signal Processing Advanced Seminar materials
This seminar will cover the following techniques.

(1) Sound source localization and Beamforming using mic-arrays
(2) Adaptive filter for echo cancellation
(3) Active noise cancellation and measurement for impulse response.

## Seminar material
[Spresenseではじめるリアルタイム信号処理プログラミング（応用編）](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Documents/SPRESENSE%E2%84%A2%E3%81%A7%E3%81%AF%E3%81%98%E3%82%81%E3%82%8B%E3%83%AA%E3%82%A2%E3%83%AB%E3%82%BF%E3%82%A4%E3%83%A0%E4%BF%A1%E5%8F%B7%E5%87%A6%E7%90%86%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%9F%E3%83%B3%E3%82%B0%EF%BC%88%E5%BF%9C%E7%94%A8%E7%B7%A8%EF%BC%89.pdf)

## Contents of Sketches
|Sketches directory|contents|
----|----
|[Sketches/1_BF_Sketches/Spresense_BeamSpectrum](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Sketches/1_BF_Sketches/Spresense_BeamSpectrum/Spresense_BeamSpectrum.ino)|Sound source localization program|
|[Sketches/1_BF_Sketches/Spresense_Beamforming](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Sketches/1_BF_Sketches/Spresense_Beamforming/Spresense_Beamforming.ino)|Sound beamforming program|
|[Sketches/2_EC_Sketches/Spresense_FeedbackCanceller](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Sketches/2_EC_Sketches/Spresense_FeedbackCanceller/Spresense_FeedbackCanceller.ino) | Sound Feedback canceller program|
|[Sketches/3_ANC_Sketches/Spresense_tsp_player_wav](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Sketches/3_ANC_Sketches/Spresense_tsp_player_wav/Spresense_tsp_player_wav.ino) | TSP signal player |
|[Sketches/3_ANC_Sketches/Spresense_tsp_recorder_wav](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Sketches/3_ANC_Sketches/Spresense_tsp_recorder_wav/Spresense_tsp_recorder_wav.ino) | TSP signal recorder|
|[Sketches/3_ANC_Sketches/Spresense_IR-FIR2TSP](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Sketches/3_ANC_Sketches/Spresense_IR-FIR2TSP/Spresense_IR-FIR2TSP.ino) | FIR filter based on impulse response made by TSP signal|
|[Sketches/3_ANC_Sketches/Spresense_anc](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Sketches/3_ANC_Sketches/Spresense_anc/Spresense_anc.ino) | Active noise canceller program |

<br/>

|Pythons directory|contents|
----|----
|[Pythons/1_BF_Python/gglobe.py](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Pythons/1_BF_Pythons/gglobe.py) | Create a mic-array chacateristic graph |
|[Pythons/1_BF_Python/simbeam.py](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Pythons/1_BF_Pythons/simbeam.py) | Create a beamforming chacateristic graph |
|[Pythons/3_ANC_Pythons/gen_tsp.py](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Pythons/3_ANC_Pythons/gen_tsp.py) | TSP signal generator |
|[Pythons/3_ANC_Pythons/gen_ir.py](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Pythons/3_ANC_Pythons/gen_ir.py) | Impulse Response generator |

<br/>

|Processing directory|contents|
----|----
|[Processing/Processing_BeamSpectrum/Processing_BeamSpectrum.pde](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar2/blob/main/Processing/Processing_BeamSpectrum/Processing_BeamSpectrum.pde)  
 | Generate the real-time graph on sound source localization |

## License
- Source codes are under LGPL V2.1
- The textbook is under CC-BY-SA <br>
![image](https://github.com/user-attachments/assets/b4e995f8-34ec-491f-924f-9cb25171d59b)
