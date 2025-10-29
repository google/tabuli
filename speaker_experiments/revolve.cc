// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <functional>
#include <future>  // NOLINT
#include <sndfile.hh>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"

ABSL_FLAG(int, output_channels, 11, "number of output channels");

// optimize this?!
ABSL_FLAG(double, distance_to_interval_ratio, 4,
          "ratio of (distance between microphone and source array) / (distance "
          "between each source); default = 40cm / 10cm = 4");

namespace {

constexpr int kSubSourcePrecision = 4000;

float SquaredMicrophoneResponse(const float angle) {
  float ret = 0.5f * (1.0f + std::cos(angle));
  return ret * ret;
}

float ExpectedLeftToRightRatio(const float angle) {
  return (1e-14 + SquaredMicrophoneResponse(angle + M_PI / 4)) /
         (1e-14 + SquaredMicrophoneResponse(angle - M_PI / 4));
}

float ActualLeftToRightRatio(float left, float right) {
  return (1e-14 + left) / (1e-14 + right);
}

constexpr int64_t kNumRotators = 256;

float AngleEffect(float dy, float distance, int rot) {
  float dist2 = dy * dy + distance * distance;
  float cos_angle = distance * distance / dist2; // 2nd
  if (rot > 80) {
    cos_angle *= cos_angle;  // 4th
  } else if (rot > 50) {
    cos_angle *= sqrt(cos_angle);  // 3rd
  }
  return cos_angle;
}

float Freq(int i) {
  // Center frequencies of the filter bank, plus one frequency in both ends.
  // [20 * 10**(3 * i / 240.) for i in range(-16, 242)]
  static const float kFreq[258] = {
    12.619146889603865, 12.987632631524226, 13.366878351372293, 13.757198246176152, 14.158915687682757, 14.572363490264555, 14.997884186649117, 15.435830311700247, 15.88656469448563, 16.350460758872998, 16.827902832903902, 17.31928646720131, 17.82501876267491, 18.34551870779559, 18.88121752571847, 19.432559031542123, 20.0, 20.584010543888564, 21.185074503545778, 21.80368984770255, 22.44036908603927, 23.095639693789167, 23.77004454874037, 24.46414238099863, 25.178508235883346, 25.913733950340387, 26.67042864326648, 27.44921922015124, 28.250750892455088, 29.075687712153236, 29.92471312188867, 30.798530521189843, 31.697863849222273, 32.62345818455677, 33.57608036245121, 34.55651961015727, 35.56558820077846, 36.60412212622112, 37.67298178979601, 38.773052719044145, 39.90524629937759, 41.070500529142926, 42.26978079673294, 43.50408068039045, 44.77442277136679, 46.08185952111691, 47.4274741132331, 48.81238136083961, 50.237728630191604, 51.70469679124381, 53.2145011959762, 54.768392685287225, 56.367658625289074, 58.013623973863076, 59.70765237835919, 61.45114730534893, 63.24555320336759, 65.09235669960917, 66.99308783156553, 68.94932131462987, 70.9626778467151, 73.03482545096755, 75.16748085768883, 77.36241092661044, 79.62143411069944, 81.94642196270831, 84.33930068571644, 86.80205272894875, 89.33671843019263, 91.94539770617442, 94.63025179229611, 97.39350503317262, 100.23744672545445, 103.16443301446114, 106.17688884619767, 109.27730997637086, 112.46826503806983, 115.75239766982412, 119.13242870580208, 122.61115842996415, 126.19146889603867, 129.87632631524227, 133.6687835137229, 137.57198246176154, 141.5891568768276, 145.72363490264556, 149.97884186649117, 154.35830311700246, 158.8656469448563, 163.50460758872998, 168.27902832903902, 173.1928646720131, 178.2501876267491, 183.4551870779559, 188.81217525718466, 194.32559031542127, 200.0, 205.8401054388856, 211.85074503545775, 218.03689847702557, 224.40369086039271, 230.95639693789164, 237.70044548740367, 244.64142380998626, 251.7850823588335, 259.13733950340395, 266.7042864326648, 274.49219220151235, 282.5075089245508, 290.75687712153245, 299.2471312188867, 307.9853052118984, 316.9786384922227, 326.2345818455676, 335.76080362451216, 345.5651961015727, 355.65588200778456, 366.04122126221114, 376.72981789796006, 387.73052719044153, 399.052462993776, 410.7050052914292, 422.6978079673293, 435.04080680390445, 447.744227713668, 460.81859521116917, 474.27474113233103, 488.123813608396, 502.3772863019159, 517.0469679124383, 532.145011959762, 547.6839268528722, 563.6765862528907, 580.1362397386306, 597.076523783592, 614.5114730534895, 632.4555320336758, 650.9235669960917, 669.9308783156551, 689.4932131462988, 709.626778467151, 730.3482545096755, 751.6748085768883, 773.6241092661043, 796.2143411069947, 819.4642196270831, 843.3930068571646, 868.0205272894875, 893.367184301926, 919.4539770617446, 946.3025179229611, 973.9350503317262, 1002.3744672545445, 1031.644330144611, 1061.768888461977, 1092.7730997637086, 1124.682650380698, 1157.5239766982413, 1191.3242870580207, 1226.1115842996417, 1261.9146889603867, 1298.7632631524227, 1336.687835137229, 1375.719824617615, 1415.891568768276, 1457.2363490264556, 1499.7884186649117, 1543.5830311700247, 1588.6564694485628, 1635.0460758873005, 1682.79028329039, 1731.9286467201305, 1782.5018762674908, 1834.5518707795588, 1888.121752571847, 1943.2559031542128, 2000.0, 2058.401054388857, 2118.507450354577, 2180.368984770256, 2244.036908603926, 2309.563969378916, 2377.004454874038, 2446.414238099863, 2517.850823588335, 2591.373395034038, 2667.0428643266478, 2744.9219220151253, 2825.075089245508, 2907.5687712153244, 2992.471312188866, 3079.8530521189837, 3169.7863849222285, 3262.345818455676, 3357.608036245121, 3455.6519610157256, 3556.5588200778457, 3660.4122126221137, 3767.2981789796, 3877.3052719044153, 3990.5246299377577, 4107.050052914292, 4226.978079673296, 4350.408068039044, 4477.44227713668, 4608.185952111689, 4742.74741132331, 4881.238136083963, 5023.77286301916, 5170.469679124382, 5321.450119597617, 5476.839268528722, 5636.76586252891, 5801.362397386306, 5970.765237835921, 6145.114730534891, 6324.555320336759, 6509.235669960921, 6699.308783156552, 6894.932131462989, 7096.267784671507, 7303.482545096754, 7516.748085768886, 7736.241092661041, 7962.1434110699465, 8194.642196270826, 8433.930068571646, 8680.20527289488, 8933.67184301926, 9194.539770617446, 9463.025179229606, 9739.350503317262, 10023.74467254545, 10316.44330144611, 10617.688884619769, 10927.73099763708, 11246.82650380698, 11575.239766982417, 11913.242870580207, 12261.115842996416, 12619.14688960386, 12987.632631524228, 13366.878351372297, 13757.198246176149, 14158.915687682762, 14572.363490264546, 14997.884186649117, 15435.830311700254, 15886.564694485625, 16350.460758873003, 16827.902832903896, 17319.286467201306, 17825.018762674918, 18345.51870779559, 18881.21752571847, 19432.559031542118, 20000.0, 20584.010543888573
  };
  return kFreq[i + 1];
}

float FreqAve(int i) {
  return sqrt(Freq(i - 1) * Freq(i + 1));
}

double CalculateBandwidthInHz(int i) {
  return sqrt ((Freq(i + 1) - Freq(i)) * (Freq(i) - Freq(i - 1)));
}

struct PerChannel {
  // [0..1] is for real and imag of 1st leaking accumulation
  // [2..3] is for real and imag of 2nd leaking accumulation
  // [4..5] is for real and imag of 3rd leaking accumulation
  // etc.
  // [14..15] is for real and imag of the 8th order leaking accumulation
  double accu[16][kNumRotators] = {0};
  float LenSqr(size_t i) {
    return accu[14][i] * accu[14][i] + accu[15][i] * accu[15][i];
  }
};

struct Rotators {
  // Four arrays of rotators.
  // [0..1] is real and imag for rotation speed
  // [2..3] is real and image for a frequency rotator of length sqrt(gain[i])
  // Values inserted into the rotators are multiplied with this rotator in both
  // input and output, leading to a total gain multiplication if the length is
  // at sqrt(gain).
  float rot[4][kNumRotators] = {0};
  std::vector<PerChannel> channel;
  // Accu has the channel related data, everything else the same between
  // channels.
  float window[kNumRotators];
  float gain[kNumRotators];
  int16_t delay[kNumRotators] = {0};
  float inputphase[2][kNumRotators] = {0};
  int16_t advance[kNumRotators] = {0};
  int16_t max_delay_ = 0;


  int FindMedian3xLeaker(float window) {
    // Approximate filter delay. TODO: optimize this value along with gain
    // values. Recordings can sound better with -2.32 as it pushes the bass
    // signals a bit earlier and likely compensates human hearing's deficiency
    // for temporal separation.
    static const double kMagic = 6.088830630584044;
    return kMagic / window;
  }

  Rotators() {}
  Rotators(int num_channels, const float sample_rate) {
    channel.resize(num_channels);
    static const float kSampleRate = 48000.0;
    static const float kHzToRad = 2.0f * M_PI / kSampleRate;
    //    static const double kWindow = 0.9988311560491858
    static const double kWindow = 0.9994979253227846;
    for (int i = 0; i < kNumRotators; ++i) {
      float bandwidth = CalculateBandwidthInHz(i);  // bandwidth per bucket.
      window[i] = std::pow(kWindow, bandwidth);
      delay[i] = FindMedian3xLeaker(1.0 - window[i]);
      float windowM1 = 1.0f - window[i];
      max_delay_ = std::max(max_delay_, delay[i]);
      const float f = FreqAve(i) * kHzToRad;
      gain[i] = pow(windowM1, 4.0);
      rot[0][i] = float(std::cos(f));
      rot[1][i] = float(-std::sin(f));
      rot[2][i] = gain[i];
      rot[3][i] = 0.0f;
    }
    for (size_t i = 0; i < kNumRotators; ++i) {
      advance[i] = 1 + max_delay_ - delay[i];
    }
    inputphase[0][0] = 1;
    inputphase[0][1] = 0;
    float prev_phase = 0;
    double phase_array[256] = {
  -0.41128396750238466 + atof(getenv("VAR0")),
  0.14413149159675934 + atof(getenv("VAR1")),
  0.021274476505972084 + atof(getenv("VAR2")),
  -0.21196830370068651 + atof(getenv("VAR3")),
  -0.34419382471041743 + atof(getenv("VAR4")),
  -0.29612125432028003 + atof(getenv("VAR5")),
  -0.23182800493745531 + atof(getenv("VAR6")),
  -0.018538269603347396 + atof(getenv("VAR7")),
  -0.58854420863440127 + atof(getenv("VAR8")),
  -0.51197602219531801 + atof(getenv("VAR9")),
  -0.34972172280628505 + atof(getenv("VAR10")),
  -0.31735320341702145 + atof(getenv("VAR11")),
  -0.063717333053197392 + atof(getenv("VAR12")),
  -0.27796160979119855 + atof(getenv("VAR13")),
  -0.31869707893612614 + atof(getenv("VAR14")),
  -0.32288002998171217 + atof(getenv("VAR15")),
  -0.17527259192965616 + atof(getenv("VAR16")),
  0.6157422844233188 + atof(getenv("VAR17")),
  -0.12386376062052776 + atof(getenv("VAR18")),
  -0.003842335773525829 + atof(getenv("VAR19")),
  -0.90257289357002235 + atof(getenv("VAR20")),
  -0.074624898286712094 + atof(getenv("VAR21")),
  -0.22389566267119781 + atof(getenv("VAR22")),
  -0.82384555743454502 + atof(getenv("VAR23")),
  0.40393700634043339 + atof(getenv("VAR24")),
  0.49983232929210197 + atof(getenv("VAR25")),
  -0.50985126308315509 + atof(getenv("VAR26")),
  -0.077588573860645951 + atof(getenv("VAR27")),
  -0.046922701186041624 + atof(getenv("VAR28")),
  -0.69440452032484479 + atof(getenv("VAR29")),
  -0.27521309439802666 + atof(getenv("VAR30")),
  0.42611640386110594 + atof(getenv("VAR31")),
  0.36512797177422923 + atof(getenv("VAR32")),
  -0.78069611850924425 + atof(getenv("VAR33")),
  -0.77175842278104512 + atof(getenv("VAR34")),
  0.21491880062971366 + atof(getenv("VAR35")),
  0.0044728528995094663 + atof(getenv("VAR36")),
  0.60358676531335309 + atof(getenv("VAR37")),
  -0.12712616281856604 + atof(getenv("VAR38")),
  -0.41728699634001404 + atof(getenv("VAR39")),
  -0.39058422081934907 + atof(getenv("VAR40")),
  0.50344422594729443 + atof(getenv("VAR41")),
  -0.18398368880963781 + atof(getenv("VAR42")),
  0.67008342041556779 + atof(getenv("VAR43")),
  -0.049897926706402182 + atof(getenv("VAR44")),
  -0.012117688967481013 + atof(getenv("VAR45")),
  0.097420497023830432 + atof(getenv("VAR46")),
  0.0029813146148219838 + atof(getenv("VAR47")),
  -0.088777640948113579 + atof(getenv("VAR48")),
  0.20658447900287139 + atof(getenv("VAR49")),
  -0.21463526115310497 + atof(getenv("VAR50")),
  0.3759419651858581 + atof(getenv("VAR51")),
  0.11638267811881861 + atof(getenv("VAR52")),
  0.10863205991857394 + atof(getenv("VAR53")),
  0.43562399668899016 + atof(getenv("VAR54")),
  0.43334850554825344 + atof(getenv("VAR55")),
  -0.33463842940386973 + atof(getenv("VAR56")),
  0.16296980122931282 + atof(getenv("VAR57")),
  0.20159484123662413 + atof(getenv("VAR58")),
  0.48602902689164251 + atof(getenv("VAR59")),
  0.6017109787381556 + atof(getenv("VAR60")),
  0.56864048030870384 + atof(getenv("VAR61")),
  0.42558863597206059 + atof(getenv("VAR62")),
  -0.14775369377139841 + atof(getenv("VAR63")),
  -0.2799205520412994 + atof(getenv("VAR64")),
  0.1083336186149836 + atof(getenv("VAR65")),
  0.13159752094720251 + atof(getenv("VAR66")),
  -0.54441716004060559 + atof(getenv("VAR67")),
  -0.49251240216835018 + atof(getenv("VAR68")),
  -0.44019202880894598 + atof(getenv("VAR69")),
  -0.11885278398283178 + atof(getenv("VAR70")),
  -0.27474034458276808 + atof(getenv("VAR71")),
  0.0073270390860471907 + atof(getenv("VAR72")),
  0.62144060389376699 + atof(getenv("VAR73")),
  0.0081645993392778782 + atof(getenv("VAR74")),
  -0.40266668434081909 + atof(getenv("VAR75")),
  0.3132275617413337 + atof(getenv("VAR76")),
  -0.18116775837218543 + atof(getenv("VAR77")),
  0.32745803957522185 + atof(getenv("VAR78")),
  0.21972655795428858 + atof(getenv("VAR79")),
  0.16010356917497914 + atof(getenv("VAR80")),
  -0.10781095858755482 + atof(getenv("VAR81")),
  0.098712313244843142 + atof(getenv("VAR82")),
  0.49183163546901321 + atof(getenv("VAR83")),
  0.043046816418507799 + atof(getenv("VAR84")),
  0.26837715663279638 + atof(getenv("VAR85")),
  0.18860545588137367 + atof(getenv("VAR86")),
  0.39261732900002455 + atof(getenv("VAR87")),
  -0.2205386321903004 + atof(getenv("VAR88")),
  0.43311813664571619 + atof(getenv("VAR89")),
  -0.57097220426739392 + atof(getenv("VAR90")),
  -0.41358068754010519 + atof(getenv("VAR91")),
  -0.083301272005444682 + atof(getenv("VAR92")),
  -0.068751560676636625 + atof(getenv("VAR93")),
  0.28322124718056474 + atof(getenv("VAR94")),
  0.6608172886121586 + atof(getenv("VAR95")),
  0.087256146699057402 + atof(getenv("VAR96")),
  0.62624261733520104 + atof(getenv("VAR97")),
  -0.31015953851597367 + atof(getenv("VAR98")),
  -0.1003264208530085 + atof(getenv("VAR99")),
  0.56688152412455517 + atof(getenv("VAR100")),
  -0.025229184484796802 + atof(getenv("VAR101")),
  -0.38082444786759284 + atof(getenv("VAR102")),
  0.28689365103959608 + atof(getenv("VAR103")),
  0.31538949593595905 + atof(getenv("VAR104")),
  0.01614842742852389 + atof(getenv("VAR105")),
  0.60323614118618607 + atof(getenv("VAR106")),
  0.30159120408553308 + atof(getenv("VAR107")),
  -0.21240252302781892 + atof(getenv("VAR108")),
  0.077135123768158717 + atof(getenv("VAR109")),
  -0.12555281188612241 + atof(getenv("VAR110")),
  -0.012340895362808371 + atof(getenv("VAR111")),
  -0.10939203720198754 + atof(getenv("VAR112")),
  0.2184411974776535 + atof(getenv("VAR113")),
  0.59783917301188683 + atof(getenv("VAR114")),
  -0.56342677236583616 + atof(getenv("VAR115")),
  -0.10713591173600961 + atof(getenv("VAR116")),
  0.38135166743459575 + atof(getenv("VAR117")),
  -0.3671730029785622 + atof(getenv("VAR118")),
  -0.32686767978992748 + atof(getenv("VAR119")),
  0.18855876207526989 + atof(getenv("VAR120")),
  0.56250646504118418 + atof(getenv("VAR121")),
  -0.31373699904057945 + atof(getenv("VAR122")),
  0.36555469025607118 + atof(getenv("VAR123")),
  0.28150344650956682 + atof(getenv("VAR124")),
  0.34685573097319711 + atof(getenv("VAR125")),
  0.11087530050788222 + atof(getenv("VAR126")),
  0.25093510854802492 + atof(getenv("VAR127")),
  -0.02964743631535674 + atof(getenv("VAR128")),
  -0.0040241031554601389 + atof(getenv("VAR129")),
  -0.77719096779766306 + atof(getenv("VAR130")),
  -0.27891110372128042 + atof(getenv("VAR131")),
  0.033718285478398322 + atof(getenv("VAR132")),
  -0.60580224291790929 + atof(getenv("VAR133")),
  0.26787184473724945 + atof(getenv("VAR134")),
  -0.20628088624464769 + atof(getenv("VAR135")),
  -0.10335511485827394 + atof(getenv("VAR136")),
  0.040405562080482557 + atof(getenv("VAR137")),
  0.28158030137058765 + atof(getenv("VAR138")),
  0.0042437781862204593 + atof(getenv("VAR139")),
  0.051887307942867777 + atof(getenv("VAR140")),
  0.10876135404540552 + atof(getenv("VAR141")),
  -0.027481412012826195 + atof(getenv("VAR142")),
  -0.26454240150534869 + atof(getenv("VAR143")),
  0.52285996056245121 + atof(getenv("VAR144")),
  -0.26696026627064917 + atof(getenv("VAR145")),
  -0.44817304291409871 + atof(getenv("VAR146")),
  -0.18864710111567456 + atof(getenv("VAR147")),
  -0.10886124394596182 + atof(getenv("VAR148")),
  -0.24136779531619512 + atof(getenv("VAR149")),
  -0.16992036372126906 + atof(getenv("VAR150")),
  0.62965130928026447 + atof(getenv("VAR151")),
  0.7505818003300212 + atof(getenv("VAR152")),
  -0.39316661169944478 + atof(getenv("VAR153")),
  0.24887844135439557 + atof(getenv("VAR154")),
  0.25541113530133935 + atof(getenv("VAR155")),
  -0.19378190105205223 + atof(getenv("VAR156")),
  1.3999923246366905 + atof(getenv("VAR157")),
  0.20724227537512493 + atof(getenv("VAR158")),
  0.63994187691050175 + atof(getenv("VAR159")),
  0.345360892836449 + atof(getenv("VAR160")),
  0.14328569694468601 + atof(getenv("VAR161")),
  0.06887153555842479 + atof(getenv("VAR162")),
  0.6138447553104015 + atof(getenv("VAR163")),
  0.15849114738843245 + atof(getenv("VAR164")),
  -0.14344393032606353 + atof(getenv("VAR165")),
  0.54063135101810689 + atof(getenv("VAR166")),
  0.31386988169817454 + atof(getenv("VAR167")),
  -0.59327571288974723 + atof(getenv("VAR168")),
  -0.035603337098825169 + atof(getenv("VAR169")),
  0.24781360594203977 + atof(getenv("VAR170")),
  0.34847024679600747 + atof(getenv("VAR171")),
  0.069422815157051163 + atof(getenv("VAR172")),
  0.12459971920577267 + atof(getenv("VAR173")),
  0.060375015241100907 + atof(getenv("VAR174")),
  0.27007068636456943 + atof(getenv("VAR175")),
  -0.18552269238067251 + atof(getenv("VAR176")),
  -0.35072721446995836 + atof(getenv("VAR177")),
  -0.40304727293515896 + atof(getenv("VAR178")),
  -0.44997647125649071 + atof(getenv("VAR179")),
  -0.28953899173390335 + atof(getenv("VAR180")),
  -0.1651282938872054 + atof(getenv("VAR181")),
  -0.30659257883935398 + atof(getenv("VAR182")),
  -0.25126432827353945 + atof(getenv("VAR183")),
  -0.10875900480990683 + atof(getenv("VAR184")),
  0.22020146272488156 + atof(getenv("VAR185")),
  0.4041121701205348 + atof(getenv("VAR186")),
  -0.10564056189619471 + atof(getenv("VAR187")),
  -0.28734466324736896 + atof(getenv("VAR188")),
  0.78945991154014927 + atof(getenv("VAR189")),
  -0.32977317965898728 + atof(getenv("VAR190")),
  0.014361251843553358 + atof(getenv("VAR191")),
  0.35727597041102394 + atof(getenv("VAR192")),
  -0.72699988185485964 + atof(getenv("VAR193")),
  -0.59587820200495256 + atof(getenv("VAR194")),
  0.36462320528552128 + atof(getenv("VAR195")),
  0.032111167331080825 + atof(getenv("VAR196")),
  -0.028969082806069593 + atof(getenv("VAR197")),
  -0.6046233636463817 + atof(getenv("VAR198")),
  -0.29151138569982271 + atof(getenv("VAR199")),
  -0.12261494462006561 + atof(getenv("VAR200")),
  -0.14243293198814527 + atof(getenv("VAR201")),
  -0.21436198376980353 + atof(getenv("VAR202")),
  0.72056345425622015 + atof(getenv("VAR203")),
  0.061301726891453412 + atof(getenv("VAR204")),
  -0.24627797799367113 + atof(getenv("VAR205")),
  0.32204694049428811 + atof(getenv("VAR206")),
  -0.1466957275023375 + atof(getenv("VAR207")),
  -0.26429657365379916 + atof(getenv("VAR208")),
  -0.073144650104051231 + atof(getenv("VAR209")),
  0.082919748173191934 + atof(getenv("VAR210")),
  -0.050422701601713832 + atof(getenv("VAR211")),
  0.77008901933837393 + atof(getenv("VAR212")),
  -0.48142247197178079 + atof(getenv("VAR213")),
  -0.40702628220251658 + atof(getenv("VAR214")),
  -0.26701200809633102 + atof(getenv("VAR215")),
  -0.43536548578001766 + atof(getenv("VAR216")),
  -0.00036002101386009566 + atof(getenv("VAR217")),
  0.43340572564932173 + atof(getenv("VAR218")),
  -0.35129781233102342 + atof(getenv("VAR219")),
  -0.55458767633435413 + atof(getenv("VAR220")),
  0.035001731907143001 + atof(getenv("VAR221")),
  -0.52826179097384851 + atof(getenv("VAR222")),
  0.99147327476973157 + atof(getenv("VAR223")),
  -0.7543928839823355 + atof(getenv("VAR224")),
  0.1578202216328892 + atof(getenv("VAR225")),
  -0.35630432786728583 + atof(getenv("VAR226")),
  0.15007331263191256 + atof(getenv("VAR227")),
  -0.93901021170462751 + atof(getenv("VAR228")),
  0.54846136094379228 + atof(getenv("VAR229")),
  0.021021961897339809 + atof(getenv("VAR230")),
  0.17652361395271679 + atof(getenv("VAR231")),
  0.19098944386199018 + atof(getenv("VAR232")),
  -0.56214307884192716 + atof(getenv("VAR233")),
  -0.34619590237479381 + atof(getenv("VAR234")),
  0.38485849910215109 + atof(getenv("VAR235")),
  0.18411740965805837 + atof(getenv("VAR236")),
  0.45997670796278528 + atof(getenv("VAR237")),
  -0.17583927327788806 + atof(getenv("VAR238")),
  -0.48649920069695857 + atof(getenv("VAR239")),
  0.28460433962513276 + atof(getenv("VAR240")),
  0.0082369114417856076 + atof(getenv("VAR241")),
  -0.21965666927956301 + atof(getenv("VAR242")),
  0.27520127495613345 + atof(getenv("VAR243")),
  1.0372447399282136 + atof(getenv("VAR244")),
  -0.22806811593623399 + atof(getenv("VAR245")),
  -0.77839837461416739 + atof(getenv("VAR246")),
  -1.096896379745679 + atof(getenv("VAR247")),
  -0.26046190974664729 + atof(getenv("VAR248")),
  2.1186302342039949 + atof(getenv("VAR249")),
  -0.88264720553460574 + atof(getenv("VAR250")),
  -1.5285883260534083 + atof(getenv("VAR251")),
  2.4425250635955775 + atof(getenv("VAR252")),
  0.056020961204742287 + atof(getenv("VAR253")),
  -1.2744303013410645 + atof(getenv("VAR254")),
  -1.4332918198668554 + atof(getenv("VAR255")),
    };
    for (size_t i = 1; i < kNumRotators; ++i) {
      float averfreq = sqrt(FreqAve(i - 1) * FreqAve(i));
      float phase0 = averfreq * advance[i - 1] / kSampleRate;
      float phase1 = averfreq * advance[i] / kSampleRate;
      float almost_one = 0.17542041354405974;
      float f = almost_one * 2 * M_PI * (phase1 - phase0);
      f += prev_phase;
      f += phase_array[i];
      inputphase[0][i] = cos(f);
      inputphase[1][i] = sin(f);
      prev_phase = f;
    }
  }
  void AddAudio(int c, int i, float audio) {
    float raudio = inputphase[0][i] * audio;
    float iaudio = inputphase[1][i] * audio;
    channel[c].accu[0][i] += rot[2][i] * raudio + rot[3][i] * iaudio;
    channel[c].accu[1][i] += -rot[2][i] * iaudio + rot[3][i] * raudio;
  }
  void OccasionallyRenormalize() {
    for (int i = 0; i < kNumRotators; ++i) {
      float norm =
	gain[i] / sqrt(rot[2][i] * rot[2][i] + rot[3][i] * rot[3][i]);
      rot[2][i] *= norm;
      rot[3][i] *= norm;
    }
  }
  void IncrementAll() {
    for (int c = 0; c < channel.size(); ++c) {
      for (int i = 0; i < kNumRotators; i++) {  // clang simdifies this.
	const float w = window[i];
	for (int k = 2; k < 16; ++k) {
	  channel[c].accu[k][i] += channel[c].accu[k - 2][i];
	}
	const float a = rot[2][i], b = rot[3][i];
	rot[2][i] = rot[0][i] * a - rot[1][i] * b;
	rot[3][i] = rot[0][i] * b + rot[1][i] * a;
	for (int k = 0; k < 16; ++k) channel[c].accu[k][i] *= w;
      }
    }
  }
  void GetTriplet(float left_to_right_ratio, int rot_ix,
		  float *out_of_phase_val,
		  float leftr, float lefti, 
		  float rightr, float righti,
		  float prev_leftr, float prev_lefti, 
		  float prev_rightr, float prev_righti,
		  float window,
		  float &left, float &center,  float &right,
		  float &out_of_phase_left,
		  float &out_of_phase_right) {
    float out_of_phase = 0;
    float q = prev_rightr * prev_leftr + prev_righti * prev_lefti;
    if (q < 0 && rot_ix >= 40 && rot_ix < 256 ) {
      float mul = 2.0 * (rot_ix - 44) * (1.0 / 24.0);
      if (mul > 1) {
	mul = 1;
      }
      if (mul < 0) {
	mul = 0;
      }
      out_of_phase = mul * -q / (0.5 * (((prev_rightr * prev_rightr + prev_righti * prev_righti) + (prev_leftr * prev_leftr + prev_lefti * prev_lefti))));
      out_of_phase *= (1.0 / window);
      out_of_phase *= (1.0 / window);
      out_of_phase *= (1.0 / window);
    }
    out_of_phase_val[0] *= 1 - window;
    out_of_phase_val[0] += window * out_of_phase;
    out_of_phase_val[1] *= 1 - window;
    out_of_phase_val[1] += window * out_of_phase_val[0];
    out_of_phase_val[2] *= 1 - window;
    out_of_phase_val[2] += window * out_of_phase_val[1];
    out_of_phase_val[2] = 0;
    out_of_phase_left = out_of_phase_val[2] * (rot[2][rot_ix] * rightr + rot[3][rot_ix] * righti);
    out_of_phase_right = out_of_phase_val[2] * (rot[2][rot_ix] * leftr + rot[3][rot_ix] * lefti);
    leftr *= 1.0 - out_of_phase_val[2];
    lefti *= 1.0 - out_of_phase_val[2];
    rightr *= 1.0 - out_of_phase_val[2];
    righti *= 1.0 - out_of_phase_val[2];

    float rlen = (rightr * rightr + righti * righti) + 1e-20;
    float llen = (leftr * leftr + lefti * lefti) + 1e-20;
    float invsumlen = 1.0 / (rlen + llen);
    rlen *= invsumlen;
    llen *= invsumlen;

    float aver = (rlen * rightr + llen * leftr);
    float avei = (rlen * righti + llen * lefti);

    center = rot[2][rot_ix] * aver + rot[3][rot_ix] * avei;

    rightr -= rlen * aver;
    righti -= rlen * avei;
    leftr -= llen * aver;
    lefti -= llen * avei;

    right = rot[2][rot_ix] * rightr + rot[3][rot_ix] * righti;
    left = rot[2][rot_ix] * leftr + rot[3][rot_ix] * lefti;
    // center = 0;
#if 0
    if (rot_ix != 44) {
       center = left = right = 0;
    }
#endif
  }
};

static constexpr int64_t kBlockSize = 1 << 15;
static const int kHistorySize = (1 << 18);
static const int kHistoryMask = kHistorySize - 1;

struct RotatorFilterBank {
  RotatorFilterBank(size_t num_channels, size_t samplerate) {
    rotators_ = new Rotators(num_channels, samplerate);

    max_delay_ = rotators_->max_delay_;
    QCHECK_LE(max_delay_, kHistorySize);
    fprintf(stderr, "Rotator bank output delay: %zu\n", max_delay_);
  }
  ~RotatorFilterBank() { delete rotators_; }
  Rotators *rotators_;
  int64_t max_delay_;
};

float HardClip(float v) { return std::max(-1.0f, std::min(1.0f, v)); }

struct MultiChannelDriverModel {
  std::vector<float>
      ave;  // For high pass filtering of input voltage (~20 Hz or so)
  std::vector<float> pos;   // Position of the driver membrane.
  std::vector<float> dpos;  // Velocity of the driver membrane.
  void Initialize(size_t n) {
    ave.resize(n);
    pos.resize(n);
    dpos.resize(n);
  }
  void Convert(float *p, size_t n) {
    // This number relates to the resonance frequence of the speakers.
    // I suspect I have around ~100 Hz.
    // It is an ad hoc formula.
    const float kResonance = 100.0;
    // Funny constant -- perhaps from 1.0 / (2 * pi * samplerate),
    // didn't analyze yet why this works, but it does.
    const float kFunnyConstant = 0.0000035;
    const float kSuspension = kFunnyConstant * kResonance;

    // damping reduces the speed of the membrane passively as it
    // emits energy or converts it to heat in the suspension deformations
    const float damping = 0.99999;
    const float kSomewhatRandomNonPhysicalPositionRegularization = 0.99998;

    const float kInputMul = 0.1;

    for (int k = 0; k < n; ++k) {
      float kAve = 0.9995;
      ave[k] *= kAve;
      ave[k] += (1.0 - kAve) * p[k];
      float v = kInputMul * (p[k] - ave[k]);
      dpos[k] *= damping;
      dpos[k] += v;
      pos[k] += dpos[k];
      v += kSuspension * pos[k];
      pos[k] *= kSomewhatRandomNonPhysicalPositionRegularization;
      p[k] = HardClip(v);
    }
  }
};

// Control delays for binaural experience.
struct BinauralModel {
  size_t index = 0;
  float channel[2][4096] = {0};
  void GetAndAdvance(float *left_arg, float *right_arg) {
    *left_arg = HardClip(channel[0][index & 0xfff]);
    *right_arg = HardClip(channel[1][index & 0xfff]);
    /*
    channel[1][(index + 27) & 0xfff] += 0.01 * channel[0][index & 0xfff];
    channel[0][(index + 27) & 0xfff] += 0.01 * channel[1][index & 0xfff];
    */
    channel[0][index & 0xfff] = 0.0;
    channel[1][index & 0xfff] = 0.0;
    ++index;
  }
  void Emit(float *p) { GetAndAdvance(p, p + 1); }
  void WriteWithDelay(size_t c, size_t delay, float v) {
    channel[c][(index + delay) & 0xfff] += v;
  }
  void WriteWithFloatDelay(int c, float float_delay, float v) {
    int delay = floor(float_delay);
    float frac = float_delay - delay;
    WriteWithDelay(c, delay, v * (1.0 - frac));
    WriteWithDelay(c, delay + 1, v * frac);
  }
};

float *GetBinauralTable() {
  static const float binau[16] = {
      1.4, 1.3, 1.2, 1.1,  1.0, 0.9,  0.8, 0.7,

      0.6, 0.5, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15,
  };
  static float table[kNumRotators * 16];
  for (int i = 0; i < kNumRotators; ++i) {
    for (int k = 0; k < 16; ++k) {
      table[i * 16 + k] = pow(binau[k], i / 128.0);
    }
  }
  return table;
}

template <typename In, typename Out>
void Process(const int output_channels_arg, const double distance_to_interval_ratio,
	     bool two_reverb_channels,
             In &input_stream, Out &output_stream,
             Out &binaural_output_stream) {
  std::vector<float> history(input_stream.channels() * kHistorySize);
  std::vector<float> input(input_stream.channels() * kBlockSize);

  int output_channels = two_reverb_channels ? output_channels_arg + 2 : output_channels_arg;
  std::vector<float> output(output_channels * kBlockSize);
  std::vector<float> binaural_output(2 * kBlockSize);
  float midpoint = 0.5 * (output_channels - 1);
  float speaker_offset_left = (2 - midpoint) * 0.1;
  float speaker_offset_right = (9 - midpoint) * 0.1;

  MultiChannelDriverModel dm;
  dm.Initialize(output_channels);
  BinauralModel binaural;
  float *btable = GetBinauralTable();
  constexpr int64_t kNumRotators = 256;
  std::vector<float> filter_gains(kNumRotators);
  for (size_t i = 0; i < kNumRotators; ++i) {
    filter_gains[i] = 0.1; // feels like 1.0 clips occasionally in current config
  }

  RotatorFilterBank rfb(input_stream.channels(),
                        input_stream.samplerate());

  std::vector<float> speaker_to_ratio_table;
  speaker_to_ratio_table.reserve(kSubSourcePrecision * (output_channels_arg - 1) +
                                 1);
  for (int i = 0; i < kSubSourcePrecision * (output_channels_arg - 1) + 1; ++i) {
    const float x_div_interval = static_cast<float>(i) / kSubSourcePrecision -
                                 0.5f * (output_channels_arg - 1);
    const float x_div_distance = x_div_interval / distance_to_interval_ratio;
    const float angle = std::atan(x_div_distance);
    speaker_to_ratio_table.push_back(ExpectedLeftToRightRatio(angle));
  }

  double sum_all_output = 0;
  int64_t total_in = 0;
  bool extend_the_end = true;
  float out_of_phase_array[kNumRotators][3] = {{0}};
  for (;;) {
    int64_t out_ix = 0;
    int64_t read = input_stream.readf(input.data(), kBlockSize);
    for (int i = 0; i < read; ++i) {
      int input_ix = i + total_in;
      history[2 * (input_ix & kHistoryMask) + 0] = input[2 * i];
      history[2 * (input_ix & kHistoryMask) + 1] = input[2 * i + 1];
    }
    // printf("read %d\n", int(read));
    if (read == 0) {
      if (extend_the_end) {
        // Empty the filters and the history.
        extend_the_end = false;
        read = rfb.max_delay_;
        printf("empty the history buffer %d\n", int(read));
        for (int i = 0; i < read; ++i) {
          int input_ix = i + total_in;
          history[2 * (input_ix & kHistoryMask) + 0] = 0;
          history[2 * (input_ix & kHistoryMask) + 1] = 0;
        }
      } else {
        break;
      }
    }
    rfb.rotators_->OccasionallyRenormalize();
    for (int i = 0; i < read; ++i) {
      for (int rot = 0; rot < kNumRotators; ++rot) {
        for (size_t c = 0; c < 2; ++c) {
          int64_t delayed_ix = total_in + i - rfb.rotators_->advance[rot];
          size_t histo_ix = 2 * (delayed_ix & kHistoryMask);
          float delayed = history[histo_ix + c];
	  // Mathematically should be less than 0.5 because two channels can add up.
	  // Imperfect phase corrections etc. can lead to further rendering problems,
	  // thus better to correct more than 0.5, i.e., for example 0.1 and normalize
	  // the output using external tools (such as sox).
	  float kNormalizationToReduceAmplitude = 0.1;
	  delayed *= kNormalizationToReduceAmplitude;
          rfb.rotators_->AddAudio(c, rot, delayed);
        }
      }
      rfb.rotators_->IncrementAll();
      for (int rot = kNumRotators - 1; rot >= 0; --rot) {
	if (rot == -1) {
	  for (int z = 0; z < 16; ++z) {
	    rfb.rotators_->channel[0].accu[z][rot] = 0;
	    rfb.rotators_->channel[1].accu[z][rot] = 0;
	  }
	}
        const float ratio =
	  ActualLeftToRightRatio(rfb.rotators_->channel[1].LenSqr(rot),
				 rfb.rotators_->channel[0].LenSqr(rot));
        float subspeaker_index =
            (std::lower_bound(speaker_to_ratio_table.begin(),
                              speaker_to_ratio_table.end(), ratio,
                              std::greater<>()) -
             speaker_to_ratio_table.begin()) *
            (1.0 / kSubSourcePrecision);
	/*
        if (subspeaker_index < 1.0) {
          subspeaker_index = 1.0;
        }
        if (subspeaker_index >= 11.0) {
          subspeaker_index = 11.0;
        }
	*/
        float stage_size = 1.3;  // meters
        float distance_from_center =
            stage_size * (subspeaker_index - 0.5 * (output_channels_arg - 1)) /
            (output_channels_arg - 1);
        float assumed_distance_to_line = stage_size * 2.0;
        float right = 0, center = 0, left = 0;
	float out_of_phase_right = 0;
	float out_of_phase_left = 0;
	float ratio_expected = subspeaker_index / (output_channels_arg - 1);
        rfb.rotators_->GetTriplet(ratio_expected,
			          rot,
				  &out_of_phase_array[rot][0],
                                  rfb.rotators_->channel[0].accu[14][rot],
                                  rfb.rotators_->channel[0].accu[15][rot],
                                  rfb.rotators_->channel[1].accu[14][rot],
                                  rfb.rotators_->channel[1].accu[15][rot],
                                  rfb.rotators_->channel[0].accu[8][rot],
                                  rfb.rotators_->channel[0].accu[9][rot],
                                  rfb.rotators_->channel[1].accu[8][rot],
                                  rfb.rotators_->channel[1].accu[9][rot],
				  rfb.rotators_->window[rot],
				  left,
                                  center,
				  right,
				  out_of_phase_left,
				  out_of_phase_right);
        if (total_in + i >= rfb.max_delay_) {
#define BINAURAL
#ifdef BINAURAL
          // left and right.
          {
            // left *= 2.0;
            // right *= 2.0;
            //  Some hacks here: 27 samples is roughly 19 cm which I use
            //  as an approximate for the added delay needed for this
            //  kind of left-right 'residual sound'. perhaps it is too
            //  much. perhaps having more than one delay would make sense.
            //
            //  This computation being left-right and with delay accross
            //  causes a relaxed feeling of space to emerge.
            float lbin = left * 2;
            float rbin = right * 2;
            size_t delay = 0;
            for (int i = 0; i < 5; ++i) {
              binaural.WriteWithDelay(0, delay, lbin);
              binaural.WriteWithDelay(1, delay, rbin);

              float lt = btable[16 * rot + 15] * lbin;
              float rt = btable[16 * rot + 15] * rbin;

              // swap:
              lbin = rt;
              rbin = lt;

              if (i == 0) {
                delay += 17;
              } else {
                delay += 27;
              }
            }
          }
          {
            // center.
            int speaker = static_cast<int>(floor(subspeaker_index));
            float off = subspeaker_index - speaker;
            float right_gain_0 = btable[16 * rot + speaker];
            float right_gain_1 = btable[16 * rot + speaker + 1];
            float right_gain = (1.0 - off) * right_gain_0 + off * right_gain_1;
            float left_gain_0 = btable[16 * rot + 15 - speaker];
            float left_gain_1 = btable[16 * rot + 15 - speaker - 1];
            float left_gain = (1.0 - off) * left_gain_0 + off * left_gain_1;
            float kDelayMul = 0.15;
            float delay_p = 0;
            {
              // Making delay diffs less in the center maintains
              // a smaller acoustic picture of the singer.
              // This relates to Euclidian distances and makes
              // physical sense, too.
              float len = (output_channels_arg - 1);
              float dx = subspeaker_index - 0.5 * len;
              float dist = sqrt(dx * dx + len * len) - len;
              if (dx < 0) {
                dist = -dist;
              }
              dist += 0.5 * len;
              delay_p = dist;
            }

            float delay_l = 1 + kDelayMul * delay_p;
            float delay_r = 1 + kDelayMul * ((output_channels_arg - 1) - delay_p);
            binaural.WriteWithFloatDelay(0, delay_l, center * left_gain);
            binaural.WriteWithFloatDelay(1, delay_r, center * right_gain);
          }
#endif
	  
          // for (int kk = rot & 1; kk < output_channels_arg; kk += 1 + (rot < 70 && rot > 8)) {  // distribute alternating frequencies to every 2nd speaker
          //for (int kk = rot & 3; kk < output_channels_arg; kk += 4) {  // the same in groups of 4
	  
	  int speaker = static_cast<int>(floor(subspeaker_index));
	  float off = subspeaker_index - speaker;
	  
	  if (rot < 25) {
	    output[out_ix * output_channels + 0] += 0.5 * right;
	    output[out_ix * output_channels + 1] += 0.5 * right;
	    output[out_ix * output_channels + output_channels_arg - 2] += 0.5 * left;
	    output[out_ix * output_channels + output_channels_arg - 1] += 0.5 * left;
	    float v = center * (1.0f / output_channels_arg);
	    for (int kk = 0; kk < output_channels_arg; kk++) {
	      output[out_ix * output_channels + kk] += v;
	    }
	    float one_per_output_channels_arg = 0.5;
	    if (two_reverb_channels) {
	      output[(out_ix + 1) * output_channels - 2 ] += left * one_per_output_channels_arg;
	      output[(out_ix + 1) * output_channels - 1 ] += right * one_per_output_channels_arg;
	      output[(out_ix + 1) * output_channels - 2 ] += out_of_phase_left * 0.5;
	      output[(out_ix + 1) * output_channels - 1 ] += out_of_phase_right * 0.5;
	    }
	  } else {
	    std::vector<float> w(output_channels_arg);
	    {
	      float sum = 1e-20;
	      for (int kk = 0; kk < output_channels_arg; kk++) {
		float speaker_offset = (kk - midpoint) * 0.1;
		w[kk] = AngleEffect(speaker_offset + distance_from_center,
				    assumed_distance_to_line, rot);
		sum += w[kk];
	      }
	      float scale = 1.0f / sum;
	      for (int kk = 0; kk < output_channels_arg; kk++) {
		output[out_ix * output_channels + kk] += w[kk] * center * scale;
	      }
	    }
	    float sidew[4] = { 0.06, 0.06, 0.05, 0.03 };
	    for (int zz = 0; zz < 4; ++zz) {
	      output[out_ix * output_channels + zz] += sidew[zz] * left;
	      output[out_ix * output_channels + output_channels_arg - 1 - zz] += sidew[zz] * right;
	    }
	    float one_per_output_channels_arg = 0.5 * (1.0 - sidew[0] - sidew[1] - sidew[2] - sidew[3]);
	    if (two_reverb_channels) {
	      // two last channels reserved for reverb signal
	      // add hock normalization
	      output[(out_ix + 1) * output_channels - 2 ] += left * one_per_output_channels_arg;
	      output[(out_ix + 1) * output_channels - 1 ] += right * one_per_output_channels_arg;
	      output[(out_ix + 1) * output_channels - 2 ] += out_of_phase_left * 0.5;
	      output[(out_ix + 1) * output_channels - 1 ] += out_of_phase_right * 0.5;
	    }
	  }
	  for (int hh = 0; hh < output_channels; ++hh) {
	    float v = output[out_ix * output_channels + hh];
	    sum_all_output += v * v;
	  }
        }
      }
      if (total_in + i >= rfb.max_delay_) {
#ifdef BINAURAL
        binaural.Emit(&binaural_output[out_ix * 2]);
#endif
        dm.Convert(&output[out_ix * output_channels], output_channels);
        ++out_ix;
      }
    }
    output_stream.writef(output.data(), out_ix);
    binaural_output_stream.writef(binaural_output.data(), out_ix);
    total_in += read;
    std::fill(output.begin(), output.end(), 0.0);
    std::fill(binaural_output.begin(), binaural_output.end(), 0.0);
  }
  printf("Energy: %.17f\n", 1.0/sum_all_output);
};

}  // namespace

int main(int argc, char **argv) {
  const std::vector<char*> positional_args = absl::ParseCommandLine(argc, argv);

  const int output_channels = absl::GetFlag(FLAGS_output_channels);
  const float distance_to_interval_ratio =
      absl::GetFlag(FLAGS_distance_to_interval_ratio);

  QCHECK_EQ(positional_args.size(), 4)
      << "Usage: " << argv[0]
      << " <input> <multichannel-output> <binaural-headphone-output>";

  SndfileHandle input_file(positional_args[1]);
  QCHECK(input_file) << input_file.strError();

  QCHECK_EQ(input_file.channels(), 2);
  
  bool two_reverb_channels = true;

  SndfileHandle output_file(
      positional_args[2], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/output_channels + 2 * two_reverb_channels, /*samplerate=*/input_file.samplerate());

  SndfileHandle binaural_output_file(
      positional_args[3], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/2, /*samplerate=*/input_file.samplerate());

  Process(output_channels, distance_to_interval_ratio, 
	  two_reverb_channels,
	  input_file, output_file,
          binaural_output_file);
}
