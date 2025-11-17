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
  // [24000/96. * 10**(3 * i / 210.) for i in range(0, 242)]
  static const float kFreq[258] = {
    // 47 samples
    4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0, 32.0, 34.0, 36.0, 38.0, 40.0, 42.0, 44.0, 46.0, 48.0, 50.0, 52.0, 54.0, 56.0, 58.0, 60.0, 62.0, 64.0, 66.0, 68.0, 70.0, 72.0, 74.0, 76.0, 78.0, 80.0, 82.0, 84.0, 86.0, 88.0, 90.0, 92.0, 94.0, 96.0, 
    // 211 (258-47) samples
    // [98 * (22000/98)**(i / 209.) for i in range(0, 211)]
98.0, 100.57170671261272, 103.2108999090591, 105.91935056325222, 108.69887612283482, 111.55134172873841, 114.47866146674609, 117.48279965189865, 120.56577214660602, 123.7296477133485, 126.97654940287529, 130.30865597883218, 133.72820337977404, 137.23748621954348, 140.83885932702202, 144.53473932628754, 148.32760625823795, 152.22000524476945, 156.21454819662577, 160.3139155660648, 164.52085814551864, 168.8381989134537, 173.26883492866972, 177.81573927430898, 182.4819630528797, 187.27063743363314, 192.1849757536672, 197.2282756741674, 202.4039213932316, 207.71538591676332, 213.16623338895803, 218.76012148394528, 224.50080386019258, 230.39213267931717, 236.43806119099634, 242.64264638571092, 249.0100517171012, 255.5445498957635, 262.25052575636073, 269.13247919997144, 276.19502821365177, 283.44291196923655, 290.88099400345874, 298.5142654815216, 306.3478485463122, 314.3869997555063, 322.6371136088676, 331.1037261681117, 339.7925187717608, 348.7093218474845, 357.86011882448247, 367.25105014853665, 376.888417402425, 386.77868753446376, 396.928497198014, 407.34465720486514, 418.0341570954844, 429.00416982919705, 440.26205659744807, 451.81537176336985, 463.67186793097676, 475.83950114738263, 488.3264362415357, 501.14105230305216, 514.2919483048231, 527.787948873172, 541.6381102094294, 555.8517261669035, 570.4383344873198, 585.4077232009184, 600.7699371945015, 616.5352849518383, 632.714345470953, 649.3179753629327, 666.3573161370241, 683.8438016769035, 701.7891659131392, 720.2054506969932, 739.1050138808475, 758.500537610675, 778.4050368361213, 798.8318680439071, 819.7947382204119, 841.307714049453, 863.3852313514308, 886.0421047701775, 909.2935377140035, 933.1551325576206, 957.6429011117781, 982.7732753676463, 1008.5631185231503, 1035.0297362986562, 1062.1908885496057, 1090.0648011838819, 1118.670178391917, 1148.026215197735, 1178.1526103393614, 1209.0695794872381, 1240.7978688095154, 1273.3587688933214, 1306.7741290313543, 1341.0663718833807, 1376.2585085224794, 1412.3741538761287, 1449.4375425724954, 1487.4735452025623, 1526.5076850090038, 1566.5661550130098, 1607.6758355905495, 1649.8643125098697, 1693.1598954423375, 1737.5916369590286, 1783.1893520258375, 1829.9836380101683, 1878.0058952126387, 1927.2883479375837, 1977.8640661164754, 2029.7669874987946, 2083.0319404252255, 2137.6946671984733, 2193.7918480673584, 2251.361125840317, 2310.4411311447957, 2371.0715083495047, 2433.292942166925, 2497.1471849539034, 2562.6770847286784, 2629.9266139231213, 2698.9408988894984, 2769.7662501815307, 2842.450193630103, 2917.0415022344487, 2993.5902288902253, 3072.147739976438, 3152.7667498237383, 3235.501356087248, 3320.4070760476257, 3407.540883864753, 3496.961248809011, 3588.7281744958354, 3682.903239149849, 3779.5496369256107, 3878.7322203126982, 3980.5175436535706, 4084.973907803434, 4192.171405962064, 4302.181970708348, 4415.079422269084, 4530.939518054468, 4649.840003493477, 4771.8606642032655, 4897.083379527594, 5025.5921774802, 5157.473291129994, 5292.815216465909, 5431.708771780237, 5574.247158610285, 5720.52602427927, 5870.643526078397, 6024.700397133217, 6182.8000139984015, 6345.048466026378, 6511.554626556309, 6682.430225971197, 6857.78992667218, 7037.751400020249, 7222.435405297103, 7411.965870738062, 7606.469976691466, 7806.078240960289, 8010.924606383328, 8221.14653071466, 8436.885078861713, 8658.285017543865, 8885.494912435006, 9118.667227855367, 9357.958429079417, 9603.529087328543, 9855.543987518895, 10114.172238836776, 10379.587388215734, 10651.967536791492, 10931.495459412916, 11218.358727289118, 11512.749833855109, 11814.86632394037, 12124.910926327102, 12443.091689786967, 12769.622122687791, 13104.721336263745, 13448.614191645263, 13801.531450747232, 14163.709931116848, 14535.39266484494, 14916.829061647419, 15308.27507622635, 15709.993380022814, 16122.25353747696, 16545.33218691342, 16979.51322617355, 17425.088003118926, 17882.355511134116, 18351.622589759736, 18833.204130590548, 19327.423288576727, 19834.611698869998, 20355.109699360313, 20889.266559052277, 21437.440712434636, 22000.0, 22577.321915076314
  };
  return kFreq[i + 1];
}

double FreqAve(int i) {
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
  double LenSqr(size_t i) {
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
  double rot[4][kNumRotators] = {0};
  std::vector<PerChannel> channel;
  // Accu has the channel related data, everything else the same between
  // channels.
  double window[kNumRotators];
  double gain[kNumRotators];
  int16_t delay[kNumRotators] = {0};
  double inputphase[2][kNumRotators] = {0};
  int16_t advance[kNumRotators] = {0};
  int16_t max_delay_ = 0;


  int FindMedian3xLeaker(float window) {
    // Approximate filter delay. TODO: optimize this value along with gain
    // values. Recordings can sound better with -2.32 as it pushes the bass
    // signals a bit earlier and likely compensates human hearing's deficiency
    // for temporal separation.
    static const double kMagic = 6.317866897154742 + atof(getenv("VAR256"));
    return kMagic / window;
  }

  Rotators() {}
  Rotators(int num_channels, const float sample_rate) {
    channel.resize(num_channels);
    static const float kSampleRate = 48000.0;
    static const float kHzToRad = 2.0f * M_PI / kSampleRate;
    //    static const double kWindow = 0.9988311560491858
    static const double kWindow = 0.99949466497330497;
    for (int i = 0; i < kNumRotators; ++i) {
      float bandwidth = CalculateBandwidthInHz(i);  // bandwidth per bucket.
      window[i] = std::pow(kWindow, bandwidth);
      delay[i] = FindMedian3xLeaker(1.0 - window[i]);
      float windowM1 = 1.0f - window[i];
      max_delay_ = std::max(max_delay_, delay[i]);
      const float f = FreqAve(i) * kHzToRad;
      gain[i] = pow(windowM1, 4.0); // * (i + 10) / 256.0;
      rot[0][i] = float(std::cos(f));
      rot[1][i] = float(-std::sin(f));
      rot[2][i] = gain[i];
      rot[3][i] = 0.0f;
    }
    for (size_t i = 0; i < kNumRotators; ++i) {
      advance[i] = 1 + max_delay_ - delay[i];
    }
    inputphase[0][0] = 1;
    inputphase[1][0] = 0;
    float prev_phase = 0;
    double phase_array[256] = {
  -0.52099899647535197 + atof(getenv("VAR0")),
  -1.2029799811033548 + atof(getenv("VAR1")),
  0.66067335010070249 + atof(getenv("VAR2")),
  0.6608318651903563 + atof(getenv("VAR3")),
  -0.52757610327131088 + atof(getenv("VAR4")),
  0.17461124825136537 + atof(getenv("VAR5")),
  -0.044597147752829185 + atof(getenv("VAR6")),
  0.67148871720100045 + atof(getenv("VAR7")),
  -0.26428087746569184 + atof(getenv("VAR8")),
  -1.0842017978419454 + atof(getenv("VAR9")),
  -1.6490282404652237 + atof(getenv("VAR10")),
  -0.16314113008374248 + atof(getenv("VAR11")),
  0.39563310500923671 + atof(getenv("VAR12")),
  1.1216594581939678 + atof(getenv("VAR13")),
  -1.5077461002075225 + atof(getenv("VAR14")),
  0.24730313171737187 + atof(getenv("VAR15")),
  -0.13552224619552666 + atof(getenv("VAR16")),
  0.48673547778022064 + atof(getenv("VAR17")),
  -0.39123928092096188 + atof(getenv("VAR18")),
  -0.68767034175653685 + atof(getenv("VAR19")),
  -0.30374419496011273 + atof(getenv("VAR20")),
  -0.38108646825258113 + atof(getenv("VAR21")),
  -0.18113946794141136 + atof(getenv("VAR22")),
  -0.19584202183896096 + atof(getenv("VAR23")),
  0.6494611502552402 + atof(getenv("VAR24")),
  0.51495444865292661 + atof(getenv("VAR25")),
  0.018729202974779372 + atof(getenv("VAR26")),
  -0.32428562622323892 + atof(getenv("VAR27")),
  -0.66019123745981445 + atof(getenv("VAR28")),
  0.72253574252682873 + atof(getenv("VAR29")),
  -0.59685762193032887 + atof(getenv("VAR30")),
  -0.75861454006469642 + atof(getenv("VAR31")),
  0.090889218028705129 + atof(getenv("VAR32")),
  -0.16613125370728896 + atof(getenv("VAR33")),
  0.3298991100835677 + atof(getenv("VAR34")),
  -0.18141938702508476 + atof(getenv("VAR35")),
  -0.038299321069803792 + atof(getenv("VAR36")),
  -0.7108194752031477 + atof(getenv("VAR37")),
  0.6873216254751664 + atof(getenv("VAR38")),
  -0.60026907748440861 + atof(getenv("VAR39")),
  -1.5875461098202002 + atof(getenv("VAR40")),
  -0.35819458611897798 + atof(getenv("VAR41")),
  0.045043788236920922 + atof(getenv("VAR42")),
  0.41388917493147565 + atof(getenv("VAR43")),
  0.079481039967547018 + atof(getenv("VAR44")),
  1.1246059880715367 + atof(getenv("VAR45")),
  0.44134891396230125 + atof(getenv("VAR46")),
  -0.34421572778790427 + atof(getenv("VAR47")),
  0.0097655767985663465 + atof(getenv("VAR48")),
  -0.10143404933870129 + atof(getenv("VAR49")),
  -1.2359076735606607 + atof(getenv("VAR50")),
  -0.63147126966399192 + atof(getenv("VAR51")),
  -0.099391341014420903 + atof(getenv("VAR52")),
  0.81762737992521373 + atof(getenv("VAR53")),
  0.89744100259895165 + atof(getenv("VAR54")),
  -0.0060399323272223573 + atof(getenv("VAR55")),
  -0.22262241091763871 + atof(getenv("VAR56")),
  0.62632371534361009 + atof(getenv("VAR57")),
  0.3674868983363726 + atof(getenv("VAR58")),
  0.050778588244311258 + atof(getenv("VAR59")),
  0.57719445255025348 + atof(getenv("VAR60")),
  0.59601175930862971 + atof(getenv("VAR61")),
  -0.42239093833099572 + atof(getenv("VAR62")),
  -0.2845460157750625 + atof(getenv("VAR63")),
  0.2017006174955514 + atof(getenv("VAR64")),
  0.20491437057589307 + atof(getenv("VAR65")),
  -0.11159343182570837 + atof(getenv("VAR66")),
  -0.22771616317073218 + atof(getenv("VAR67")),
  0.31624254407275276 + atof(getenv("VAR68")),
  -0.90712471141770568 + atof(getenv("VAR69")),
  -0.44656389391225421 + atof(getenv("VAR70")),
  -1.3561311914466736 + atof(getenv("VAR71")),
  0.46209365256674145 + atof(getenv("VAR72")),
  0.052294500380856576 + atof(getenv("VAR73")),
  -0.70998652960939634 + atof(getenv("VAR74")),
  -0.59373361064270647 + atof(getenv("VAR75")),
  0.65439868548326097 + atof(getenv("VAR76")),
  0.13911128137874648 + atof(getenv("VAR77")),
  -0.2616783709030121 + atof(getenv("VAR78")),
  -0.51773060715679964 + atof(getenv("VAR79")),
  0.29964047666272764 + atof(getenv("VAR80")),
  0.18773676342574389 + atof(getenv("VAR81")),
  0.026037976297094149 + atof(getenv("VAR82")),
  0.61179377617084707 + atof(getenv("VAR83")),
  1.6036069859141016 + atof(getenv("VAR84")),
  0.10848574107161309 + atof(getenv("VAR85")),
  -0.30939622022176866 + atof(getenv("VAR86")),
  0.0648169768910897 + atof(getenv("VAR87")),
  -0.2647898920568505 + atof(getenv("VAR88")),
  0.34707979208200407 + atof(getenv("VAR89")),
  0.48456448597370938 + atof(getenv("VAR90")),
  -0.1922265544934991 + atof(getenv("VAR91")),
  0.83931972636707375 + atof(getenv("VAR92")),
  0.44050344291975224 + atof(getenv("VAR93")),
  -0.47466098691232578 + atof(getenv("VAR94")),
  0.21542315412132237 + atof(getenv("VAR95")),
  -1.3161167552669077 + atof(getenv("VAR96")),
  1.2106042710748963 + atof(getenv("VAR97")),
  0.5612610492372595 + atof(getenv("VAR98")),
  -0.41684533767792775 + atof(getenv("VAR99")),
  -0.99810946894970298 + atof(getenv("VAR100")),
  0.46589996909051901 + atof(getenv("VAR101")),
  0.53266862676311899 + atof(getenv("VAR102")),
  -0.16201235679556217 + atof(getenv("VAR103")),
  0.44486546792880183 + atof(getenv("VAR104")),
  0.27894287246817329 + atof(getenv("VAR105")),
  -0.24222258621631179 + atof(getenv("VAR106")),
  -0.78800263887629884 + atof(getenv("VAR107")),
  -0.62884510929688431 + atof(getenv("VAR108")),
  -0.39895086717260964 + atof(getenv("VAR109")),
  0.92293397912215647 + atof(getenv("VAR110")),
  0.21358192814217633 + atof(getenv("VAR111")),
  0.0036641756122359321 + atof(getenv("VAR112")),
  -0.83453193145862892 + atof(getenv("VAR113")),
  0.75479858228904273 + atof(getenv("VAR114")),
  1.3006316263605682 + atof(getenv("VAR115")),
  0.64183918780811955 + atof(getenv("VAR116")),
  0.64883263758373866 + atof(getenv("VAR117")),
  -0.49042344425226581 + atof(getenv("VAR118")),
  0.28966266805232233 + atof(getenv("VAR119")),
  -0.55739292953410302 + atof(getenv("VAR120")),
  0.71536937577532334 + atof(getenv("VAR121")),
  -0.78680474081170559 + atof(getenv("VAR122")),
  0.035792504335022984 + atof(getenv("VAR123")),
  0.62418101992565034 + atof(getenv("VAR124")),
  -0.25867712617716015 + atof(getenv("VAR125")),
  -0.59375078865494646 + atof(getenv("VAR126")),
  -0.54055874214244903 + atof(getenv("VAR127")),
  -0.057320169944542435 + atof(getenv("VAR128")),
  -0.55052616517668018 + atof(getenv("VAR129")),
  0.66607391905963909 + atof(getenv("VAR130")),
  -0.56509815923411522 + atof(getenv("VAR131")),
  -0.80339235688876742 + atof(getenv("VAR132")),
  0.47087700119832904 + atof(getenv("VAR133")),
  0.19081135954571515 + atof(getenv("VAR134")),
  -0.91374300786445972 + atof(getenv("VAR135")),
  -0.27113988882323553 + atof(getenv("VAR136")),
  -0.87545274843506338 + atof(getenv("VAR137")),
  0.47127457360857977 + atof(getenv("VAR138")),
  0.046518210067160844 + atof(getenv("VAR139")),
  -0.49039306118104742 + atof(getenv("VAR140")),
  -0.22589322697979963 + atof(getenv("VAR141")),
  -0.14868454904137759 + atof(getenv("VAR142")),
  -0.57185244422361425 + atof(getenv("VAR143")),
  0.10992952175373664 + atof(getenv("VAR144")),
  -0.18307343111652485 + atof(getenv("VAR145")),
  -0.43579598366117184 + atof(getenv("VAR146")),
  -0.37843513769146847 + atof(getenv("VAR147")),
  -0.24460537077220923 + atof(getenv("VAR148")),
  -0.21257711824915707 + atof(getenv("VAR149")),
  0.13692059429674036 + atof(getenv("VAR150")),
  -0.53280663831957342 + atof(getenv("VAR151")),
  -0.01840192344725124 + atof(getenv("VAR152")),
  -0.29782487590481155 + atof(getenv("VAR153")),
  -0.23432722416060262 + atof(getenv("VAR154")),
  -0.35773526326022026 + atof(getenv("VAR155")),
  0.74985871293395112 + atof(getenv("VAR156")),
  4.2478472780965228 + atof(getenv("VAR157")),
  0.3457004374034664 + atof(getenv("VAR158")),
  -0.14921252595371468 + atof(getenv("VAR159")),
  0.15398084280523919 + atof(getenv("VAR160")),
  -0.45636055998376379 + atof(getenv("VAR161")),
  0.010954141373062869 + atof(getenv("VAR162")),
  -0.64561884861452512 + atof(getenv("VAR163")),
  -0.19533870273303741 + atof(getenv("VAR164")),
  -0.25238032051083309 + atof(getenv("VAR165")),
  0.24330871269142165 + atof(getenv("VAR166")),
  -0.43842698906824989 + atof(getenv("VAR167")),
  0.096190795179786681 + atof(getenv("VAR168")),
  -0.64517326146166476 + atof(getenv("VAR169")),
  -0.10321684623441062 + atof(getenv("VAR170")),
  -0.14585565472227952 + atof(getenv("VAR171")),
  6.0142885670908592 + atof(getenv("VAR172")),
  -0.34123714555107837 + atof(getenv("VAR173")),
  -0.44612813979793531 + atof(getenv("VAR174")),
  0.22263583543711762 + atof(getenv("VAR175")),
  -0.65040527868228371 + atof(getenv("VAR176")),
  0.059310104326968149 + atof(getenv("VAR177")),
  -0.019640133348218611 + atof(getenv("VAR178")),
  -0.090329040695416918 + atof(getenv("VAR179")),
  -0.18883458820683666 + atof(getenv("VAR180")),
  -0.28472978847920311 + atof(getenv("VAR181")),
  -0.37753636717117511 + atof(getenv("VAR182")),
  -0.47649284681830623 + atof(getenv("VAR183")),
  0.41072055433434868 + atof(getenv("VAR184")),
  -0.67310852492505147 + atof(getenv("VAR185")),
  0.2594154222847857 + atof(getenv("VAR186")),
  -0.88783361668286043 + atof(getenv("VAR187")),
  0.10049631163228903 + atof(getenv("VAR188")),
  0.012016806477160599 + atof(getenv("VAR189")),
  -0.066138884738103171 + atof(getenv("VAR190")),
  -0.14123628373969641 + atof(getenv("VAR191")),
  -0.23396798662319596 + atof(getenv("VAR192")),
  -0.32889221458136342 + atof(getenv("VAR193")),
  -0.43649836822104809 + atof(getenv("VAR194")),
  -0.52415962316374476 + atof(getenv("VAR195")),
  0.72297098588564546 + atof(getenv("VAR196")),
  -0.71748364840476764 + atof(getenv("VAR197")),
  0.5886579987677617 + atof(getenv("VAR198")),
  -0.92182149684563131 + atof(getenv("VAR199")),
  0.45559800367731923 + atof(getenv("VAR200")),
  0.39011482236253431 + atof(getenv("VAR201")),
  -1.2507737672234571 + atof(getenv("VAR202")),
  0.21010287100115624 + atof(getenv("VAR203")),
  0.14207987423275659 + atof(getenv("VAR204")),
  0.055799285522189372 + atof(getenv("VAR205")),
  -0.025225567370862709 + atof(getenv("VAR206")),
  -0.12685719061237591 + atof(getenv("VAR207")),
  -0.22500034454690718 + atof(getenv("VAR208")),
  -0.30975645937428498 + atof(getenv("VAR209")),
  -0.44245379596232931 + atof(getenv("VAR210")),
  -0.54569773960146761 + atof(getenv("VAR211")),
  1.3704841550990017 + atof(getenv("VAR212")),
  -0.7407509420774967 + atof(getenv("VAR213")),
  -0.8672230034226095 + atof(getenv("VAR214")),
  1.2199896754524058 + atof(getenv("VAR215")),
  -1.0843738391874418 + atof(getenv("VAR216")),
  -1.2064583831005626 + atof(getenv("VAR217")),
  1.0432915485488232 + atof(getenv("VAR218")),
  -1.4390913273804551 + atof(getenv("VAR219")),
  0.92829558429259051 + atof(getenv("VAR220")),
  0.89329674161701389 + atof(getenv("VAR221")),
  -1.8090838425320472 + atof(getenv("VAR222")),
  0.75839774003487959 + atof(getenv("VAR223")),
  0.70084241482092269 + atof(getenv("VAR224")),
  -2.2300667407975583 + atof(getenv("VAR225")),
  0.53412277029912358 + atof(getenv("VAR226")),
  0.51573625720505756 + atof(getenv("VAR227")),
  0.29308328515769155 + atof(getenv("VAR228")),
  0.42478570135780774 + atof(getenv("VAR229")),
  0.3041868299840324 + atof(getenv("VAR230")),
  0.15443222091228118 + atof(getenv("VAR231")),
  0.26541855054161118 + atof(getenv("VAR232")),
  -0.049444742016121743 + atof(getenv("VAR233")),
  0.10417347505366505 + atof(getenv("VAR234")),
  -0.11512486120533197 + atof(getenv("VAR235")),
  -0.28575264222519964 + atof(getenv("VAR236")),
  -0.29194797193907457 + atof(getenv("VAR237")),
  -0.43552364471932137 + atof(getenv("VAR238")),
  -0.51948669488098742 + atof(getenv("VAR239")),
  -0.62838909203458804 + atof(getenv("VAR240")),
  -0.74549526551082923 + atof(getenv("VAR241")),
  -0.89361492344353277 + atof(getenv("VAR242")),
  -2.7279809128333601 + atof(getenv("VAR243")),
  5.2266095977057407 + atof(getenv("VAR244")),
  -1.1731844632186708 + atof(getenv("VAR245")),
  -1.2738664608740899 + atof(getenv("VAR246")),
  -2.5540232163956675 + atof(getenv("VAR247")),
  -1.3532975424531064 + atof(getenv("VAR248")),
  4.7857973710833175 + atof(getenv("VAR249")),
  -2.463662679405489 + atof(getenv("VAR250")),
  -1.7543366378890437 + atof(getenv("VAR251")),
  3.8241855537486762 + atof(getenv("VAR252")),
  -2.0303743237982861 + atof(getenv("VAR253")),
  -2.2084648470392105 + atof(getenv("VAR254")),
  -2.4067524909183078 + atof(getenv("VAR255")),
    };
    for (size_t i = 1; i < kNumRotators; ++i) {
      float averfreq = sqrt(FreqAve(i - 1) * FreqAve(i));
      float phase0 = averfreq * advance[i - 1] / kSampleRate;
      float phase1 = averfreq * advance[i] / kSampleRate;
      static const double almost_one = 0.17519540560808894 + 0.001 * atof(getenv("VAR258"));
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
	  
	  if (rot < 40) {
	    // bass.
	    for (int ii = 0; ii < output_channels_arg / 2; ii++) {
	      output[out_ix * output_channels + ii] +=
		right / (output_channels_arg * 0.5);
	      output[out_ix * output_channels + output_channels_arg - ii - 1] += 
		left / (output_channels_arg * 0.5);
	    }
	    // Remove three central speakers for bass output, others will
	    // reproduce bass.
	    float v = center * (1.0f / (output_channels_arg - 3));
	    for (int kk = 0; kk < output_channels_arg; kk++) {
	      if (!(kk + 1 == output_channels_arg / 2 ||
		    kk == output_channels_arg / 2 ||
		    kk - 1 == output_channels_arg / 2)) {
		output[out_ix * output_channels + kk] += v;
	      }
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
