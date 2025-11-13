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
    static const double kMagic = 6.3178825464810267 + atof(getenv("VAR256"));
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
  -0.52063455436524941 + atof(getenv("VAR0")),
  -1.2029804557890014 + atof(getenv("VAR1")),
  0.66069341033585771 + atof(getenv("VAR2")),
  0.66081929264592731 + atof(getenv("VAR3")),
  -0.52758442056951316 + atof(getenv("VAR4")),
  0.1745861978380894 + atof(getenv("VAR5")),
  -0.04459388276404519 + atof(getenv("VAR6")),
  0.67148540068531071 + atof(getenv("VAR7")),
  -0.26431045556656646 + atof(getenv("VAR8")),
  -1.0842007034569179 + atof(getenv("VAR9")),
  -1.6490224585934601 + atof(getenv("VAR10")),
  -0.1631434278502602 + atof(getenv("VAR11")),
  0.39563403834316135 + atof(getenv("VAR12")),
  1.1216395120634799 + atof(getenv("VAR13")),
  -1.5077489915946909 + atof(getenv("VAR14")),
  0.24731468833770104 + atof(getenv("VAR15")),
  -0.1355230677966853 + atof(getenv("VAR16")),
  0.48673317958519619 + atof(getenv("VAR17")),
  -0.39123550203167407 + atof(getenv("VAR18")),
  -0.68767273946882868 + atof(getenv("VAR19")),
  -0.30371892583241178 + atof(getenv("VAR20")),
  -0.38108583846885713 + atof(getenv("VAR21")),
  -0.18112655098133504 + atof(getenv("VAR22")),
  -0.19584521935068935 + atof(getenv("VAR23")),
  0.64946394778300109 + atof(getenv("VAR24")),
  0.51497854989866387 + atof(getenv("VAR25")),
  0.018728145061580517 + atof(getenv("VAR26")),
  -0.3242986615802721 + atof(getenv("VAR27")),
  -0.66019319422304246 + atof(getenv("VAR28")),
  0.72253780043106308 + atof(getenv("VAR29")),
  -0.59685828059949309 + atof(getenv("VAR30")),
  -0.75860272575155918 + atof(getenv("VAR31")),
  0.09090799149239856 + atof(getenv("VAR32")),
  -0.16613529066621707 + atof(getenv("VAR33")),
  0.32990486136207392 + atof(getenv("VAR34")),
  -0.18142346720265268 + atof(getenv("VAR35")),
  -0.038297489047594745 + atof(getenv("VAR36")),
  -0.71081171687221856 + atof(getenv("VAR37")),
  0.68732338690773864 + atof(getenv("VAR38")),
  -0.60026789453807616 + atof(getenv("VAR39")),
  -1.5875589333975149 + atof(getenv("VAR40")),
  -0.35819483727331475 + atof(getenv("VAR41")),
  0.045037299308737892 + atof(getenv("VAR42")),
  0.41388838741469602 + atof(getenv("VAR43")),
  0.079473064173366678 + atof(getenv("VAR44")),
  1.1246359186642456 + atof(getenv("VAR45")),
  0.4413452650499371 + atof(getenv("VAR46")),
  -0.34421295778578076 + atof(getenv("VAR47")),
  0.0097691973181800382 + atof(getenv("VAR48")),
  -0.10143586339802185 + atof(getenv("VAR49")),
  -1.2359103311579556 + atof(getenv("VAR50")),
  -0.63148036614011793 + atof(getenv("VAR51")),
  -0.09939362611959679 + atof(getenv("VAR52")),
  0.81762429060524466 + atof(getenv("VAR53")),
  0.89743923313943808 + atof(getenv("VAR54")),
  -0.0060445919838179162 + atof(getenv("VAR55")),
  -0.22262429044629209 + atof(getenv("VAR56")),
  0.62632166311536341 + atof(getenv("VAR57")),
  0.36750628791667284 + atof(getenv("VAR58")),
  0.050779288506756073 + atof(getenv("VAR59")),
  0.57719484530902276 + atof(getenv("VAR60")),
  0.5960202899359287 + atof(getenv("VAR61")),
  -0.42240759124962113 + atof(getenv("VAR62")),
  -0.2845482592986745 + atof(getenv("VAR63")),
  0.20170189196349084 + atof(getenv("VAR64")),
  0.20491205679853042 + atof(getenv("VAR65")),
  -0.11159309955621524 + atof(getenv("VAR66")),
  -0.22771483973257553 + atof(getenv("VAR67")),
  0.31624240211802762 + atof(getenv("VAR68")),
  -0.90712343938566842 + atof(getenv("VAR69")),
  -0.44656244137981088 + atof(getenv("VAR70")),
  -1.356125816166019 + atof(getenv("VAR71")),
  0.46209346634933018 + atof(getenv("VAR72")),
  0.052293939616959238 + atof(getenv("VAR73")),
  -0.70999394959319795 + atof(getenv("VAR74")),
  -0.59373839541757722 + atof(getenv("VAR75")),
  0.65440737859876597 + atof(getenv("VAR76")),
  0.1391098262904869 + atof(getenv("VAR77")),
  -0.26168506418785914 + atof(getenv("VAR78")),
  -0.51772955942301779 + atof(getenv("VAR79")),
  0.29963597115778456 + atof(getenv("VAR80")),
  0.18773498040223963 + atof(getenv("VAR81")),
  0.026035139388243044 + atof(getenv("VAR82")),
  0.61178338662600285 + atof(getenv("VAR83")),
  1.6036242537391552 + atof(getenv("VAR84")),
  0.10848755977943091 + atof(getenv("VAR85")),
  -0.30939674606425493 + atof(getenv("VAR86")),
  0.064813507774207316 + atof(getenv("VAR87")),
  -0.26479175647427455 + atof(getenv("VAR88")),
  0.34708137144283219 + atof(getenv("VAR89")),
  0.48456251570781866 + atof(getenv("VAR90")),
  -0.19222497756357845 + atof(getenv("VAR91")),
  0.83932444261927186 + atof(getenv("VAR92")),
  0.44049050345958529 + atof(getenv("VAR93")),
  -0.47466080280568773 + atof(getenv("VAR94")),
  0.21541872541403434 + atof(getenv("VAR95")),
  -1.3161160234493037 + atof(getenv("VAR96")),
  1.2106121293320229 + atof(getenv("VAR97")),
  0.56127060585949562 + atof(getenv("VAR98")),
  -0.41684302201225609 + atof(getenv("VAR99")),
  -0.9981112877032563 + atof(getenv("VAR100")),
  0.46589837137276063 + atof(getenv("VAR101")),
  0.53267105367098566 + atof(getenv("VAR102")),
  -0.16201317673128668 + atof(getenv("VAR103")),
  0.44486964347799268 + atof(getenv("VAR104")),
  0.27894250057164865 + atof(getenv("VAR105")),
  -0.24221979207884342 + atof(getenv("VAR106")),
  -0.78799989770169987 + atof(getenv("VAR107")),
  -0.62885129429151743 + atof(getenv("VAR108")),
  -0.39895066738983009 + atof(getenv("VAR109")),
  0.92292732840813629 + atof(getenv("VAR110")),
  0.21358647462899899 + atof(getenv("VAR111")),
  0.0036627880645741856 + atof(getenv("VAR112")),
  -0.83453546960757263 + atof(getenv("VAR113")),
  0.75480267859394801 + atof(getenv("VAR114")),
  1.3006319239810225 + atof(getenv("VAR115")),
  0.64183997623912981 + atof(getenv("VAR116")),
  0.64883391272671243 + atof(getenv("VAR117")),
  -0.49042749987359407 + atof(getenv("VAR118")),
  0.28966604748149893 + atof(getenv("VAR119")),
  -0.55739472340056273 + atof(getenv("VAR120")),
  0.71538278786826992 + atof(getenv("VAR121")),
  -0.78680616728507968 + atof(getenv("VAR122")),
  0.035794291212412108 + atof(getenv("VAR123")),
  0.62419281378634195 + atof(getenv("VAR124")),
  -0.25867874979382882 + atof(getenv("VAR125")),
  -0.59374634619018973 + atof(getenv("VAR126")),
  -0.54056508245915968 + atof(getenv("VAR127")),
  -0.057314911943100878 + atof(getenv("VAR128")),
  -0.55053009851702395 + atof(getenv("VAR129")),
  0.66607229669802381 + atof(getenv("VAR130")),
  -0.56509877830410438 + atof(getenv("VAR131")),
  -0.80340848470578419 + atof(getenv("VAR132")),
  0.47088754881226574 + atof(getenv("VAR133")),
  0.19080729553787651 + atof(getenv("VAR134")),
  -0.9137370031619978 + atof(getenv("VAR135")),
  -0.27114173530681063 + atof(getenv("VAR136")),
  -0.87544802711480474 + atof(getenv("VAR137")),
  0.4712906112808623 + atof(getenv("VAR138")),
  0.046520429541017816 + atof(getenv("VAR139")),
  -0.49039017991566969 + atof(getenv("VAR140")),
  -0.22590950207030103 + atof(getenv("VAR141")),
  -0.14868805735416274 + atof(getenv("VAR142")),
  -0.57185181807445862 + atof(getenv("VAR143")),
  0.10992897363354097 + atof(getenv("VAR144")),
  -0.18307324177324108 + atof(getenv("VAR145")),
  -0.43580901388509646 + atof(getenv("VAR146")),
  -0.37842657339236874 + atof(getenv("VAR147")),
  -0.2446040257663499 + atof(getenv("VAR148")),
  -0.21258016274237077 + atof(getenv("VAR149")),
  0.13691842128222118 + atof(getenv("VAR150")),
  -0.53280460319385303 + atof(getenv("VAR151")),
  -0.018401704667260817 + atof(getenv("VAR152")),
  -0.29782579217495203 + atof(getenv("VAR153")),
  -0.23432900893291622 + atof(getenv("VAR154")),
  -0.35773111400432556 + atof(getenv("VAR155")),
  0.74986176267485971 + atof(getenv("VAR156")),
  4.2478399369628974 + atof(getenv("VAR157")),
  0.34570134043808171 + atof(getenv("VAR158")),
  -0.14921454420955793 + atof(getenv("VAR159")),
  0.15397905768812231 + atof(getenv("VAR160")),
  -0.4563645721713398 + atof(getenv("VAR161")),
  0.010954616775087602 + atof(getenv("VAR162")),
  -0.64561472000809239 + atof(getenv("VAR163")),
  -0.19533370290930838 + atof(getenv("VAR164")),
  -0.25238535010743401 + atof(getenv("VAR165")),
  0.24330619224469019 + atof(getenv("VAR166")),
  -0.43842635824149173 + atof(getenv("VAR167")),
  0.096194169406339777 + atof(getenv("VAR168")),
  -0.64517593885463009 + atof(getenv("VAR169")),
  -0.1032193711842186 + atof(getenv("VAR170")),
  -0.14585671143944881 + atof(getenv("VAR171")),
  6.0142839336817975 + atof(getenv("VAR172")),
  -0.34122325339955739 + atof(getenv("VAR173")),
  -0.44612915754591725 + atof(getenv("VAR174")),
  0.22263312895853127 + atof(getenv("VAR175")),
  -0.65040469239914001 + atof(getenv("VAR176")),
  0.059315277392479458 + atof(getenv("VAR177")),
  -0.019641858837912248 + atof(getenv("VAR178")),
  -0.090332325918584061 + atof(getenv("VAR179")),
  -0.18883045112574443 + atof(getenv("VAR180")),
  -0.28473391600911108 + atof(getenv("VAR181")),
  -0.37753280797447519 + atof(getenv("VAR182")),
  -0.4764919997810807 + atof(getenv("VAR183")),
  0.41073060756752067 + atof(getenv("VAR184")),
  -0.6731137186879057 + atof(getenv("VAR185")),
  0.259410726543359 + atof(getenv("VAR186")),
  -0.88783578090834436 + atof(getenv("VAR187")),
  0.10049462145673371 + atof(getenv("VAR188")),
  0.012019828382398482 + atof(getenv("VAR189")),
  -0.066130349260298163 + atof(getenv("VAR190")),
  -0.14123969503201939 + atof(getenv("VAR191")),
  -0.23396658534913067 + atof(getenv("VAR192")),
  -0.32889025238596198 + atof(getenv("VAR193")),
  -0.43649759583544029 + atof(getenv("VAR194")),
  -0.52415609038840605 + atof(getenv("VAR195")),
  0.7229704479737703 + atof(getenv("VAR196")),
  -0.71748418958373616 + atof(getenv("VAR197")),
  0.58865645796690236 + atof(getenv("VAR198")),
  -0.92182273921258817 + atof(getenv("VAR199")),
  0.45559702132524615 + atof(getenv("VAR200")),
  0.39011307522133121 + atof(getenv("VAR201")),
  -1.2507749860032245 + atof(getenv("VAR202")),
  0.21010218301644057 + atof(getenv("VAR203")),
  0.14207713364199404 + atof(getenv("VAR204")),
  0.055799301059102954 + atof(getenv("VAR205")),
  -0.025227788771411769 + atof(getenv("VAR206")),
  -0.12685621234526784 + atof(getenv("VAR207")),
  -0.22499792185515155 + atof(getenv("VAR208")),
  -0.30975542306269027 + atof(getenv("VAR209")),
  -0.44245487972469444 + atof(getenv("VAR210")),
  -0.54569649937981857 + atof(getenv("VAR211")),
  1.3704849897583375 + atof(getenv("VAR212")),
  -0.74075085751324055 + atof(getenv("VAR213")),
  -0.86722247858202661 + atof(getenv("VAR214")),
  1.2199846332479416 + atof(getenv("VAR215")),
  -1.0843713765070406 + atof(getenv("VAR216")),
  -1.2064578033448627 + atof(getenv("VAR217")),
  1.0432986922475802 + atof(getenv("VAR218")),
  -1.4390905842185651 + atof(getenv("VAR219")),
  0.92829294377210936 + atof(getenv("VAR220")),
  0.89329225397270273 + atof(getenv("VAR221")),
  -1.8090785399807885 + atof(getenv("VAR222")),
  0.7584002222919709 + atof(getenv("VAR223")),
  0.70084278652380261 + atof(getenv("VAR224")),
  -2.2300711875194423 + atof(getenv("VAR225")),
  0.53412361570864897 + atof(getenv("VAR226")),
  0.51573480047462239 + atof(getenv("VAR227")),
  0.29308784935824911 + atof(getenv("VAR228")),
  0.42478416531190555 + atof(getenv("VAR229")),
  0.30418752120131537 + atof(getenv("VAR230")),
  0.15442557767796217 + atof(getenv("VAR231")),
  0.26541604035376654 + atof(getenv("VAR232")),
  -0.04946472727411684 + atof(getenv("VAR233")),
  0.10416619861802461 + atof(getenv("VAR234")),
  -0.11512427595821134 + atof(getenv("VAR235")),
  -0.28575173132260639 + atof(getenv("VAR236")),
  -0.29194489398363949 + atof(getenv("VAR237")),
  -0.435522081360119 + atof(getenv("VAR238")),
  -0.5194848324506568 + atof(getenv("VAR239")),
  -0.62838732685998389 + atof(getenv("VAR240")),
  -0.74543942307576228 + atof(getenv("VAR241")),
  -0.89361713101299978 + atof(getenv("VAR242")),
  -2.7279807091960531 + atof(getenv("VAR243")),
  5.2266030362319258 + atof(getenv("VAR244")),
  -1.1731756534745796 + atof(getenv("VAR245")),
  -1.2738749301353915 + atof(getenv("VAR246")),
  -2.5540386260198598 + atof(getenv("VAR247")),
  -1.3532905408567328 + atof(getenv("VAR248")),
  4.7858637453287463 + atof(getenv("VAR249")),
  -2.4636613937335388 + atof(getenv("VAR250")),
  -1.7543315154832926 + atof(getenv("VAR251")),
  3.8241805933321236 + atof(getenv("VAR252")),
  -2.0303749888766323 + atof(getenv("VAR253")),
  -2.2084391802347234 + atof(getenv("VAR254")),
  -2.4068159368354256 + atof(getenv("VAR255")),
    };
    for (size_t i = 1; i < kNumRotators; ++i) {
      float averfreq = sqrt(FreqAve(i - 1) * FreqAve(i));
      float phase0 = averfreq * advance[i - 1] / kSampleRate;
      float phase1 = averfreq * advance[i] / kSampleRate;
      static const double almost_one = 0.17519540710848353 + 0.001 * atof(getenv("VAR258"));
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
	  
	  if (rot < 50) {
	    // bass.
	    output[out_ix * output_channels + 0] += 0.5 * right;
	    output[out_ix * output_channels + 1] += 0.5 * right;
	    output[out_ix * output_channels + output_channels_arg - 2] += 0.5 * left;
	    output[out_ix * output_channels + output_channels_arg - 1] += 0.5 * left;
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
