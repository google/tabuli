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
  double accu[8][kNumRotators] = {0};
  double LenSqr(size_t i) {
    return accu[6][i] * accu[6][i] + accu[7][i] * accu[7][i];
  }
};

double get_gain_lut(int i) {
  static const double gain_lut[256] = {
    1 + atof(getenv("VAR512")),
    1 + atof(getenv("VAR513")),
    10.808554960080418 + atof(getenv("VAR258")),
    1 + atof(getenv("VAR259")),
    1 + atof(getenv("VAR260")),
    1 + atof(getenv("VAR261")),
    1 + atof(getenv("VAR262")),
    1 + atof(getenv("VAR263")),
    1 + atof(getenv("VAR264")),
    1 + atof(getenv("VAR265")),
    1 + atof(getenv("VAR266")),
    1 + atof(getenv("VAR267")),
    1 + atof(getenv("VAR268")),
    1 + atof(getenv("VAR269")),
    1 + atof(getenv("VAR270")),
    1 + atof(getenv("VAR271")),
    1 + atof(getenv("VAR272")),
    1 + atof(getenv("VAR273")),
    1 + atof(getenv("VAR274")),
    1 + atof(getenv("VAR275")),
    1 + atof(getenv("VAR276")),
    1 + atof(getenv("VAR277")),
    1 + atof(getenv("VAR278")),
    1 + atof(getenv("VAR279")),
    1 + atof(getenv("VAR280")),
    1 + atof(getenv("VAR281")),
    1 + atof(getenv("VAR282")),
    1 + atof(getenv("VAR283")),
    1 + atof(getenv("VAR284")),
    1 + atof(getenv("VAR285")),
    1 + atof(getenv("VAR286")),
    1 + atof(getenv("VAR287")),
    1 + atof(getenv("VAR288")),
    1 + atof(getenv("VAR289")),
    1 + atof(getenv("VAR290")),
    1 + atof(getenv("VAR291")),
    1 + atof(getenv("VAR292")),
    1 + atof(getenv("VAR293")),
    1 + atof(getenv("VAR294")),
    1 + atof(getenv("VAR295")),
    1 + atof(getenv("VAR296")),
    1 + atof(getenv("VAR297")),
    1 + atof(getenv("VAR298")),
    1 + atof(getenv("VAR299")),
    1 + atof(getenv("VAR300")),
    1 + atof(getenv("VAR301")),
    1 + atof(getenv("VAR302")),
    1 + atof(getenv("VAR303")),
    1 + atof(getenv("VAR304")),
    1 + atof(getenv("VAR305")),
    1 + atof(getenv("VAR306")),
    1 + atof(getenv("VAR307")),
    1 + atof(getenv("VAR308")),
    1 + atof(getenv("VAR309")),
    1 + atof(getenv("VAR310")),
    1 + atof(getenv("VAR311")),
    1 + atof(getenv("VAR312")),
    1 + atof(getenv("VAR313")),
    1 + atof(getenv("VAR314")),
    1 + atof(getenv("VAR315")),
    1 + atof(getenv("VAR316")),
    1 + atof(getenv("VAR317")),
    1 + atof(getenv("VAR318")),
    1 + atof(getenv("VAR319")),
    1 + atof(getenv("VAR320")),
    1 + atof(getenv("VAR321")),
    1 + atof(getenv("VAR322")),
    1 + atof(getenv("VAR323")),
    1 + atof(getenv("VAR324")),
    1 + atof(getenv("VAR325")),
    1 + atof(getenv("VAR326")),
    1 + atof(getenv("VAR327")),
    1 + atof(getenv("VAR328")),
    1 + atof(getenv("VAR329")),
    1 + atof(getenv("VAR330")),
    1 + atof(getenv("VAR331")),
    1 + atof(getenv("VAR332")),
    1 + atof(getenv("VAR333")),
    1 + atof(getenv("VAR334")),
    1 + atof(getenv("VAR335")),
    1 + atof(getenv("VAR336")),
    1 + atof(getenv("VAR337")),
    1 + atof(getenv("VAR338")),
    1 + atof(getenv("VAR339")),
    1 + atof(getenv("VAR340")),
    1 + atof(getenv("VAR341")),
    1 + atof(getenv("VAR342")),
    1 + atof(getenv("VAR343")),
    1 + atof(getenv("VAR344")),
    1 + atof(getenv("VAR345")),
    1 + atof(getenv("VAR346")),
    1 + atof(getenv("VAR347")),
    1 + atof(getenv("VAR348")),
    1 + atof(getenv("VAR349")),
    1 + atof(getenv("VAR350")),
    1 + atof(getenv("VAR351")),
    1 + atof(getenv("VAR352")),
    1 + atof(getenv("VAR353")),
    1 + atof(getenv("VAR354")),
    1 + atof(getenv("VAR355")),
    1 + atof(getenv("VAR356")),
    1 + atof(getenv("VAR357")),
    1 + atof(getenv("VAR358")),
    1 + atof(getenv("VAR359")),
    1 + atof(getenv("VAR360")),
    1 + atof(getenv("VAR361")),
    1 + atof(getenv("VAR362")),
    1 + atof(getenv("VAR363")),
    1 + atof(getenv("VAR364")),
    1 + atof(getenv("VAR365")),
    1 + atof(getenv("VAR366")),
    1 + atof(getenv("VAR367")),
    1 + atof(getenv("VAR368")),
    1 + atof(getenv("VAR369")),
    1 + atof(getenv("VAR370")),
    1 + atof(getenv("VAR371")),
    1 + atof(getenv("VAR372")),
    1 + atof(getenv("VAR373")),
    1 + atof(getenv("VAR374")),
    1 + atof(getenv("VAR375")),
    1 + atof(getenv("VAR376")),
    1 + atof(getenv("VAR377")),
    1 + atof(getenv("VAR378")),
    1 + atof(getenv("VAR379")),
    1 + atof(getenv("VAR380")),
    1 + atof(getenv("VAR381")),
    1 + atof(getenv("VAR382")),
    1 + atof(getenv("VAR383")),
    1 + atof(getenv("VAR384")),
    1 + atof(getenv("VAR385")),
    1 + atof(getenv("VAR386")),
    1 + atof(getenv("VAR387")),
    1 + atof(getenv("VAR388")),
    1 + atof(getenv("VAR389")),
    1 + atof(getenv("VAR390")),
    1 + atof(getenv("VAR391")),
    1 + atof(getenv("VAR392")),
    1 + atof(getenv("VAR393")),
    1 + atof(getenv("VAR394")),
    1 + atof(getenv("VAR395")),
    1 + atof(getenv("VAR396")),
    1 + atof(getenv("VAR397")),
    1 + atof(getenv("VAR398")),
    1 + atof(getenv("VAR399")),
    1 + atof(getenv("VAR400")),
    1 + atof(getenv("VAR401")),
    1 + atof(getenv("VAR402")),
    1 + atof(getenv("VAR403")),
    1 + atof(getenv("VAR404")),
    1 + atof(getenv("VAR405")),
    1 + atof(getenv("VAR406")),
    1 + atof(getenv("VAR407")),
    1 + atof(getenv("VAR408")),
    1 + atof(getenv("VAR409")),
    1 + atof(getenv("VAR410")),
    1 + atof(getenv("VAR411")),
    1 + atof(getenv("VAR412")),
    1 + atof(getenv("VAR413")),
    1 + atof(getenv("VAR414")),
    1 + atof(getenv("VAR415")),
    1 + atof(getenv("VAR416")),
    1 + atof(getenv("VAR417")),
    1 + atof(getenv("VAR418")),
    1 + atof(getenv("VAR419")),
    1 + atof(getenv("VAR420")),
    1 + atof(getenv("VAR421")),
    1 + atof(getenv("VAR422")),
    1 + atof(getenv("VAR423")),
    1 + atof(getenv("VAR424")),
    1 + atof(getenv("VAR425")),
    1 + atof(getenv("VAR426")),
    1 + atof(getenv("VAR427")),
    1 + atof(getenv("VAR428")),
    1 + atof(getenv("VAR429")),
    1 + atof(getenv("VAR430")),
    1 + atof(getenv("VAR431")),
    1 + atof(getenv("VAR432")),
    1 + atof(getenv("VAR433")),
    1 + atof(getenv("VAR434")),
    1 + atof(getenv("VAR435")),
    1 + atof(getenv("VAR436")),
    1 + atof(getenv("VAR437")),
    1 + atof(getenv("VAR438")),
    1 + atof(getenv("VAR439")),
    1 + atof(getenv("VAR440")),
    1 + atof(getenv("VAR441")),
    1 + atof(getenv("VAR442")),
    1 + atof(getenv("VAR443")),
    1 + atof(getenv("VAR444")),
    1 + atof(getenv("VAR445")),
    1 + atof(getenv("VAR446")),
    1 + atof(getenv("VAR447")),
    1 + atof(getenv("VAR448")),
    1 + atof(getenv("VAR449")),
    1 + atof(getenv("VAR450")),
    1 + atof(getenv("VAR451")),
    1 + atof(getenv("VAR452")),
    1 + atof(getenv("VAR453")),
    1 + atof(getenv("VAR454")),
    1 + atof(getenv("VAR455")),
    1 + atof(getenv("VAR456")),
    1 + atof(getenv("VAR457")),
    1 + atof(getenv("VAR458")),
    1 + atof(getenv("VAR459")),
    1 + atof(getenv("VAR460")),
    1 + atof(getenv("VAR461")),
    1 + atof(getenv("VAR462")),
    1 + atof(getenv("VAR463")),
    1 + atof(getenv("VAR464")),
    1 + atof(getenv("VAR465")),
    1 + atof(getenv("VAR466")),
    1 + atof(getenv("VAR467")),
    1 + atof(getenv("VAR468")),
    1 + atof(getenv("VAR469")),
    1 + atof(getenv("VAR470")),
    1 + atof(getenv("VAR471")),
    1 + atof(getenv("VAR472")),
    1 + atof(getenv("VAR473")),
    1 + atof(getenv("VAR474")),
    1 + atof(getenv("VAR475")),
    1 + atof(getenv("VAR476")),
    1 + atof(getenv("VAR477")),
    1 + atof(getenv("VAR478")),
    1 + atof(getenv("VAR479")),
    1 + atof(getenv("VAR480")),
    1 + atof(getenv("VAR481")),
    1 + atof(getenv("VAR482")),
    1 + atof(getenv("VAR483")),
    1 + atof(getenv("VAR484")),
    1 + atof(getenv("VAR485")),
    1 + atof(getenv("VAR486")),
    1 + atof(getenv("VAR487")),
    1 + atof(getenv("VAR488")),
    1 + atof(getenv("VAR489")),
    1 + atof(getenv("VAR490")),
    1 + atof(getenv("VAR491")),
    1 + atof(getenv("VAR492")),
    1 + atof(getenv("VAR493")),
    1 + atof(getenv("VAR494")),
    1 + atof(getenv("VAR495")),
    1 + atof(getenv("VAR496")),
    1 + atof(getenv("VAR497")),
    1 + atof(getenv("VAR498")),
    1 + atof(getenv("VAR499")),
    1 + atof(getenv("VAR500")),
    1 + atof(getenv("VAR501")),
    1 + atof(getenv("VAR502")),
    1 + atof(getenv("VAR503")),
    1 + atof(getenv("VAR504")),
    1 + atof(getenv("VAR505")),
    1 + atof(getenv("VAR506")),
    1 + atof(getenv("VAR507")),
    1 + atof(getenv("VAR508")),
    1 + atof(getenv("VAR509")),
    1 + atof(getenv("VAR510")),
    1 + atof(getenv("VAR511")),
  };
  return abs(gain_lut[i]) * 0.99 + 0.01;
}

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


  int FindMedian3xLeaker(double window) {
    // Approximate filter delay.
    // static const double kMagic = 4.993680535570288; // 47370.85036251393
    // static const double kMagic = 5.002;  // 47345.26851666388
    // static const double kMagic = 5.012; // 47352.448694138846,
    // static const double kMagic = 2.75;  // 3.7552033199e-07
    // static const double kMagic = 2.85;  // 3.824753322e-07
    // static const double kMagic = 2.65;  // 3.8468165241e-07
    // static const double kMagic = 2.76; // 3.7532833173e-07
    // static const double kMagic = 2.765; // 3.7546202344e-07,
    static const double kMagic = 2.7562587531505258 + 0.0001 * atof(getenv("VAR256"));
    
    static const double kRound = 0.0036029491102758291 + 0.01 * atof(getenv("VAR257"));
    return int(kMagic / window + kRound);
  }

  Rotators() {}
  Rotators(int num_channels, const float sample_rate) {
    channel.resize(num_channels);
    static const float kSampleRate = 48000.0;
    static const float kHzToRad = 2.0f * M_PI / kSampleRate;
    //    static const double kWindow = 0.9988311560491858
    static const double kWindow = 0.9996; // 72129.93163818851 (0.9996)->60697->
    //    static const double kWindow = 0.9996;  // 47352.448694138846
    // static const double kWindow = 0.99963;
    for (int i = 0; i < kNumRotators; ++i) {
      float bandwidth = CalculateBandwidthInHz(i);  // bandwidth per bucket.
      //printf("%d %g\n", i, bandwidth);
      window[i] = std::pow(kWindow, bandwidth);
      delay[i] = FindMedian3xLeaker(1.0 - window[i]);
      float windowM1 = 1.0f - window[i];
      max_delay_ = std::max(max_delay_, delay[i]);
      const float f = FreqAve(i) * kHzToRad;
      gain[i] = pow(windowM1, 2.0); // * (i + 10) / 256.0;
      gain[i] *= get_gain_lut(i);
      if (i < 7) {
	gain[i] *= 0.5 + 0.5 * (7 - i) / 7.0;
      }
      //      gain[i] *= sqrt(bandwidth / 300.0);
      rot[0][i] = float(std::cos(f));
      rot[1][i] = float(-std::sin(f));
      rot[2][i] = gain[i];
      rot[3][i] = 0.0f;
    }
    for (size_t i = 0; i < kNumRotators; ++i) {
      advance[i] = max_delay_ - delay[i];
    }
    static const double phase_array[256] = {
-4.2227599918643151 + atof(getenv("VAR0")),
-0.42892549931957469 + atof(getenv("VAR1")),
1.963941062652895 + atof(getenv("VAR2")),
3.594637304796505 + atof(getenv("VAR3")),
-0.60219947119311079 + atof(getenv("VAR4")),
0.55752725744840725 + atof(getenv("VAR5")),
-6.1479490192084603 + atof(getenv("VAR6")),
0.61482415175601501 + atof(getenv("VAR7")),
3.1889597186278538 + atof(getenv("VAR8")),
-0.37393535287952395 + atof(getenv("VAR9")),
14.407694471140413 + atof(getenv("VAR10")),
16.679464161413911 + atof(getenv("VAR11")),
6.4102261111146053 + atof(getenv("VAR12")),
2.2112781204208023 + atof(getenv("VAR13")),
16.893884954921138 + atof(getenv("VAR14")),
19.158110977249397 + atof(getenv("VAR15")),
2.4817799476194722 + atof(getenv("VAR16")),
-7.7934529729276871 + atof(getenv("VAR17")),
-5.3094989655973626 + atof(getenv("VAR18")),
3.1524272207851447 + atof(getenv("VAR19")),
11.649481311892432 + atof(getenv("VAR20")),
7.1336569832425418 + atof(getenv("VAR21")),
-3.5607880003396155 + atof(getenv("VAR22")),
11.087677241709521 + atof(getenv("VAR23")),
0.66051749349127409 + atof(getenv("VAR24")),
-3.6979540262356196 + atof(getenv("VAR25")),
10.766261757661372 + atof(getenv("VAR26")),
0.27913122895809539 + atof(getenv("VAR27")),
2.4102353770307854 + atof(getenv("VAR28")),
10.621836361636955 + atof(getenv("VAR29")),
6.2500891263085361 + atof(getenv("VAR30")),
1.7812168485331044 + atof(getenv("VAR31")),
10.059999538861966 + atof(getenv("VAR32")),
12.083627923971378 + atof(getenv("VAR33")),
7.6317031410954623 + atof(getenv("VAR34")),
9.2879006472972492 + atof(getenv("VAR35")),
4.7850143918207513 + atof(getenv("VAR36")),
0.074384831090404099 + atof(getenv("VAR37")),
-16.728011061319251 + atof(getenv("VAR38")),
-8.4072038572968388 + atof(getenv("VAR39")),
12.255612967722417 + atof(getenv("VAR40")),
1.9274689351199328 + atof(getenv("VAR41")),
4.3279672479969618 + atof(getenv("VAR42")),
0.46691087482477284 + atof(getenv("VAR43")),
-4.1652097701693434 + atof(getenv("VAR44")),
4.065211137554031 + atof(getenv("VAR45")),
2.3766138197952462 + atof(getenv("VAR46")),
0.74108287166955344 + atof(getenv("VAR47")),
1.0301945353047031 + atof(getenv("VAR48")),
1.752683547386251 + atof(getenv("VAR49")),
2.6317204281956164 + atof(getenv("VAR50")),
16.232611729041558 + atof(getenv("VAR51")),
-1.7652280693818005 + atof(getenv("VAR52")),
11.408481129082276 + atof(getenv("VAR53")),
-19.61802944943166 + atof(getenv("VAR54")),
-0.2692184572740689 + atof(getenv("VAR55")),
0.6505704024669926 + atof(getenv("VAR56")),
20.134188654436279 + atof(getenv("VAR57")),
14.508291734710628 + atof(getenv("VAR58")),
-3.9020767242659371 + atof(getenv("VAR59")),
-3.181204869409727 + atof(getenv("VAR60")),
-2.5541025370928585 + atof(getenv("VAR61")),
4.1485804342811603 + atof(getenv("VAR62")),
-1.8204084514212906 + atof(getenv("VAR63")),
-2.4081640020529829 + atof(getenv("VAR64")),
-2.3076325641527431 + atof(getenv("VAR65")),
-1.8834174081306454 + atof(getenv("VAR66")),
4.954476382064585 + atof(getenv("VAR67")),
30.910714212669362 + atof(getenv("VAR68")),
0.74773217439174178 + atof(getenv("VAR69")),
1.8575148257548713 + atof(getenv("VAR70")),
2.6512661259229175 + atof(getenv("VAR71")),
15.50106703327976 + atof(getenv("VAR72")),
3.1586815224662299 + atof(getenv("VAR73")),
-2.7786873456750474 + atof(getenv("VAR74")),
-2.6318540997649666 + atof(getenv("VAR75")),
4.2307357362603266 + atof(getenv("VAR76")),
35.798477674520626 + atof(getenv("VAR77")),
29.372555314094328 + atof(getenv("VAR78")),
4.4875624447612363 + atof(getenv("VAR79")),
10.851143435248316 + atof(getenv("VAR80")),
-1.6895774360380158 + atof(getenv("VAR81")),
11.04332761562968 + atof(getenv("VAR82")),
-7.7939937052292363 + atof(getenv("VAR83")),
11.193959306357614 + atof(getenv("VAR84")),
11.042348410853297 + atof(getenv("VAR85")),
10.734913006952389 + atof(getenv("VAR86")),
-1.9123456843328304 + atof(getenv("VAR87")),
-20.397425005680098 + atof(getenv("VAR88")),
-0.82013470897823337 + atof(getenv("VAR89")),
-6.118751739788121 + atof(getenv("VAR90")),
1.0558809782808836 + atof(getenv("VAR91")),
-17.352667560982429 + atof(getenv("VAR92")),
-16.933225237106992 + atof(getenv("VAR93")),
9.0086424508381135 + atof(getenv("VAR94")),
-15.901390272332328 + atof(getenv("VAR95")),
-3.0635474285754412 + atof(getenv("VAR96")),
9.9520154432676637 + atof(getenv("VAR97")),
4.3123188419839202 + atof(getenv("VAR98")),
-13.992746919952651 + atof(getenv("VAR99")),
-19.492211047552161 + atof(getenv("VAR100")),
-6.7110484983465399 + atof(getenv("VAR101")),
5.7376863152510378 + atof(getenv("VAR102")),
-6.4700346517056415 + atof(getenv("VAR103")),
0.18754170142287738 + atof(getenv("VAR104")),
-5.4691621543759652 + atof(getenv("VAR105")),
1.2790791465512314 + atof(getenv("VAR106")),
8.3936095492199296 + atof(getenv("VAR107")),
2.752021909540884 + atof(getenv("VAR108")),
3.4200315432744106 + atof(getenv("VAR109")),
16.264071682088542 + atof(getenv("VAR110")),
-8.726393729492214 + atof(getenv("VAR111")),
-2.1220247325923194 + atof(getenv("VAR112")),
4.2333910878516052 + atof(getenv("VAR113")),
4.3094397103046678 + atof(getenv("VAR114")),
-20.637359000106482 + atof(getenv("VAR115")),
4.7483428181203253 + atof(getenv("VAR116")),
-7.8780938373983291 + atof(getenv("VAR117")),
11.099574173515196 + atof(getenv("VAR118")),
-1.3179260084232236 + atof(getenv("VAR119")),
-7.1309987044472694 + atof(getenv("VAR120")),
-6.8155886523757117 + atof(getenv("VAR121")),
-0.25501137454898704 + atof(getenv("VAR122")),
12.777215776353112 + atof(getenv("VAR123")),
-11.510624582521558 + atof(getenv("VAR124")),
-4.5454189661965119 + atof(getenv("VAR125")),
-3.9270309720081591 + atof(getenv("VAR126")),
-3.7363294019833035 + atof(getenv("VAR127")),
-3.4849917211982122 + atof(getenv("VAR128")),
3.0161904131957025 + atof(getenv("VAR129")),
-9.6107496593123241 + atof(getenv("VAR130")),
-34.619063464179291 + atof(getenv("VAR131")),
21.943052270738686 + atof(getenv("VAR132")),
15.853441623999931 + atof(getenv("VAR133")),
-27.870461829551221 + atof(getenv("VAR134")),
9.6364884570940106 + atof(getenv("VAR135")),
16.193823989450095 + atof(getenv("VAR136")),
16.160620717116515 + atof(getenv("VAR137")),
3.3435000401201078 + atof(getenv("VAR138")),
28.497957090960305 + atof(getenv("VAR139")),
-2.7791904265624177 + atof(getenv("VAR140")),
3.49999487500736 + atof(getenv("VAR141")),
3.4349040015383125 + atof(getenv("VAR142")),
-21.828463529275318 + atof(getenv("VAR143")),
-9.308145755866116 + atof(getenv("VAR144")),
9.6776462420211953 + atof(getenv("VAR145")),
9.3997329802926668 + atof(getenv("VAR146")),
15.760129550816764 + atof(getenv("VAR147")),
-3.0232406421994509 + atof(getenv("VAR148")),
-2.9466379856399603 + atof(getenv("VAR149")),
-15.616237250256177 + atof(getenv("VAR150")),
3.5830719153363759 + atof(getenv("VAR151")),
3.5569707682707867 + atof(getenv("VAR152")),
-2.657997234515836 + atof(getenv("VAR153")),
9.5459454076196106 + atof(getenv("VAR154")),
16.14861277813273 + atof(getenv("VAR155")),
9.9350549911974877 + atof(getenv("VAR156")),
9.9516622453995591 + atof(getenv("VAR157")),
3.6079157772361858 + atof(getenv("VAR158")),
-46.580152391459649 + atof(getenv("VAR159")),
10.451187012055946 + atof(getenv("VAR160")),
-2.3665659930857954 + atof(getenv("VAR161")),
-2.6909716818578504 + atof(getenv("VAR162")),
-2.4107699832366452 + atof(getenv("VAR163")),
-2.4597554268024382 + atof(getenv("VAR164")),
-2.8797192582686764 + atof(getenv("VAR165")),
-15.292810664063653 + atof(getenv("VAR166")),
-2.6539050061919145 + atof(getenv("VAR167")),
-9.0240759521945133 + atof(getenv("VAR168")),
9.8568616622153531 + atof(getenv("VAR169")),
9.8977408621408767 + atof(getenv("VAR170")),
3.483771223974911 + atof(getenv("VAR171")),
-2.5453453324393718 + atof(getenv("VAR172")),
3.6727220183896803 + atof(getenv("VAR173")),
-9.1072215131842942 + atof(getenv("VAR174")),
16.196116521533671 + atof(getenv("VAR175")),
9.5143138681541508 + atof(getenv("VAR176")),
-15.091379371660123 + atof(getenv("VAR177")),
22.81275820441865 + atof(getenv("VAR178")),
-3.0405121777231412 + atof(getenv("VAR179")),
-15.349336591524676 + atof(getenv("VAR180")),
-2.5577140109736343 + atof(getenv("VAR181")),
-8.7933861650714888 + atof(getenv("VAR182")),
10.029786709211931 + atof(getenv("VAR183")),
-21.410594229755194 + atof(getenv("VAR184")),
-2.6339587098952899 + atof(getenv("VAR185")),
3.5565246356015505 + atof(getenv("VAR186")),
-2.7902660649982627 + atof(getenv("VAR187")),
-9.5262991459378838 + atof(getenv("VAR188")),
-14.997931741080977 + atof(getenv("VAR189")),
16.261478396022707 + atof(getenv("VAR190")),
3.2539863134293654 + atof(getenv("VAR191")),
-2.393324958438888 + atof(getenv("VAR192")),
-2.7849486452962129 + atof(getenv("VAR193")),
-27.182371226950476 + atof(getenv("VAR194")),
3.507129754318091 + atof(getenv("VAR195")),
-2.1963531527661972 + atof(getenv("VAR196")),
9.764054818953074 + atof(getenv("VAR197")),
10.323708529889712 + atof(getenv("VAR198")),
3.1323722732250361 + atof(getenv("VAR199")),
22.397915627237889 + atof(getenv("VAR200")),
3.910177846992609 + atof(getenv("VAR201")),
10.575594064832249 + atof(getenv("VAR202")),
3.2506386094125119 + atof(getenv("VAR203")),
9.856581223007149 + atof(getenv("VAR204")),
-2.5703219150177405 + atof(getenv("VAR205")),
22.770432194151002 + atof(getenv("VAR206")),
-14.625388854040969 + atof(getenv("VAR207")),
4.3871329908319261 + atof(getenv("VAR208")),
-8.0467276351983124 + atof(getenv("VAR209")),
-1.732466708674139 + atof(getenv("VAR210")),
23.486177902642396 + atof(getenv("VAR211")),
-1.6152347831697984 + atof(getenv("VAR212")),
-7.9767652644925509 + atof(getenv("VAR213")),
23.240264815442789 + atof(getenv("VAR214")),
4.2273320360754552 + atof(getenv("VAR215")),
-2.1990351598516993 + atof(getenv("VAR216")),
-2.4892763847064372 + atof(getenv("VAR217")),
-65.722951147360533 + atof(getenv("VAR218")),
11.56056956798496 + atof(getenv("VAR219")),
-1.369981227076589 + atof(getenv("VAR220")),
-8.2997716024585966 + atof(getenv("VAR221")),
-15.14340598228196 + atof(getenv("VAR222")),
-0.63945286495116549 + atof(getenv("VAR223")),
-13.868182658153399 + atof(getenv("VAR224")),
-2.0152217481341443 + atof(getenv("VAR225")),
6.2786289955564625 + atof(getenv("VAR226")),
-7.0978238244392342 + atof(getenv("VAR227")),
-2.3379004622615911 + atof(getenv("VAR228")),
-1.8101005949245461 + atof(getenv("VAR229")),
-3.8875237085847227 + atof(getenv("VAR230")),
18.201699548844054 + atof(getenv("VAR231")),
6.776320285599347 + atof(getenv("VAR232")),
9.5927132232684933 + atof(getenv("VAR233")),
16.439929425923378 + atof(getenv("VAR234")),
1.1579228038383174 + atof(getenv("VAR235")),
-3.5252725833610996 + atof(getenv("VAR236")),
1.2279457190228977 + atof(getenv("VAR237")),
-9.3528729882752639 + atof(getenv("VAR238")),
7.6620091957053056 + atof(getenv("VAR239")),
9.5550703492032696 + atof(getenv("VAR240")),
20.031059096642053 + atof(getenv("VAR241")),
-3.2304747970132635 + atof(getenv("VAR242")),
-5.5003727469258088 + atof(getenv("VAR243")),
8.9400179442692274 + atof(getenv("VAR244")),
4.5632511215140124 + atof(getenv("VAR245")),
1.9890778271283305 + atof(getenv("VAR246")),
-2.3788907123392256 + atof(getenv("VAR247")),
19.863230753376769 + atof(getenv("VAR248")),
9.0888131193771375 + atof(getenv("VAR249")),
-7.865810198532615 + atof(getenv("VAR250")),
-4.8734977544793923 + atof(getenv("VAR251")),
9.4408957606174386 + atof(getenv("VAR252")),
5.0161776481718361 + atof(getenv("VAR253")),
7.6308628411488533 + atof(getenv("VAR254")),
9.1494365252168457 + atof(getenv("VAR255")),
    };
    for (size_t i = 0; i < kNumRotators; ++i) {
      float f = phase_array[i];
      inputphase[0][i] = cos(f);
      inputphase[1][i] = sin(f);
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
	for (int k = 2; k < 8; ++k) {
	  channel[c].accu[k][i] += channel[c].accu[k - 2][i];
	}
	const float a = rot[2][i], b = rot[3][i];
	rot[2][i] = rot[0][i] * a - rot[1][i] * b;
	rot[3][i] = rot[0][i] * b + rot[1][i] * a;
	for (int k = 0; k < 8; ++k) channel[c].accu[k][i] *= w;
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
      if (out_of_phase >= 0.25) {
	out_of_phase = 0.25;
      }
      out_of_phase_val[0] /= (1.0 - window);
      out_of_phase_val[0] = 0.0;
    }
    out_of_phase_val[0] += window * out_of_phase;
    out_of_phase_val[0] *= 1.0 - window;
    out_of_phase_left = out_of_phase_val[0] * (rot[2][rot_ix] * leftr + rot[3][rot_ix] * lefti);
    out_of_phase_right = out_of_phase_val[0] * (rot[2][rot_ix] * rightr + rot[3][rot_ix] * righti);
    leftr *= 1.0 - out_of_phase_val[0];
    lefti *= 1.0 - out_of_phase_val[0];
    rightr *= 1.0 - out_of_phase_val[0];
    righti *= 1.0 - out_of_phase_val[0];

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
    //    return; // do nothing
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
    const float damping = 0.9999;
    const float kSomewhatRandomNonPhysicalPositionRegularization = 0.9999;

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
  double L2_diff_all_channels = 0;
  int64_t total_in = 0;
  bool extend_the_end = true;
  float out_of_phase_array[kNumRotators][1] = {{0}};
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
	  for (int z = 0; z < 8; ++z) {
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
                                  rfb.rotators_->channel[0].accu[6][rot],
                                  rfb.rotators_->channel[0].accu[7][rot],
                                  rfb.rotators_->channel[1].accu[6][rot],
                                  rfb.rotators_->channel[1].accu[7][rot],
                                  rfb.rotators_->channel[0].accu[4][rot],
                                  rfb.rotators_->channel[0].accu[5][rot],
                                  rfb.rotators_->channel[1].accu[4][rot],
                                  rfb.rotators_->channel[1].accu[5][rot],
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
	  {
	    double sum_out_c = 0;
	    double sum_in_c = 0;
	    for (int hh = 0; hh < output_channels; ++hh) {
	      float v = output[out_ix * output_channels + hh];
	      sum_out_c += abs(v);
	    }
	    for (int hh = 0; hh < 2; ++hh) {
	      float v = history[2 * (out_ix & kHistoryMask) + hh];
	      sum_in_c += abs(v);
	    }
	    sum_all_output += sum_out_c;
	    static double kMul = 10.808554960080418 + atof(getenv("VAR258"));
	    double diff = kMul * sum_out_c - sum_in_c;
	    L2_diff_all_channels += abs(diff);
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
  //  printf("Energy: %.17f\n", 1.0/sum_all_output);
  L2_diff_all_channels += sum_all_output;
  L2_diff_all_channels /= pow(sum_all_output, 7);
  printf("Energy: %.17g\n", L2_diff_all_channels);
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
