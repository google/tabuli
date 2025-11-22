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

double get_gain_lut(int i) {
  static const double gain_lut[256] = {
    1 + atof(getenv("VAR512")),
    1 + atof(getenv("VAR513")),
    1 + atof(getenv("VAR258")),
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

static double g_entropy = 0;

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
    // Approximate filter delay.
    static const double kMagic = 4.993680535570288;
    return kMagic / window;
  }

  Rotators() {}
  Rotators(int num_channels, const float sample_rate) {
    channel.resize(num_channels);
    static const float kSampleRate = 48000.0;
    static const float kHzToRad = 2.0f * M_PI / kSampleRate;
    //    static const double kWindow = 0.9988311560491858
    static const double kWindow = 0.9996;
    // static const double kWindow = 0.99963;
    for (int i = 0; i < kNumRotators; ++i) {
      float bandwidth = CalculateBandwidthInHz(i);  // bandwidth per bucket.
      //printf("%d %g\n", i, bandwidth);
      window[i] = std::pow(kWindow, bandwidth);
      delay[i] = FindMedian3xLeaker(1.0 - window[i]);
      float windowM1 = 1.0f - window[i];
      max_delay_ = std::max(max_delay_, delay[i]);
      const float f = FreqAve(i) * kHzToRad;
      gain[i] = pow(windowM1, 4.0); // * (i + 10) / 256.0;
      gain[i] *= get_gain_lut(i);
      //      gain[i] *= sqrt(bandwidth / 300.0);
      if (i != 0) {
	double v = get_gain_lut(i) - get_gain_lut(i - 1);
	g_entropy += abs(v);
      }
      g_entropy += abs(get_gain_lut(i) - 1.0);
      rot[0][i] = float(std::cos(f));
      rot[1][i] = float(-std::sin(f));
      rot[2][i] = gain[i];
      rot[3][i] = 0.0f;
    }
    for (size_t i = 0; i < kNumRotators; ++i) {
      advance[i] = 1 + max_delay_ - delay[i];
    }
    static const double phase_array[256] = {
-7.517642422827687 + atof(getenv("VAR0")),
0.93927349156670492 + atof(getenv("VAR1")),
3.5512166031666719 + atof(getenv("VAR2")),
-0.18091933544819006 + atof(getenv("VAR3")),
-3.7914850746379893 + atof(getenv("VAR4")),
-0.6073048437258225 + atof(getenv("VAR5")),
-4.4752277869383601 + atof(getenv("VAR6")),
-3.0647878182896648 + atof(getenv("VAR7")),
-1.1842910453887616 + atof(getenv("VAR8")),
1.2636044202960282 + atof(getenv("VAR9")),
9.3848899810805566 + atof(getenv("VAR10")),
10.722778652472542 + atof(getenv("VAR11")),
-0.47793138534137575 + atof(getenv("VAR12")),
1.7896585717097615 + atof(getenv("VAR13")),
10.424996658297465 + atof(getenv("VAR14")),
12.970450098236368 + atof(getenv("VAR15")),
-9.1701737000059218 + atof(getenv("VAR16")),
-12.858165856977411 + atof(getenv("VAR17")),
-4.2865396562342877 + atof(getenv("VAR18")),
-2.9645436670600476 + atof(getenv("VAR19")),
10.935135661965473 + atof(getenv("VAR20")),
0.11680789404755736 + atof(getenv("VAR21")),
-9.8405735823110039 + atof(getenv("VAR22")),
11.41811495473616 + atof(getenv("VAR23")),
-4.2346968072165883 + atof(getenv("VAR24")),
-2.6182296438129415 + atof(getenv("VAR25")),
11.55973881075418 + atof(getenv("VAR26")),
2.0682196352651854 + atof(getenv("VAR27")),
-2.4160657608935385 + atof(getenv("VAR28")),
14.912746896090788 + atof(getenv("VAR29")),
5.7635315572077515 + atof(getenv("VAR30")),
-1.9052711955753836 + atof(getenv("VAR31")),
8.5479463768624822 + atof(getenv("VAR32")),
4.9560031258467223 + atof(getenv("VAR33")),
2.4411011988074089 + atof(getenv("VAR34")),
5.1836148590134021 + atof(getenv("VAR35")),
1.2552623453435743 + atof(getenv("VAR36")),
-1.2209274907572427 + atof(getenv("VAR37")),
-12.397293283746086 + atof(getenv("VAR38")),
-7.5071545586447019 + atof(getenv("VAR39")),
8.0666628530315254 + atof(getenv("VAR40")),
-0.37767713270484071 + atof(getenv("VAR41")),
6.667323577492267 + atof(getenv("VAR42")),
2.1107480727860479 + atof(getenv("VAR43")),
-0.68564646611905766 + atof(getenv("VAR44")),
1.9232721211115731 + atof(getenv("VAR45")),
-1.7755628569182194 + atof(getenv("VAR46")),
0.025126862396785833 + atof(getenv("VAR47")),
1.6303247874517452 + atof(getenv("VAR48")),
-4.4939058925677564 + atof(getenv("VAR49")),
1.5304778785634792 + atof(getenv("VAR50")),
18.949417938644977 + atof(getenv("VAR51")),
-1.7966044141946822 + atof(getenv("VAR52")),
3.9274780588320715 + atof(getenv("VAR53")),
-21.65135741469096 + atof(getenv("VAR54")),
2.9961790928257068 + atof(getenv("VAR55")),
-4.5355130983603926 + atof(getenv("VAR56")),
18.401450247702549 + atof(getenv("VAR57")),
11.335854565904262 + atof(getenv("VAR58")),
3.5545669396169561 + atof(getenv("VAR59")),
-3.5185956210079108 + atof(getenv("VAR60")),
-5.3499015024446672 + atof(getenv("VAR61")),
-2.1856460276046694 + atof(getenv("VAR62")),
0.64980649675188129 + atof(getenv("VAR63")),
-2.5539723792193043 + atof(getenv("VAR64")),
-3.482020318358114 + atof(getenv("VAR65")),
1.9259220078571748 + atof(getenv("VAR66")),
3.3881907588125211 + atof(getenv("VAR67")),
27.924193042501866 + atof(getenv("VAR68")),
1.0340036218354931 + atof(getenv("VAR69")),
1.7075887635054423 + atof(getenv("VAR70")),
1.0094030888728891 + atof(getenv("VAR71")),
15.350270052578821 + atof(getenv("VAR72")),
7.2491661823604501 + atof(getenv("VAR73")),
-6.3383761457802912 + atof(getenv("VAR74")),
-1.6899756729558566 + atof(getenv("VAR75")),
3.8767981287555195 + atof(getenv("VAR76")),
34.134190887895159 + atof(getenv("VAR77")),
26.983992082088804 + atof(getenv("VAR78")),
7.8253320192101423 + atof(getenv("VAR79")),
13.426108189589538 + atof(getenv("VAR80")),
-6.3430491110492966 + atof(getenv("VAR81")),
17.962291695913756 + atof(getenv("VAR82")),
-3.2398067819145693 + atof(getenv("VAR83")),
8.0389015057243043 + atof(getenv("VAR84")),
5.6507852171798039 + atof(getenv("VAR85")),
6.6640003090809525 + atof(getenv("VAR86")),
0.085073315643286393 + atof(getenv("VAR87")),
-19.999149317906035 + atof(getenv("VAR88")),
-1.5045943186667661 + atof(getenv("VAR89")),
-3.4718677896213901 + atof(getenv("VAR90")),
7.219156015612981 + atof(getenv("VAR91")),
-14.700000891659736 + atof(getenv("VAR92")),
-19.717780411961517 + atof(getenv("VAR93")),
11.720567962714041 + atof(getenv("VAR94")),
-15.798038028039029 + atof(getenv("VAR95")),
-5.2470266605815796 + atof(getenv("VAR96")),
10.850384051231268 + atof(getenv("VAR97")),
3.0442673308088586 + atof(getenv("VAR98")),
-17.014483530467167 + atof(getenv("VAR99")),
-13.500201211050607 + atof(getenv("VAR100")),
-7.8361528592799745 + atof(getenv("VAR101")),
0.5745089713610555 + atof(getenv("VAR102")),
-5.8335728595744243 + atof(getenv("VAR103")),
-1.777968417592332 + atof(getenv("VAR104")),
0.45314332610764829 + atof(getenv("VAR105")),
-0.94176168219302159 + atof(getenv("VAR106")),
8.6736791536208031 + atof(getenv("VAR107")),
-0.38089746135396413 + atof(getenv("VAR108")),
0.34126631832099269 + atof(getenv("VAR109")),
12.689574723274529 + atof(getenv("VAR110")),
-9.2031158773979254 + atof(getenv("VAR111")),
-3.8200687527770376 + atof(getenv("VAR112")),
0.31436280514392323 + atof(getenv("VAR113")),
0.91605165723822712 + atof(getenv("VAR114")),
-18.908921726335119 + atof(getenv("VAR115")),
3.6776741263611203 + atof(getenv("VAR116")),
-11.06672903446839 + atof(getenv("VAR117")),
12.130151422257573 + atof(getenv("VAR118")),
-3.5086263528413739 + atof(getenv("VAR119")),
-5.8380789349583182 + atof(getenv("VAR120")),
-9.1848584561327602 + atof(getenv("VAR121")),
-2.0891996842884319 + atof(getenv("VAR122")),
7.9970029138625076 + atof(getenv("VAR123")),
-12.420296937706622 + atof(getenv("VAR124")),
-5.1652840399227795 + atof(getenv("VAR125")),
-5.6784499377339097 + atof(getenv("VAR126")),
-5.4886466118851809 + atof(getenv("VAR127")),
-2.1601698465693602 + atof(getenv("VAR128")),
4.0380088538864216 + atof(getenv("VAR129")),
-12.75830520526411 + atof(getenv("VAR130")),
-33.183197155665653 + atof(getenv("VAR131")),
15.226255843537663 + atof(getenv("VAR132")),
13.404552543171901 + atof(getenv("VAR133")),
-32.132803591828861 + atof(getenv("VAR134")),
4.568628139024554 + atof(getenv("VAR135")),
11.358970604483108 + atof(getenv("VAR136")),
11.815205604849965 + atof(getenv("VAR137")),
4.2770087113312751 + atof(getenv("VAR138")),
28.427123240980286 + atof(getenv("VAR139")),
-3.9110033828110384 + atof(getenv("VAR140")),
6.0851605780085807 + atof(getenv("VAR141")),
5.8003801142648328 + atof(getenv("VAR142")),
-27.636109912600961 + atof(getenv("VAR143")),
-10.829786592759763 + atof(getenv("VAR144")),
12.444268009414682 + atof(getenv("VAR145")),
10.000450609318021 + atof(getenv("VAR146")),
14.155169246727629 + atof(getenv("VAR147")),
0.71802095391522192 + atof(getenv("VAR148")),
4.6128475574390215 + atof(getenv("VAR149")),
-10.36068287996104 + atof(getenv("VAR150")),
4.9504799426763393 + atof(getenv("VAR151")),
-0.43123550722846243 + atof(getenv("VAR152")),
-5.9105176659733889 + atof(getenv("VAR153")),
8.9512523023253259 + atof(getenv("VAR154")),
11.416862592510839 + atof(getenv("VAR155")),
4.0685557431848478 + atof(getenv("VAR156")),
9.4513295472881058 + atof(getenv("VAR157")),
3.709416090274301 + atof(getenv("VAR158")),
-47.32362548077694 + atof(getenv("VAR159")),
9.7641101930225549 + atof(getenv("VAR160")),
-5.9870424921864087 + atof(getenv("VAR161")),
-8.5127274285720809 + atof(getenv("VAR162")),
-2.408062325644619 + atof(getenv("VAR163")),
-1.4965566887113047 + atof(getenv("VAR164")),
-1.8090902900975607 + atof(getenv("VAR165")),
-20.468082773566433 + atof(getenv("VAR166")),
-1.8880895180411581 + atof(getenv("VAR167")),
-1.5902984185306615 + atof(getenv("VAR168")),
5.2599341370252413 + atof(getenv("VAR169")),
4.6301756620907391 + atof(getenv("VAR170")),
8.1977296035598357 + atof(getenv("VAR171")),
-6.7335319246187142 + atof(getenv("VAR172")),
4.9232373622827525 + atof(getenv("VAR173")),
-9.8750862377676949 + atof(getenv("VAR174")),
12.694094247652227 + atof(getenv("VAR175")),
5.320497330435658 + atof(getenv("VAR176")),
-14.776342515764888 + atof(getenv("VAR177")),
24.880239430999808 + atof(getenv("VAR178")),
-1.913681448374992 + atof(getenv("VAR179")),
-19.25510079090007 + atof(getenv("VAR180")),
-0.87722412151442442 + atof(getenv("VAR181")),
-8.1716269961323906 + atof(getenv("VAR182")),
10.844711752012246 + atof(getenv("VAR183")),
-20.83731312282735 + atof(getenv("VAR184")),
-0.99050853466874589 + atof(getenv("VAR185")),
-1.0607641286645715 + atof(getenv("VAR186")),
-1.6771012442149045 + atof(getenv("VAR187")),
-13.564461564675527 + atof(getenv("VAR188")),
-13.986248087830155 + atof(getenv("VAR189")),
12.980819520620468 + atof(getenv("VAR190")),
2.4491915765727805 + atof(getenv("VAR191")),
-4.4835847861502076 + atof(getenv("VAR192")),
-7.1103431907928432 + atof(getenv("VAR193")),
-30.411776184248939 + atof(getenv("VAR194")),
2.4880930204697216 + atof(getenv("VAR195")),
-5.5564350297161313 + atof(getenv("VAR196")),
8.7210102835483241 + atof(getenv("VAR197")),
7.833757263046091 + atof(getenv("VAR198")),
5.4711389342847392 + atof(getenv("VAR199")),
20.178238361136749 + atof(getenv("VAR200")),
-4.5514540698786936 + atof(getenv("VAR201")),
7.3077766636082115 + atof(getenv("VAR202")),
0.49064323555160838 + atof(getenv("VAR203")),
5.0549131814927577 + atof(getenv("VAR204")),
-2.544574864923892 + atof(getenv("VAR205")),
19.687418920653052 + atof(getenv("VAR206")),
-18.899197743952303 + atof(getenv("VAR207")),
6.3565289338853752 + atof(getenv("VAR208")),
-6.7259507186014558 + atof(getenv("VAR209")),
-4.6013146433660692 + atof(getenv("VAR210")),
17.676822900962975 + atof(getenv("VAR211")),
-0.9936481153259854 + atof(getenv("VAR212")),
-9.9872384605199986 + atof(getenv("VAR213")),
19.491828713868802 + atof(getenv("VAR214")),
9.1974138530439689 + atof(getenv("VAR215")),
0.74587483537048949 + atof(getenv("VAR216")),
-1.8874068370620789 + atof(getenv("VAR217")),
-64.009612235959736 + atof(getenv("VAR218")),
3.0161999766034051 + atof(getenv("VAR219")),
0.65664558890268365 + atof(getenv("VAR220")),
-11.997062302140973 + atof(getenv("VAR221")),
-11.155913628380047 + atof(getenv("VAR222")),
-2.7853318511805241 + atof(getenv("VAR223")),
-10.596564036656195 + atof(getenv("VAR224")),
2.5385748079997672 + atof(getenv("VAR225")),
1.1889309355508775 + atof(getenv("VAR226")),
-7.3086348450496859 + atof(getenv("VAR227")),
0.4099800627087305 + atof(getenv("VAR228")),
-3.8067861794073123 + atof(getenv("VAR229")),
-1.929993432298359 + atof(getenv("VAR230")),
18.978433757184899 + atof(getenv("VAR231")),
1.9957780048382978 + atof(getenv("VAR232")),
10.322038556534281 + atof(getenv("VAR233")),
12.070786554076582 + atof(getenv("VAR234")),
1.2209193958815596 + atof(getenv("VAR235")),
-4.2672890296415007 + atof(getenv("VAR236")),
-4.0329802701908957 + atof(getenv("VAR237")),
-9.2878571637849241 + atof(getenv("VAR238")),
1.256408781676418 + atof(getenv("VAR239")),
7.4330642120720549 + atof(getenv("VAR240")),
15.99858854928339 + atof(getenv("VAR241")),
-1.5277338943187142 + atof(getenv("VAR242")),
-2.9232897957579826 + atof(getenv("VAR243")),
8.1325426149500863 + atof(getenv("VAR244")),
4.6637889521918963 + atof(getenv("VAR245")),
5.6080597434749775 + atof(getenv("VAR246")),
-14.047327186693465 + atof(getenv("VAR247")),
13.288356937528597 + atof(getenv("VAR248")),
5.7571215992350542 + atof(getenv("VAR249")),
-9.3536426400481343 + atof(getenv("VAR250")),
-9.8018331002856272 + atof(getenv("VAR251")),
7.375190444860042 + atof(getenv("VAR252")),
-1.1382827846169852 + atof(getenv("VAR253")),
1.1670641351888982 + atof(getenv("VAR254")),
0.78452436665612624 + atof(getenv("VAR255")),
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
    return; // do nothing
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
	    static const double kL2Mul = 62.389288452518187 + atof(getenv("VAR257"));
	    double diff = kL2Mul * sum_out_c - sum_in_c;
	    L2_diff_all_channels += diff * diff;
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
  L2_diff_all_channels *= 1.0 + 0.001 * g_entropy;
  printf("hipsu: %.17f\n", g_entropy);
  printf("Energy: %.17f\n", L2_diff_all_channels);
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
