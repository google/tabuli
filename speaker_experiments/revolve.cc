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
  // [22 * 10**(3 * i / 240.) for i in range(-16, 242)]
  static const float kFreq[258] = {
    13.881061578564251, 14.286395894676648, 14.703566186509523, 15.132918070793767, 15.574807256451034, 16.02959983929101, 16.49767260531403, 16.979413342870274, 17.475221163934194, 17.9855068347603, 18.510693116194293, 19.051215113921437, 19.607520638942404, 20.18007057857515, 20.769339278290314, 21.375814934696336, 22.0, 22.64241159827742, 23.303581953900355, 23.984058832472808, 24.684405994643193, 25.40520366316808, 26.147049003614406, 26.910556619098497, 27.696359059471682, 28.505107345374427, 29.33747150759313, 30.194141142166362, 31.075825981700596, 31.98325648336856, 32.917184434077534, 33.878383573308824, 34.8676502341445, 35.88580400301244, 36.933688398696326, 38.01217157117299, 39.1221470208563, 40.264534338843234, 41.44027996877561, 42.65035799094856, 43.89577092931535, 45.17755058205722, 46.49675887640623, 47.854488748429496, 49.25186504850347, 50.69004547322861, 52.17022152455641, 53.69361949692357, 55.261501493210766, 56.87516647036819, 58.53595131557381, 60.24523195381595, 62.004424487817985, 63.81498637124939, 65.6784176161951, 67.59626203588383, 69.57010852370435, 71.60159236957008, 73.69239661472209, 75.84425344609286, 78.0589456313866, 80.3383079960643, 82.68422894345771, 85.09865201927148, 87.58357752176939, 90.14106415897913, 92.7732307542881, 95.48225800184363, 98.2703902732119, 101.13993747679187, 104.09327697152571, 107.13285553648987, 110.26119139799988, 113.48087631590725, 116.79457773081744, 120.20504097400794, 123.7150915418768, 127.32763743680653, 131.0456715763823, 134.87227427296057, 138.81061578564254, 142.8639589467665, 147.0356618650952, 151.32918070793767, 155.74807256451032, 160.29599839291012, 164.97672605314028, 169.7941334287027, 174.75221163934194, 179.85506834760298, 185.10693116194292, 190.51215113921438, 196.07520638942398, 201.80070578575152, 207.69339278290312, 213.7581493469634, 220.0, 226.42411598277417, 233.0358195390035, 239.84058832472812, 246.844059946432, 254.0520366316808, 261.47049003614404, 269.1055661909849, 276.96359059471683, 285.0510734537443, 293.37471507593125, 301.9414114216636, 310.75825981700586, 319.8325648336857, 329.1718443407754, 338.7838357330882, 348.6765023414449, 358.85804003012436, 369.33688398696336, 380.12171571173, 391.221470208563, 402.64534338843225, 414.40279968775604, 426.50357990948567, 438.9577092931535, 451.7755058205721, 464.96758876406227, 478.5448874842949, 492.5186504850348, 506.9004547322861, 521.7022152455642, 536.9361949692357, 552.6150149321074, 568.7516647036821, 585.3595131557381, 602.4523195381595, 620.0442448781797, 638.1498637124937, 656.7841761619512, 675.9626203588383, 695.7010852370435, 716.0159236957008, 736.9239661472207, 758.4425344609288, 780.5894563138661, 803.383079960643, 826.8422894345771, 850.9865201927147, 875.8357752176942, 901.4106415897915, 927.732307542881, 954.8225800184363, 982.7039027321186, 1011.3993747679191, 1040.932769715257, 1071.3285553648989, 1102.611913979999, 1134.8087631590722, 1167.9457773081747, 1202.0504097400794, 1237.150915418768, 1273.2763743680653, 1310.4567157638228, 1348.722742729606, 1388.1061578564252, 1428.639589467665, 1470.356618650952, 1513.2918070793762, 1557.4807256451038, 1602.959983929101, 1649.7672605314028, 1697.941334287027, 1747.522116393419, 1798.5506834760304, 1851.0693116194293, 1905.1215113921437, 1960.75206389424, 2018.0070578575146, 2076.9339278290317, 2137.581493469634, 2200.0, 2264.241159827743, 2330.358195390035, 2398.4058832472815, 2468.4405994643184, 2540.520366316808, 2614.7049003614416, 2691.055661909849, 2769.635905947169, 2850.5107345374417, 2933.7471507593127, 3019.4141142166377, 3107.5825981700586, 3198.325648336857, 3291.7184434077526, 3387.838357330882, 3486.765023414451, 3588.5804003012436, 3693.368839869633, 3801.217157117298, 3912.21470208563, 4026.453433884325, 4144.02799687756, 4265.035799094857, 4389.577092931533, 4517.755058205721, 4649.675887640625, 4785.4488748429485, 4925.186504850348, 5069.004547322858, 5217.022152455641, 5369.361949692359, 5526.150149321075, 5687.516647036821, 5853.595131557379, 6024.523195381595, 6200.4424487818005, 6381.498637124936, 6567.841761619513, 6759.62620358838, 6957.010852370435, 7160.159236957013, 7369.2396614722065, 7584.425344609288, 7805.894563138657, 8033.830799606429, 8268.422894345775, 8509.865201927145, 8758.35775217694, 9014.10641589791, 9277.32307542881, 9548.225800184367, 9827.039027321187, 10113.993747679191, 10409.327697152567, 10713.285553648988, 11026.119139799994, 11348.087631590723, 11679.457773081747, 12020.504097400788, 12371.509154187679, 12732.76374368066, 13104.567157638226, 13487.227427296059, 13881.061578564246, 14286.39589467665, 14703.566186509528, 15132.918070793763, 15574.807256451037, 16029.599839291002, 16497.67260531403, 16979.41334287028, 17475.221163934188, 17985.506834760305, 18510.693116194285, 19051.215113921437, 19607.52063894241, 20180.07057857515, 20769.339278290317, 21375.81493469633, 22000.0, 22642.411598277427
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
    static const double kMagic = 6.1260634213819953 + atof(getenv("VAR256"));
    return kMagic / window;
  }

  Rotators() {}
  Rotators(int num_channels, const float sample_rate) {
    channel.resize(num_channels);
    static const float kSampleRate = 48000.0;
    static const float kHzToRad = 2.0f * M_PI / kSampleRate;
    //    static const double kWindow = 0.9988311560491858
    static const double kWindow = 0.99949466472687154 + 0.0005 * atof(getenv("VAR257"));
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
    inputphase[0][1] = 0;
    float prev_phase = 0;
    double phase_array[256] = {
  -0.16660282878721072 + atof(getenv("VAR0")),
  -1.1862206692653188 + atof(getenv("VAR1")),
  -0.0014903974196743414 + atof(getenv("VAR2")),
  0.61900388135915707 + atof(getenv("VAR3")),
  -0.5443525032170442 + atof(getenv("VAR4")),
  0.2356054621846653 + atof(getenv("VAR5")),
  0.062365255201009592 + atof(getenv("VAR6")),
  0.80651664683761093 + atof(getenv("VAR7")),
  -0.092884458375986162 + atof(getenv("VAR8")),
  -0.89611192506076176 + atof(getenv("VAR9")),
  -1.0270788771323269 + atof(getenv("VAR10")),
  -0.23381442972095881 + atof(getenv("VAR11")),
  -0.0082869205366234919 + atof(getenv("VAR12")),
  0.60507147801702588 + atof(getenv("VAR13")),
  -1.4872814402773245 + atof(getenv("VAR14")),
  -0.16499715083314068 + atof(getenv("VAR15")),
  -0.73196334213762759 + atof(getenv("VAR16")),
  0.60006024902036625 + atof(getenv("VAR17")),
  0.22121716101510866 + atof(getenv("VAR18")),
  0.088792790800173479 + atof(getenv("VAR19")),
  -0.95688496507484577 + atof(getenv("VAR20")),
  -0.075226215733773416 + atof(getenv("VAR21")),
  0.070188883315104644 + atof(getenv("VAR22")),
  0.31621799312969784 + atof(getenv("VAR23")),
  1.2029007646261125 + atof(getenv("VAR24")),
  0.23393137960350277 + atof(getenv("VAR25")),
  0.35529507402506172 + atof(getenv("VAR26")),
  0.0039641430605089553 + atof(getenv("VAR27")),
  -0.57986122594569145 + atof(getenv("VAR28")),
  0.74148259751464907 + atof(getenv("VAR29")),
  -0.32017324042301504 + atof(getenv("VAR30")),
  0.060609104425777729 + atof(getenv("VAR31")),
  -0.2071725038346782 + atof(getenv("VAR32")),
  -0.074723601672028794 + atof(getenv("VAR33")),
  0.071942584731643305 + atof(getenv("VAR34")),
  0.45267040232550848 + atof(getenv("VAR35")),
  -0.3903695293823663 + atof(getenv("VAR36")),
  -0.54686055461114214 + atof(getenv("VAR37")),
  0.75083511130103575 + atof(getenv("VAR38")),
  -0.35958925034498895 + atof(getenv("VAR39")),
  -1.1435173051291501 + atof(getenv("VAR40")),
  -0.21214064536482116 + atof(getenv("VAR41")),
  -0.24092154636003349 + atof(getenv("VAR42")),
  0.65640837563430599 + atof(getenv("VAR43")),
  -0.17508356476983167 + atof(getenv("VAR44")),
  0.77802377453915006 + atof(getenv("VAR45")),
  0.54544454729501002 + atof(getenv("VAR46")),
  -0.57779797022404533 + atof(getenv("VAR47")),
  0.21040280057241492 + atof(getenv("VAR48")),
  0.2431468815597011 + atof(getenv("VAR49")),
  -0.9335104678648648 + atof(getenv("VAR50")),
  -0.18202600611877398 + atof(getenv("VAR51")),
  0.52579201614837179 + atof(getenv("VAR52")),
  0.2881141676790121 + atof(getenv("VAR53")),
  0.59178396180181247 + atof(getenv("VAR54")),
  -0.034689423960526197 + atof(getenv("VAR55")),
  -0.88209772506530704 + atof(getenv("VAR56")),
  0.18644757685903068 + atof(getenv("VAR57")),
  0.32296095437118699 + atof(getenv("VAR58")),
  0.50394008105548016 + atof(getenv("VAR59")),
  -0.083196009396009948 + atof(getenv("VAR60")),
  0.52750482286247968 + atof(getenv("VAR61")),
  0.34869847870938053 + atof(getenv("VAR62")),
  -0.43174760595768008 + atof(getenv("VAR63")),
  0.38791725086695433 + atof(getenv("VAR64")),
  -0.20825273508760994 + atof(getenv("VAR65")),
  -0.33762372134526841 + atof(getenv("VAR66")),
  -0.042327749357187375 + atof(getenv("VAR67")),
  -0.13654793969295881 + atof(getenv("VAR68")),
  -0.48411179177797231 + atof(getenv("VAR69")),
  -0.45834183638014514 + atof(getenv("VAR70")),
  -0.27295952657421446 + atof(getenv("VAR71")),
  -0.2858983191223548 + atof(getenv("VAR72")),
  0.54260115864470781 + atof(getenv("VAR73")),
  -0.40745966745548962 + atof(getenv("VAR74")),
  -0.85395284345145306 + atof(getenv("VAR75")),
  0.23526303384112163 + atof(getenv("VAR76")),
  -0.30077731434418808 + atof(getenv("VAR77")),
  -0.62737480581402549 + atof(getenv("VAR78")),
  -0.0060175367995687293 + atof(getenv("VAR79")),
  0.26598440706957699 + atof(getenv("VAR80")),
  -0.24288531224972862 + atof(getenv("VAR81")),
  -0.19011497158035037 + atof(getenv("VAR82")),
  0.66389167742143451 + atof(getenv("VAR83")),
  1.0783541974535584 + atof(getenv("VAR84")),
  0.22385466127476089 + atof(getenv("VAR85")),
  -0.072078739796789401 + atof(getenv("VAR86")),
  -0.012470133702016502 + atof(getenv("VAR87")),
  -0.99166130149248899 + atof(getenv("VAR88")),
  0.98179578496273545 + atof(getenv("VAR89")),
  0.35209846957492968 + atof(getenv("VAR90")),
  -0.17645010170774805 + atof(getenv("VAR91")),
  0.12544947953952623 + atof(getenv("VAR92")),
  0.2743220774530315 + atof(getenv("VAR93")),
  -0.03090746099089553 + atof(getenv("VAR94")),
  0.20916488588054333 + atof(getenv("VAR95")),
  -0.78068907200950277 + atof(getenv("VAR96")),
  0.20565596563321561 + atof(getenv("VAR97")),
  0.27454171525952115 + atof(getenv("VAR98")),
  0.20223194693378765 + atof(getenv("VAR99")),
  -0.4596969198438321 + atof(getenv("VAR100")),
  0.40316940536530582 + atof(getenv("VAR101")),
  0.37390226096273455 + atof(getenv("VAR102")),
  -0.052385488909331597 + atof(getenv("VAR103")),
  -0.30814024025519754 + atof(getenv("VAR104")),
  -0.21270071863597029 + atof(getenv("VAR105")),
  0.25747275278392046 + atof(getenv("VAR106")),
  -0.53488288224091107 + atof(getenv("VAR107")),
  -0.63226614961009786 + atof(getenv("VAR108")),
  0.013387243835965502 + atof(getenv("VAR109")),
  0.27523038280650086 + atof(getenv("VAR110")),
  0.49367872530440188 + atof(getenv("VAR111")),
  -0.55578074436992186 + atof(getenv("VAR112")),
  -0.56348513247300147 + atof(getenv("VAR113")),
  0.32716326987854494 + atof(getenv("VAR114")),
  1.3938530191756064 + atof(getenv("VAR115")),
  0.35451654922640441 + atof(getenv("VAR116")),
  0.3875641827933507 + atof(getenv("VAR117")),
  -0.77938127964355375 + atof(getenv("VAR118")),
  -0.19193297023634892 + atof(getenv("VAR119")),
  -0.35527131037569681 + atof(getenv("VAR120")),
  1.1808163480651603 + atof(getenv("VAR121")),
  -0.61867979537441042 + atof(getenv("VAR122")),
  -0.0065694456554534469 + atof(getenv("VAR123")),
  0.51436954899462695 + atof(getenv("VAR124")),
  0.17204417957117837 + atof(getenv("VAR125")),
  -0.52028967547380489 + atof(getenv("VAR126")),
  -0.75889255021618984 + atof(getenv("VAR127")),
  0.25796614152309322 + atof(getenv("VAR128")),
  -0.73690800336350804 + atof(getenv("VAR129")),
  0.29230217315222268 + atof(getenv("VAR130")),
  -0.90881885999784584 + atof(getenv("VAR131")),
  -1.4861320748939886 + atof(getenv("VAR132")),
  0.47285197817916341 + atof(getenv("VAR133")),
  -0.046538443527159426 + atof(getenv("VAR134")),
  -0.83876071943414976 + atof(getenv("VAR135")),
  0.030679291631331859 + atof(getenv("VAR136")),
  -0.40978375640775566 + atof(getenv("VAR137")),
  0.28575011351960034 + atof(getenv("VAR138")),
  0.87071780638368734 + atof(getenv("VAR139")),
  0.023523939452691257 + atof(getenv("VAR140")),
  0.07188280589685879 + atof(getenv("VAR141")),
  0.27452248054127654 + atof(getenv("VAR142")),
  -1.3916921044832826 + atof(getenv("VAR143")),
  0.84833109386500194 + atof(getenv("VAR144")),
  0.39787389410112611 + atof(getenv("VAR145")),
  -0.14038546533222621 + atof(getenv("VAR146")),
  -0.049758141876881337 + atof(getenv("VAR147")),
  -0.086567053065677885 + atof(getenv("VAR148")),
  0.47648035028416014 + atof(getenv("VAR149")),
  0.22335273840825573 + atof(getenv("VAR150")),
  -1.076608023036606 + atof(getenv("VAR151")),
  0.7130529324051168 + atof(getenv("VAR152")),
  -0.60266949435038109 + atof(getenv("VAR153")),
  0.22404258299438212 + atof(getenv("VAR154")),
  -0.15811300539946829 + atof(getenv("VAR155")),
  -0.032808228297034843 + atof(getenv("VAR156")),
  0.90622254223599674 + atof(getenv("VAR157")),
  -0.19206855302287396 + atof(getenv("VAR158")),
  0.15840058815195715 + atof(getenv("VAR159")),
  -0.11719748301661366 + atof(getenv("VAR160")),
  0.018037410123859843 + atof(getenv("VAR161")),
  0.31948512986432259 + atof(getenv("VAR162")),
  -0.65848364184062325 + atof(getenv("VAR163")),
  -0.78985447729373159 + atof(getenv("VAR164")),
  -0.042272547475396924 + atof(getenv("VAR165")),
  -0.13687445002592807 + atof(getenv("VAR166")),
  0.11040861458879452 + atof(getenv("VAR167")),
  -0.37915505610183459 + atof(getenv("VAR168")),
  -0.093139907897923688 + atof(getenv("VAR169")),
  -0.12897861925718013 + atof(getenv("VAR170")),
  1.3890929799499951 + atof(getenv("VAR171")),
  2.5191641276048862 + atof(getenv("VAR172")),
  0.99271150468170322 + atof(getenv("VAR173")),
  0.20411368763271562 + atof(getenv("VAR174")),
  -0.14036461628873642 + atof(getenv("VAR175")),
  0.031216659695113138 + atof(getenv("VAR176")),
  -0.80178218699941761 + atof(getenv("VAR177")),
  -0.1154375400436871 + atof(getenv("VAR178")),
  -0.45888419614714243 + atof(getenv("VAR179")),
  0.0064816095261247333 + atof(getenv("VAR180")),
  0.24510265068800136 + atof(getenv("VAR181")),
  -0.59098977826306298 + atof(getenv("VAR182")),
  0.035500300055316195 + atof(getenv("VAR183")),
  -0.18938305783199019 + atof(getenv("VAR184")),
  -0.29931184991839854 + atof(getenv("VAR185")),
  -0.38602628339979916 + atof(getenv("VAR186")),
  0.098153480343166621 + atof(getenv("VAR187")),
  -0.24642746341455474 + atof(getenv("VAR188")),
  0.53017263170452766 + atof(getenv("VAR189")),
  -0.69337178504148755 + atof(getenv("VAR190")),
  -0.74384336915716398 + atof(getenv("VAR191")),
  0.37465407594524497 + atof(getenv("VAR192")),
  0.11057612095488628 + atof(getenv("VAR193")),
  -1.1875154841149147 + atof(getenv("VAR194")),
  0.13094001343766526 + atof(getenv("VAR195")),
  -0.35786689204921895 + atof(getenv("VAR196")),
  0.52786822750844731 + atof(getenv("VAR197")),
  -0.34804025717598963 + atof(getenv("VAR198")),
  0.33419203624648691 + atof(getenv("VAR199")),
  -0.7540477284418049 + atof(getenv("VAR200")),
  0.19208952075495245 + atof(getenv("VAR201")),
  -0.34534501460487071 + atof(getenv("VAR202")),
  0.19367257800808402 + atof(getenv("VAR203")),
  -0.11957810405384754 + atof(getenv("VAR204")),
  -0.89251117378757561 + atof(getenv("VAR205")),
  0.60169813732879318 + atof(getenv("VAR206")),
  -0.010732150132212697 + atof(getenv("VAR207")),
  0.3085156110707844 + atof(getenv("VAR208")),
  0.015620881807069326 + atof(getenv("VAR209")),
  0.16803723278282545 + atof(getenv("VAR210")),
  -0.64157783813035507 + atof(getenv("VAR211")),
  0.23129136299338116 + atof(getenv("VAR212")),
  -0.48879353314219032 + atof(getenv("VAR213")),
  -0.43406707797940297 + atof(getenv("VAR214")),
  -0.56520465727278224 + atof(getenv("VAR215")),
  -0.29720670723788112 + atof(getenv("VAR216")),
  0.14102768572574059 + atof(getenv("VAR217")),
  0.30253508450279182 + atof(getenv("VAR218")),
  0.1551319809730014 + atof(getenv("VAR219")),
  -0.66803334789192292 + atof(getenv("VAR220")),
  0.99398778715466873 + atof(getenv("VAR221")),
  -1.7687404657496808 + atof(getenv("VAR222")),
  1.4954471553247983 + atof(getenv("VAR223")),
  -1.7626224209496271 + atof(getenv("VAR224")),
  1.0663827572047793 + atof(getenv("VAR225")),
  -0.42669948744579772 + atof(getenv("VAR226")),
  0.32308804624092491 + atof(getenv("VAR227")),
  -2.0751341402862797 + atof(getenv("VAR228")),
  1.1662943848298997 + atof(getenv("VAR229")),
  1.3185113005643199 + atof(getenv("VAR230")),
  -0.1195435427478386 + atof(getenv("VAR231")),
  0.97545746276268386 + atof(getenv("VAR232")),
  -1.453407655122402 + atof(getenv("VAR233")),
  -1.3712378391674236 + atof(getenv("VAR234")),
  -0.26791521646666555 + atof(getenv("VAR235")),
  0.65771957426226879 + atof(getenv("VAR236")),
  0.15046015397100884 + atof(getenv("VAR237")),
  0.093668183579075734 + atof(getenv("VAR238")),
  -0.55738940173175866 + atof(getenv("VAR239")),
  -0.093776402946021548 + atof(getenv("VAR240")),
  -0.97986793084792823 + atof(getenv("VAR241")),
  -0.79373084934835014 + atof(getenv("VAR242")),
  -2.4024248658740417 + atof(getenv("VAR243")),
  4.960140834584899 + atof(getenv("VAR244")),
  -1.0315156798793723 + atof(getenv("VAR245")),
  -1.124117746230221 + atof(getenv("VAR246")),
  -2.728362594627741 + atof(getenv("VAR247")),
  -1.5293810549593023 + atof(getenv("VAR248")),
  4.8597780346752932 + atof(getenv("VAR249")),
  -2.5069312053735331 + atof(getenv("VAR250")),
  -1.6360849290574833 + atof(getenv("VAR251")),
  3.6626017292099822 + atof(getenv("VAR252")),
  -2.0076647328470809 + atof(getenv("VAR253")),
  -2.0512965923241575 + atof(getenv("VAR254")),
  -2.6336860590139377 + atof(getenv("VAR255")),
    };
    for (size_t i = 1; i < kNumRotators; ++i) {
      float averfreq = sqrt(FreqAve(i - 1) * FreqAve(i));
      float phase0 = averfreq * advance[i - 1] / kSampleRate;
      float phase1 = averfreq * advance[i] / kSampleRate;
      static const double almost_one = 0.17546371595120597 + 0.001 * atof(getenv("VAR258"));
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
