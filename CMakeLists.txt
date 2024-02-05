cmake_minimum_required(VERSION 3.10)

project(Tabuli CXX)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SndFile REQUIRED IMPORTED_TARGET sndfile)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3f)
find_package(Eigen3 REQUIRED)
add_subdirectory(third_party/absl)

add_executable(driver_model driver_model/driver_model.cc)
target_link_libraries(driver_model PkgConfig::SndFile)

foreach (experiment IN ITEMS angular emphasizer revolve spectrum_similarity two_to_three virtual_speakers)
  add_executable(${experiment} speaker_experiments/${experiment}.cc)
  target_link_libraries(${experiment} PkgConfig::SndFile absl::flags absl::flags_parse absl::log absl::log_internal_check_impl)
endforeach ()

target_link_libraries(angular PkgConfig::FFTW3)
target_link_libraries(spectrum_similarity PkgConfig::FFTW3)
target_link_libraries(two_to_three PkgConfig::FFTW3)

target_link_libraries(virtual_speakers Eigen3::Eigen)
