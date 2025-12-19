cmake_minimum_required(VERSION 3.15)

project(PluginSaturador1 VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# -----------------------------
# Fetch JUCE
# -----------------------------
FetchContent_Declare(
  juce
  GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
  GIT_TAG 8.0.10
)
FetchContent_MakeAvailable(juce)

# -----------------------------
# Fetch VST3 SDK (public)
# -----------------------------
FetchContent_Declare(
  vst3sdk
  GIT_REPOSITORY https://github.com/steinbergmedia/vst3_public_sdk.git
  GIT_TAG master
)
FetchContent_MakeAvailable(vst3sdk)

# -----------------------------
# Find correct VST3 SDK include root for JUCE
# JUCE expects to be able to include:
#   <pluginterfaces/base/fplatform.h>
# So the SDK path must be the directory that CONTAINS the "pluginterfaces" folder.
# This searches robustly regardless of repo layout.
# -----------------------------
file(GLOB_RECURSE VST3_FPLATFORM
     "${vst3sdk_SOURCE_DIR}/*/pluginterfaces/base/fplatform.h"
     "${vst3sdk_SOURCE_DIR}/pluginterfaces/base/fplatform.h")

if (NOT VST3_FPLATFORM)
  message(FATAL_ERROR
    "VST3 SDK not found: could not locate pluginterfaces/base/fplatform.h inside ${vst3sdk_SOURCE_DIR}.")
endif()

list(GET VST3_FPLATFORM 0 VST3_FPLATFORM_FILE)

# fplatform.h -> .../pluginterfaces/base
get_filename_component(_vst3_base_dir "${VST3_FPLATFORM_FILE}" DIRECTORY)
# .../pluginterfaces
get_filename_component(_vst3_pluginterfaces_dir "${_vst3_base_dir}" DIRECTORY)
# parent of pluginterfaces (THIS is the include root)
get_filename_component(VST3_SDK_PATH "${_vst3_pluginterfaces_dir}" DIRECTORY)

message(STATUS "Found fplatform.h at: ${VST3_FPLATFORM_FILE}")
message(STATUS "Using VST3 SDK path for JUCE: ${VST3_SDK_PATH}")

juce_set_vst3_sdk_path("${VST3_SDK_PATH}")

# -----------------------------
# Plugin Target (VST3 only)
# -----------------------------
juce_add_plugin(PluginSaturador1
  COMPANY_NAME "golk"
  PRODUCT_NAME "Plugin Saturador 1"

  PLUGIN_MANUFACTURER_CODE GoLK
  PLUGIN_CODE SaT1

  FORMATS VST3

  IS_SYNTH FALSE
  IS_MIDI_EFFECT FALSE
  NEEDS_MIDI_INPUT FALSE
  NEEDS_MIDI_OUTPUT FALSE

  COPY_PLUGIN_AFTER_BUILD FALSE
)

# Genera JuceHeader.h para que #include <JuceHeader.h> funcione
juce_generate_juce_header(PluginSaturador1)

target_sources(PluginSaturador1 PRIVATE
  PluginProcessor.cpp
  PluginProcessor.h
  LevelMatcher.h
  Saturation1073ish.h
)

target_compile_definitions(PluginSaturador1 PRIVATE
  JUCE_WEB_BROWSER=0
  JUCE_USE_CURL=0
)

target_link_libraries(PluginSaturador1 PRIVATE
  juce::juce_audio_processors
  juce::juce_dsp
  juce::juce_gui_basics
  juce::juce_gui_extra

  juce::juce_recommended_config_flags
  juce::juce_recommended_warning_flags
  juce::juce_recommended_lto_flags
)

