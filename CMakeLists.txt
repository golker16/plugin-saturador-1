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
# MSVC runtime estático (/MT)
# Evita depender del Visual C++ Redistributable en la PC del usuario.
# -----------------------------
if (MSVC)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# -----------------------------
# Plugin Target (VST3)
# JUCE ya incluye headers del VST3 SDK internamente.
# -----------------------------
juce_add_plugin(PluginSaturador1
  COMPANY_NAME "golk"
  PRODUCT_NAME "Plugin Saturador 1"

  # 4-char codes (mejor en mayúsculas)
  PLUGIN_MANUFACTURER_CODE GOLK
  PLUGIN_CODE SAT1

  FORMATS VST3

  # Categorías explícitas (ayuda a algunos hosts)
  VST3_CATEGORIES "Fx" "Distortion"

  # (Si estabas usando esto para evitar problemas de hosts, lo dejamos)
  VST3_AUTO_MANIFEST FALSE

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

  # Importante: evita que JUCE intente headers de VST2
  JUCE_VST3_CAN_REPLACE_VST2=0
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

