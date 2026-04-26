cmake_minimum_required(VERSION 3.16.0)

# Force a fixed app version so ESP-IDF does not query Git metadata.
set(PROJECT_VER "1")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_Avionica)
