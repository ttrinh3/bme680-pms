# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/esp/esp-idf/components/bootloader/subproject"
  "C:/VisualStudio/esp32/projects/centralnode-lora-mqtt-enterprise/build/bootloader"
  "C:/VisualStudio/esp32/projects/centralnode-lora-mqtt-enterprise/build/bootloader-prefix"
  "C:/VisualStudio/esp32/projects/centralnode-lora-mqtt-enterprise/build/bootloader-prefix/tmp"
  "C:/VisualStudio/esp32/projects/centralnode-lora-mqtt-enterprise/build/bootloader-prefix/src/bootloader-stamp"
  "C:/VisualStudio/esp32/projects/centralnode-lora-mqtt-enterprise/build/bootloader-prefix/src"
  "C:/VisualStudio/esp32/projects/centralnode-lora-mqtt-enterprise/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/VisualStudio/esp32/projects/centralnode-lora-mqtt-enterprise/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
