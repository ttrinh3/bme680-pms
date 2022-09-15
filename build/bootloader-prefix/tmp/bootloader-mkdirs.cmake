# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Timot/esp/esp-idf/components/bootloader/subproject"
  "C:/Users/Timot/jtac/bme680-pms/build/bootloader"
  "C:/Users/Timot/jtac/bme680-pms/build/bootloader-prefix"
  "C:/Users/Timot/jtac/bme680-pms/build/bootloader-prefix/tmp"
  "C:/Users/Timot/jtac/bme680-pms/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/Timot/jtac/bme680-pms/build/bootloader-prefix/src"
  "C:/Users/Timot/jtac/bme680-pms/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Timot/jtac/bme680-pms/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
