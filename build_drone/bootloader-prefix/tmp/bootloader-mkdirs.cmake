# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/yasmine/Projects/embedded/esp-idf/components/bootloader/subproject"
  "/home/yasmine/Projects/quadcopter/drone/build_drone/bootloader"
  "/home/yasmine/Projects/quadcopter/drone/build_drone/bootloader-prefix"
  "/home/yasmine/Projects/quadcopter/drone/build_drone/bootloader-prefix/tmp"
  "/home/yasmine/Projects/quadcopter/drone/build_drone/bootloader-prefix/src/bootloader-stamp"
  "/home/yasmine/Projects/quadcopter/drone/build_drone/bootloader-prefix/src"
  "/home/yasmine/Projects/quadcopter/drone/build_drone/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/yasmine/Projects/quadcopter/drone/build_drone/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/yasmine/Projects/quadcopter/drone/build_drone/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
