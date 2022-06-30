# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/GreenHouseAR_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/GreenHouseAR_autogen.dir/ParseCache.txt"
  "GreenHouseAR_autogen"
  )
endif()
