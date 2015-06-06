# - Find wiringPi

# Look for the header file.
find_path(wiringPi_INCLUDE NAMES wiringPi.h
  PATHS $ENV{WIRINGPI_ROOT}/include /usr /usr/local /usr/share)

find_library(wiringPi_LIBRARY NAMES wiringPi
  PATHS $ENV{WIRINGPI_ROOT}/lib /usr/lib /usr/local/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(wiringPi DEFAULT_MSG wiringPi_INCLUDE wiringPi_LIBRARY)

# if(wiringPi_FOUND)
#  message(STATUS "Found wiringPi (include: ${wiringPi_INCLUDE}, ${wiringPi_LIBRARY})")
#  set(wiringPi_INCLUDES ${wiringPi_INCLUDE})
#  set(wiringPi_LIBRARIES ${wiringPi_LIBRARY})
#  mark_as_advanced(wiringPi_INCLUDE wiringPi_LIBRARY)
#endif()
