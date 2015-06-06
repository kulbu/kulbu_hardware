# - Find wiringPi
#
#  wiringpi_INCLUDES  - List of wiringPi includes
#  wiringpi_FOUND     - True if wiringPi found.

# Look for the header file.
find_path(wiringPi_INCLUDE NAMES wiringPi.h
  PATHS $ENV{WIRINGPI_ROOT} /usr /usr/local /usr/share)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(wiringPi DEFAULT_MSG wiringPi_INCLUDE)

if(wiringPi_FOUND)
  message(STATUS "Found wiringPi (include: ${wiringPi_INCLUDE})")
  set(wiringPi_INCLUDES ${wiringPi_INCLUDE})
  mark_as_advanced(wiringPi_INCLUDE)
endif()
