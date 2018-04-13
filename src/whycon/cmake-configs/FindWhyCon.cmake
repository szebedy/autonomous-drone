find_path(WhyCon_INCLUDE_DIRS whycon/localization_system.h PATH_SUFFIXES whycon HINTS ${CMAKE_CURRENT_LIST_DIR}/../../../include)
find_library(WhyCon_LIBRARIES NAMES whycon HINTS ${CMAKE_CURRENT_LIST_DIR}/../../../lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(whycon DEFAULT_MSG WhyCon_LIBRARIES WhyCon_INCLUDE_DIRS)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS program_options thread system REQUIRED)

set(WhyCon_LIBRARIES ${WHYCON_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
set(WhyCon_INCLUDE_DIRS ${WHYCON_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

mark_as_advanced(WhyCon_LIBRARIES WhyCon_INCLUDE_DIRS)
