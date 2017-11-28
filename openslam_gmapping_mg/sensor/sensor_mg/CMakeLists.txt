add_library(sensor_mg mgreading.cpp mgsensor.cpp)
target_link_libraries(sensor_mg sensor_base)

install(TARGETS sensor_mg DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
