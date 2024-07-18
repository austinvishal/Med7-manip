set(PLUGIN_NAME ApplyForceTorque)
QT5_ADD_RESOURCES(AFT_RESOURCES_RCC ${PLUGIN_NAME}.qrc)

add_library(${PLUGIN_NAME} SHARED src/ApplyForceTorque.cc
                                  include/${PROJECT_NAME}/ApplyForceTorque.hh
                                  src/WrenchVisualizer.cc
                                  include/${PROJECT_NAME}/WrenchVisualizer.hh
                                  ${AFT_RESOURCES_RCC})
target_include_directories(${PLUGIN_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>"
                                                 "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(${PLUGIN_NAME} PRIVATE ignition-gazebo${GZ_SIM_VER}::gui)
install(TARGETS ${PLUGIN_NAME}
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include)
