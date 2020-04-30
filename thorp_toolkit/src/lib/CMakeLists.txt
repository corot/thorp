##############################################################################
# Library
##############################################################################

add_library(${PROJECT_NAME}
    tf.cpp
    planning_scene.cpp
)
add_dependencies(${PROJECT_NAME} thorp_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

##############################################################################
# Installs
##############################################################################

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
 
