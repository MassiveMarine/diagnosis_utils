# This calls tug_cfg scripts to generate header files for given configuration
# definitions:
macro(generate_cfg)
  if(${PROJECT_NAME}_CATKIN_PACKAGE)
    message(FATAL_ERROR "${PROJECT_NAME}: generate_cfg() called after catkin_package()")
  endif()

  # Make sure package destination variables are defined:
  catkin_destinations()
  
  foreach(_arg ${ARGN})
    set(_cfg_file_path ${PROJECT_SOURCE_DIR}/${_arg})
    get_filename_component(_cfg_name ${_cfg_file_path} NAME_WE)
    set(_header_file_path ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${_cfg_name}.h)
    
    add_custom_command(
      COMMENT "Generating ${_cfg_name}.h"
      OUTPUT ${_header_file_path}
      DEPENDS ${_cfg_file_path}
      COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      COMMAND rosrun tug_cfg gencfg.py ${PROJECT_NAME} ${_cfg_file_path} > ${_header_file_path}
    )
    
    install(FILES ${_header_file_path} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

    list(APPEND ${PROJECT_NAME}_generated_cfgs ${_header_file_path})
  endforeach(_arg)
  
  add_custom_target(${PROJECT_NAME}_generate_cfg ALL
    DEPENDS ${${PROJECT_NAME}_generated_cfgs}
  )
  
  list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_cfg)
endmacro()

