
set( 
  CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH} 
  ${CMAKE_CURRENT_SOURCE_DIR}/code/cmake/Modules
  )

add_subdirectory( code )
add_subdirectory( examples )


if (CISST_HAS_QT)
  add_subdirectory( codeQt )
endif (CISST_HAS_QT)
