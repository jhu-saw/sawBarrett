#
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set(
  REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical )

find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if( cisst_FOUND  )

  # load cisst configuration
  include( ${CISST_USE_FILE} )
  
  cisst_find_saw_component(sawCANBus REQUIRED)
  cisst_find_saw_component(sawBarrett REQUIRED)
  
  if(sawCANBus_FOUND  AND sawBarrett_FOUND)
    
    include_directories(
      ${sawCANBus_INCLUDE_DIR}
      ${sawBarrett_INCLUDE_DIR} )

    link_directories(${sawCANBus_LIBRARY_DIR})

    set( REQUIRED_SAW_COMPONENTS sawBarrett sawCANBus )

    add_executable( osaPuckExample    osaPuckExample.cpp )
    add_executable( osaGroupExample   osaGroupExample.cpp )
    add_executable( osaWAMExample     osaWAMExample.cpp )
    add_executable( osaBH8_280Example osaBH8_280Example.cpp )

    add_executable( mtsPuckExample mtsPuckExample.cpp )
    add_executable( mtsWAMExample  mtsWAMExample.cpp )

    target_link_libraries( osaPuckExample    ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
    target_link_libraries( osaGroupExample   ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
    target_link_libraries( osaWAMExample     ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
    target_link_libraries( osaBH8_280Example ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
    target_link_libraries( mtsPuckExample    ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
    target_link_libraries( mtsWAMExample     ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )

    cisst_find_saw_component( sawControllers )
    cisst_find_saw_component( sawTrajectories )
    cisst_find_saw_component( sawKeyboard )

    if( sawControllers_FOUND AND
        sawKeyboard_FOUND    AND
        sawTrajectories_FOUND )

      include_directories(
        ${sawKeyboard_INCLUDE_DIR}
        ${sawControllers_INCLUDE_DIR}
        ${sawTrajectories_INCLUDE_DIR} )
      
      link_directories( 
        ${sawControllers_LIBRARY_DIR} 
        ${sawKeyboard_LIBRARY_DIR}
        ${sawTrajectories_LIBRARY_DIR} )
      
      set(
        REQUIRED_SAW_COMPONENTS
        ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES}
        sawControllers
        sawTrajectories
        sawKeyboard )
      
      if (sawCAN_HAS_RTSocketCAN)

        set(sawBarrett_EXAMPLES
          osaWAMGCExample
          osaWAMPDGCExample
          mtsWAMGCExample
          mtsWAMPDGCExample
          mtsWAMMoveExample
          mtsWAMServer
          )

        foreach (_example ${sawBarrett_EXAMPLES})
          add_executable( ${_example} ${_example}.cpp )
          target_link_libraries( ${_example} ${REQUIRED_SAW_COMPONENTS} )
          cisst_target_link_libraries(${_example} ${REQUIRED_CISST_LIBRARIES})
        endforeach ()
        
      endif (sawCAN_HAS_RTSocketCAN)
      
    endif( 
      sawControllers_FOUND AND 
      sawKeyboard_FOUND    AND 
      sawTrajectories_FOUND )
    
  endif( sawCANBus_FOUND  AND sawBarrett_FOUND )
  
endif( cisst_FOUND )
