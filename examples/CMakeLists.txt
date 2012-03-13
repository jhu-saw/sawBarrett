#
# $Id$
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

    if( sawControllers_FOUND AND sawKeyboard_FOUND AND sawTrajectories_FOUND )
      link_directories(${sawControllers_LIBRARY_DIR} ${sawKeyboard_LIBRARY_DIR} ${sawTrajectories_LIBRARY_DIR} )

      set(
        REQUIRED_SAW_COMPONENTS
        ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES}
        sawControllers
        sawTrajectories
        sawKeyboard )

      add_executable( osaWAMGCExample     osaWAMGCExample.cpp )
      add_executable( osaWAMPDGCExample   osaWAMPDGCExample.cpp )
      add_executable( mtsWAMGCExample     mtsWAMGCExample.cpp )
      add_executable( mtsWAMPDGCExample   mtsWAMPDGCExample.cpp )
      add_executable( mtsWAMMoveExample   mtsWAMMoveExample.cpp )
      add_executable( mtsWAMServer        mtsWAMServer.cpp )

      target_link_libraries( osaWAMGCExample   ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
      target_link_libraries( osaWAMPDGCExample ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
      target_link_libraries( mtsWAMGCExample   ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
      target_link_libraries( mtsWAMPDGCExample ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
      target_link_libraries( mtsWAMMoveExample ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )
      target_link_libraries( mtsWAMServer      ${REQUIRED_SAW_COMPONENTS} ${REQUIRED_CISST_LIBRARIES} )

    endif( sawControllers_FOUND AND sawKeyboard_FOUND AND sawTrajectories_FOUND )
  endif(sawBarrett_FOUND and sawCANBus_FOUND)

endif( cisst_FOUND AND sawBarrett_FOUND )
