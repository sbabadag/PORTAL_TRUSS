cmake_minimum_required(VERSION 3.21)

project(PortalSolver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Concurrent)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Concurrent)

# OpenSees: set these to your local build/install (headers + libraries).
set(OPENSEES_INCLUDE_DIR "" CACHE PATH "Directory containing Domain.h, Node.h, etc.")
set(OPENSEES_LIBRARY_DIR "" CACHE PATH "Directory containing OpenSees libraries")

add_executable(PortalSolver
    Buckling.cpp
    Buckling.h
    LectureTrussSolver.cpp
    LectureTrussSolver.h
    main.cpp
    MainWindow.cpp
    MainWindow.h
    MainWindow.ui
    OpenSeesRunner.cpp
    OpenSeesRunner.h
    PortalSolver.cpp
    PortalSolver.h
    SectionOptimizer.cpp
    SectionOptimizer.h
    SteelCatalog.cpp
    SteelCatalog.h
    TurkishLoads.cpp
    TurkishLoads.h
)

target_include_directories(PortalSolver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OPENSEES_INCLUDE_DIR}
)

target_link_libraries(PortalSolver PRIVATE
    Qt::Widgets
    Qt::Concurrent
)

include(cmake/PortalOpenSees.cmake)

# Copy Qt DLLs + plugins next to the .exe (fixes "Qt6Core.dll not found" when running outside Qt bin).
if(WIN32)
    if(TARGET Qt6::qmake)
        get_target_property(_portal_qmake Qt6::qmake IMPORTED_LOCATION)
        cmake_path(GET _portal_qmake PARENT_PATH _portal_qt_bin)
        find_program(PORTAL_WINDEPLOYQT NAMES windeployqt HINTS "${_portal_qt_bin}" NO_DEFAULT_PATH)
    endif()
    if(NOT PORTAL_WINDEPLOYQT AND DEFINED Qt6_DIR)
        find_program(PORTAL_WINDEPLOYQT NAMES windeployqt HINTS "${Qt6_DIR}/../../../bin" NO_DEFAULT_PATH)
    endif()
    if(PORTAL_WINDEPLOYQT)
        add_custom_command(
            TARGET PortalSolver
            POST_BUILD
            COMMAND "${PORTAL_WINDEPLOYQT}"
                    "$<TARGET_FILE:PortalSolver>"
            COMMENT "windeployqt: Qt runtime next to PortalSolver.exe"
        )
    else()
        message(WARNING "windeployqt not found; run it manually on PortalSolver.exe or add Qt bin to PATH.")
    endif()
endif()

if(OPENSEES_LIBRARY_DIR)
    target_link_directories(PortalSolver PRIVATE ${OPENSEES_LIBRARY_DIR})
endif()

# Link OpenSees core libs when configured. Adjust names to match your build.
if(OPENSEES_INCLUDE_DIR AND OPENSEES_LIBRARY_DIR)
    # Example — uncomment and rename after you know your .lib/.a names:
    # target_link_libraries(PortalSolver PRIVATE OpenSees)
    message(STATUS "OpenSees include: ${OPENSEES_INCLUDE_DIR}")
    message(STATUS "OpenSees libs dir: ${OPENSEES_LIBRARY_DIR}")
endif()

if(MSVC)
    target_compile_definitions(PortalSolver PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
