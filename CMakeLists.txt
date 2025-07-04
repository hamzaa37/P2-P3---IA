# Set the minimum required version of CMake and define the project and language.
cmake_minimum_required(VERSION 3.10)
project(Motor_Grafico_Belkan CXX)

# Set the C++ standard to C++17. This is the modern way to set the standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- FIND ALL REQUIRED PACKAGES ---
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(JPEG REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

# --- GLOB SOURCE FILES ---
file(GLOB SOURCES "src/*.cpp" "src/*.cxx" "src/*.cc")
file(GLOB SOURCES_LIB3D "src/obj3dlib/*.cpp" "src/obj3dlib/*.cxx" "src/obj3dlib/*.cc" "src/obj3dlib/*/*.cpp" "src/obj3dlib/*/*.cxx" "src/obj3dlib/*/*.cc")
file(GLOB SOURCES_LIBMOTOR "src/motorlib/*.cpp" "src/motorlib/*.cxx" "src/motorlib/*.cc")
file(GLOB SOURCES_MODELOS "src/modelos/*/*.cpp" "src/modelos/*/*.cxx" "src/modelos/*.cc" "src/modelos/*.cpp" "src/modelos/*.cxx" "src/modelos/*.cc")
file(GLOB SOURCES_GLUI "src/glui/*.cpp" "src/glui/*.cxx" "src/glui/*.cc")
file(GLOB SOURCES_COMPORTAMIENTOS "src/comportamientos/*.cpp" "src/comportamientos/*.cxx" "src/comportamientos/*.cc" "Comportamientos_Jugador/*.cpp")

# --- SET OUTPUT DIRECTORIES ---
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# --- DEFINE AND LINK LIBRARIES AND EXECUTABLES ---
add_library(obj3DLib ${SOURCES_LIB3D})
target_include_directories(obj3DLib PRIVATE src include)
target_link_libraries(obj3DLib PRIVATE OpenGL::GL OpenGL::GLU GLUT::GLUT JPEG::JPEG Threads::Threads)

add_library(GLUILib ${SOURCES_GLUI})
target_include_directories(GLUILib PRIVATE src include)
target_link_libraries(GLUILib PRIVATE OpenGL::GL OpenGL::GLU GLUT::GLUT)

add_library(modelosBelkanLib ${SOURCES_MODELOS})
target_include_directories(modelosBelkanLib PRIVATE src include)
target_link_libraries(modelosBelkanLib PRIVATE obj3DLib OpenGL::GL OpenGL::GLU GLUT::GLUT OpenMP::OpenMP_CXX Threads::Threads)

add_library(comportamientosLib ${SOURCES_COMPORTAMIENTOS})
target_include_directories(comportamientosLib PRIVATE src include)
target_link_libraries(comportamientosLib PRIVATE OpenMP::OpenMP_CXX Threads::Threads)

add_library(motorLib ${SOURCES_LIBMOTOR})
target_include_directories(motorLib PRIVATE src include)
target_link_libraries(motorLib PRIVATE obj3DLib GLUILib comportamientosLib OpenGL::GL OpenGL::GLU GLUT::GLUT OpenMP::OpenMP_CXX Threads::Threads)

add_executable(practica2 "bin_src/juego.cc" ${SOURCES})
target_include_directories(practica2 PRIVATE src include)
# ADDED OpenGL::GLU HERE
target_link_libraries(practica2 PRIVATE obj3DLib motorLib modelosBelkanLib OpenGL::GLU OpenMP::OpenMP_CXX Threads::Threads)

add_executable(practica2SG "bin_src/juegoSG.cc" ${SOURCES})
target_include_directories(practica2SG PRIVATE src include)
# ADDED OpenGL::GLU HERE
target_link_libraries(practica2SG PRIVATE obj3DLib motorLib modelosBelkanLib OpenGL::GLU OpenMP::OpenMP_CXX Threads::Threads)