project(Baseball_QT)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
aux_source_directory(. SRC_LIST)


find_package( OpenCV 3 REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})

set(project_sources
    main.cpp
    ball_catcher.cpp)

add_executable(${PROJECT_NAME} ${project_sources})


target_link_libraries(Baseball_QT ${OpenCV_LIBS} )
