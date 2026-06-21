# FreeRTOS internal cmake file. Do not use it in user top-level project

add_library(picadrive_include INTERFACE)

target_include_directories(picadrive_include
    INTERFACE
        .
)

target_link_libraries(picadrive_include
    INTERFACE

)
