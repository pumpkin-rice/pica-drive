# loguru internal cmake file. Do not use it in user top-level project

add_library(loguru STATIC)

set(SRC
    loguru/loguru.cpp
)

set_target_properties(loguru
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${ARCHIVE_OUTPUT_DIRECTORY}"
    LIBRARY_OUTPUT_DIRECTORY "${ARCHIVE_OUTPUT_DIRECTORY}"
    RUNTIME_OUTPUT_DIRECTORY "${ARCHIVE_OUTPUT_DIRECTORY}"
)

target_include_directories(loguru
    PUBLIC
        .
)

target_sources(loguru
    PRIVATE
        ${SRC}
)

target_link_libraries(loguru
    PUBLIC
)
