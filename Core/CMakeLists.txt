file(GLOB FILES "Src/*.*" "Inc/*.*")

add_executable(${PROJECT_NAME} ${FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        )

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_unique_id
        tinyusb_device
        tinyusb_board
        cmsis_core
        )

target_link_options(pico_standard_link INTERFACE "LINKER:--print-memory-usage")

pico_add_extra_outputs(${PROJECT_NAME})
pico_add_uf2_output(${PROJECT_NAME})