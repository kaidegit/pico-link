file(GLOB FILES "Src/*.*" "Inc/*.*")

set(PICO_TOOL ${CMAKE_CURRENT_SOURCE_DIR}/../tools/picotool.exe)

add_executable(${PROJECT_NAME} ${FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        )

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_unique_id
        tinyusb_device
        tinyusb_board
        )

#add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)

target_link_options(pico_standard_link INTERFACE "LINKER:--print-memory-usage")

pico_add_extra_outputs(${PROJECT_NAME})
pico_add_uf2_output(${PROJECT_NAME})

#add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#        COMMAND ${PICO_TOOL}
#        )
