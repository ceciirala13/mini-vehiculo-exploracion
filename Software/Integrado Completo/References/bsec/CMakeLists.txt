idf_component_register(
    SRCS "config/bsec_iaq.c"
    INCLUDE_DIRS "include" "config"
)

# Vincular el binario precompilado de Bosch para ESP32-S3
set(BSEC_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/esp32_s3/libalgobsec.a")

add_library(algobsec STATIC IMPORTED GLOBAL)
set_target_properties(algobsec PROPERTIES IMPORTED_LOCATION "${BSEC_LIB}")

if(TARGET ${COMPONENT_LIB})
    target_link_libraries(${COMPONENT_LIB} PUBLIC algobsec)
endif()
if(TARGET ${COMPONENT_TARGET})
    target_link_libraries(${COMPONENT_TARGET} PUBLIC algobsec)
endif()
