file(GLOB SRCS ${CMAKE_CURRENT_LIST_DIR}/*.c*)
idf_component_register(SRCS ${SRCS}
                    INCLUDE_DIRS ".")
