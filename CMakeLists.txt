add_library(embroidery INTERFACE)

target_sources(embroidery INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/embroidery.c
 ${CMAKE_CURRENT_LIST_DIR}/brother.c
 ${CMAKE_CURRENT_LIST_DIR}/tajima.c
)

target_include_directories(embroidery INTERFACE ${CMAKE_CURRENT_LIST_DIR})
