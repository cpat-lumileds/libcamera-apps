cmake_minimum_required(VERSION 3.6)

add_library(lumileds lumileds_i2c.cpp)

install(TARGETS lumileds LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})