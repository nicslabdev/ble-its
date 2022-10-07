if(NOT TARGET Boost::system)
    message(STATUS "Skip build of its_ble because of missing Boost::system dependency")
    return()
endif()

if(NOT TARGET Boost::program_options)
    message(STATUS "Skip build of its_ble because of missing Boost::program_options dependency")
    return()
endif()

find_package(Threads REQUIRED)

add_executable(its_ble
    application.cpp
    cam_tx_application.cpp
    cam_rx_application.cpp
    dcc_passthrough.cpp
    ethernet_device.cpp
    link_layer.cpp
    main.cpp
    positioning.cpp
    raw_socket_link.cpp
    router_context.cpp
    security.cpp
    time_trigger.cpp
    uart_link_tx.cpp
    uart_link_rx.cpp
    log.cpp
)

target_link_libraries(its_ble PUBLIC Boost::system Boost::program_options Threads::Threads vanetza serial)
install(TARGETS its_ble EXPORT ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

