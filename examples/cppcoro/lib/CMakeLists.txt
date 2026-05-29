project(cppcorolib)

include_directories(../include)

set(sourcesorig
    async_auto_reset_event.cpp
    async_manual_reset_event.cpp
    async_mutex.cpp
    auto_reset_event.cpp
    cancellation_registration.cpp
    cancellation_source.cpp
    cancellation_state.cpp
    cancellation_token.cpp
    file.cpp
    file_read_operation.cpp
    file_write_operation.cpp
    io_service.cpp
    ip_address.cpp
    ip_endpoint.cpp
    ipv4_address.cpp
    ipv4_endpoint.cpp
    ipv6_address.cpp
    ipv6_endpoint.cpp
    lightweight_manual_reset_event.cpp
    read_only_file.cpp
    readable_file.cpp
    socket.cpp
    socket_accept_operation.cpp
    socket_connect_operation.cpp
    socket_disconnect_operation.cpp
    socket_helpers.cpp
    socket_recv_from_operation.cpp
    socket_recv_operation.cpp
    socket_send_operation.cpp
    socket_send_to_operation.cpp
    spin_mutex.cpp
    spin_wait.cpp
    static_thread_pool.cpp
    win32.cpp
    writable_file.cpp
    write_only_file.cpp
)

set(sources
	async_auto_reset_event.cpp
	async_manual_reset_event.cpp
	async_mutex.cpp
	cancellation_state.cpp
	cancellation_token.cpp
	cancellation_source.cpp
	cancellation_registration.cpp
	lightweight_manual_reset_event.cpp
	ip_address.cpp
	ip_endpoint.cpp
	ipv4_address.cpp
	ipv4_endpoint.cpp
	ipv6_address.cpp
	ipv6_endpoint.cpp
	static_thread_pool.cpp
	auto_reset_event.cpp
	spin_wait.cpp
	spin_mutex.cpp
)

if(WIN32)
    set(win32Sources
        win32.cpp
        io_service.cpp
        file.cpp
        readable_file.cpp
        writable_file.cpp
        read_only_file.cpp
        write_only_file.cpp
        read_write_file.cpp
        file_read_operation.cpp
        file_write_operation.cpp
        socket_helpers.cpp
        socket.cpp
        socket_accept_operation.cpp
        socket_connect_operation.cpp
        socket_disconnect_operation.cpp
        socket_send_operation.cpp
        socket_send_to_operation.cpp
        socket_recv_operation.cpp
        socket_recv_from_operation.cpp
    )
    list(APPEND sources ${win32Sources})

elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(linuxSources
        linux.cpp
        io_service.cpp
        file.cpp
        readable_file.cpp
        writable_file.cpp
        read_only_file.cpp
        write_only_file.cpp
        read_write_file.cpp
        file_read_operation.cpp
        file_write_operation.cpp
        socket_helpers.cpp
        socket.cpp
        socket_accept_operation.cpp
        socket_connect_operation.cpp
        socket_disconnect_operation.cpp
        socket_send_operation.cpp
        socket_send_to_operation.cpp
        socket_recv_operation.cpp
        socket_recv_from_operation.cpp
    )
    list(APPEND sources ${linuxSources})
endif()

add_library(cppcorolib
    ${sources}
)

target_link_libraries(cppcorolib)

install(TARGETS cppcorolib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
