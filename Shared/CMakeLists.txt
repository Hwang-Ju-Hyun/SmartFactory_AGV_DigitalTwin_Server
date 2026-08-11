set(CMAKE_BUILD_TYPE Debug)

add_library(Shared STATIC
    SocketAddress.cpp
    SocketUtil.cpp
    TCPSocket.cpp    
    MemoryStream.cpp        
    SocketAddressFactory.cpp                 
    TCPSession.cpp
    PacketSerializer.cpp
    TrajectoryBuilder.cpp
    MovementSimulator.cpp
    Object.cpp
    Robo.cpp
    LinkingContext.cpp
    ObjectRegistry.cpp    
    Map.cpp
    PathFinder.cpp    
    AGVManager.cpp
    WarehouseManager.cpp
    Event.cpp
    DispatchManager.cpp    
    RRAstar.cpp
    ReservationTable.cpp
)

target_include_directories(Shared PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
