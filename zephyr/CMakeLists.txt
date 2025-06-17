if(CONFIG_ZENOH_PICO)

zephyr_compile_definitions(ZENOH_ZEPHYR)
zephyr_include_directories(../include)
zephyr_library()

function(configure_zenoh_feature config)
  string(REPLACE CONFIG_ZENOH_PICO Z_FEATURE feature ${config})
  if(${config})
    zephyr_compile_definitions(${feature}=1)
  else()
    zephyr_compile_definitions(${feature}=0)
  endif()
endfunction()


configure_zenoh_feature(CONFIG_ZENOH_PICO_LINK_SERIAL)
configure_zenoh_feature(CONFIG_ZENOH_PICO_MULTI_THREAD)
configure_zenoh_feature(CONFIG_ZENOH_PICO_PUBLICATION)
configure_zenoh_feature(CONFIG_ZENOH_PICO_SUBSCRIPTION)
configure_zenoh_feature(CONFIG_ZENOH_PICO_QUERY)
configure_zenoh_feature(CONFIG_ZENOH_PICO_QUERYABLE)
configure_zenoh_feature(CONFIG_ZENOH_PICO_RAWETH_TRANSPORT)
configure_zenoh_feature(CONFIG_ZENOH_PICO_LINK_TCP)
configure_zenoh_feature(CONFIG_ZENOH_PICO_LINK_UDP_UNICAST)
configure_zenoh_feature(CONFIG_ZENOH_PICO_LINK_UDP_MULTICAST)
configure_zenoh_feature(CONFIG_ZENOH_PICO_SCOUTING_UDP)
configure_zenoh_feature(CONFIG_ZENOH_PICO_LINK_WS)


# Always include core sources
file(GLOB_RECURSE Sources_Core
  "../src/api/*.c"
  "../src/collections/*.c"
  "../src/protocol/*.c"
  "../src/session/*.c"
  "../src/utils/*.c"
  "../src/system/common/*.c"
)

# Include base link and transport files
file(GLOB Sources_Link_Base
  "../src/link/endpoint.c"
  "../src/link/link.c"
)

# Include core transport files
file(GLOB Sources_Transport_Base 
  "../src/transport/manager.c"
  "../src/transport/peer.c"
  "../src/transport/transport.c"
  "../src/transport/utils.c"
  "../src/transport/common/*.c"
)

file(GLOB Sources_Net "../src/net/*.c")

list(APPEND Sources ${Sources_Core} ${Sources_Link_Base} ${Sources_Transport_Base} ${Sources_Net})

# Conditionally include transport implementations
if(CONFIG_ZENOH_PICO_LINK_SERIAL OR CONFIG_ZENOH_PICO_LINK_TCP OR CONFIG_ZENOH_PICO_LINK_UDP_UNICAST OR CONFIG_ZENOH_PICO_LINK_WS)
  file(GLOB Sources_Unicast "../src/transport/unicast/*.c")
  list(APPEND Sources ${Sources_Unicast})
endif()

if(CONFIG_ZENOH_PICO_LINK_UDP_MULTICAST)
  file(GLOB Sources_Multicast_Transport "../src/transport/multicast/*.c")
  file(GLOB Sources_Multicast_Base "../src/transport/multicast.c")
  list(APPEND Sources ${Sources_Multicast_Transport} ${Sources_Multicast_Base})
endif()

if(CONFIG_ZENOH_PICO_RAWETH_TRANSPORT)
  file(GLOB Sources_Raweth_Transport "../src/transport/raweth/*.c")
  list(APPEND Sources ${Sources_Raweth_Transport})
endif()

# Conditionally include link implementations based on enabled features
if(CONFIG_ZENOH_PICO_LINK_SERIAL)
  file(GLOB Sources_Serial "../src/link/unicast/serial.c")
  file(GLOB Sources_Serial_Config "../src/link/config/serial.c")
  list(APPEND Sources ${Sources_Serial} ${Sources_Serial_Config})
endif()

if(CONFIG_ZENOH_PICO_LINK_TCP)
  file(GLOB Sources_TCP "../src/link/unicast/tcp.c")
  file(GLOB Sources_TCP_Config "../src/link/config/tcp.c")
  list(APPEND Sources ${Sources_TCP} ${Sources_TCP_Config})
endif()

if(CONFIG_ZENOH_PICO_LINK_UDP_UNICAST)
  file(GLOB Sources_UDP "../src/link/unicast/udp.c")
  file(GLOB Sources_UDP_Config "../src/link/config/udp.c")
  list(APPEND Sources ${Sources_UDP} ${Sources_UDP_Config})
endif()

if(CONFIG_ZENOH_PICO_LINK_UDP_MULTICAST)
  file(GLOB Sources_UDP_Multi "../src/link/multicast/*.c")
  list(APPEND Sources ${Sources_UDP_Multi})
endif()

if(CONFIG_ZENOH_PICO_LINK_WS)
  file(GLOB Sources_WS "../src/link/unicast/ws.c")
  file(GLOB Sources_WS_Config "../src/link/config/ws.c")
  list(APPEND Sources ${Sources_WS} ${Sources_WS_Config})
endif()

if(CONFIG_ZENOH_PICO_RAWETH_TRANSPORT)
  file(GLOB Sources_RawEth_Config "../src/link/config/bt.c")
  list(APPEND Sources ${Sources_RawEth_Config})
endif()

# Conditionally include Zephyr system files
file(GLOB Sources_Zephyr_Base "../src/system/zephyr/system.c")
list(APPEND Sources ${Sources_Zephyr_Base})

# Only include network.c if networking features are enabled
if(CONFIG_ZENOH_PICO_LINK_TCP OR CONFIG_ZENOH_PICO_LINK_UDP_UNICAST OR CONFIG_ZENOH_PICO_LINK_UDP_MULTICAST OR CONFIG_ZENOH_PICO_LINK_WS OR CONFIG_ZENOH_PICO_RAWETH_TRANSPORT)
  file(GLOB Sources_Network "../src/system/zephyr/network.c")
  list(APPEND Sources ${Sources_Network})
endif()

zephyr_library_sources(${Sources})

# Note: atomic library not available in this Zephyr SDK toolchain
endif()
