set(SRCS "HttpServer.cpp" "HttpRequest.cpp" "HttpResponse.cpp")

set(INCLUDE_DIRS ".")

set(COMPONENT_PRIV_REQUIRES esp_hw_support esp_https_server esp_wifi joltwallet__littlefs)

register_component()
