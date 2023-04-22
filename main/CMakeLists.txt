
idf_component_register(	SRCS "main.cpp"
						INCLUDE_DIRS "." "include/"
						REQUIRES driver esp32_driver_mcp320x)

component_compile_options(-std=gnu++2a)
