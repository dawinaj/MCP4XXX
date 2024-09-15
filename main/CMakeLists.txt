
idf_component_register(	SRCS "main.cpp"
						INCLUDE_DIRS "." "include/"
						REQUIRES driver)

component_compile_options(-std=gnu++2a)
