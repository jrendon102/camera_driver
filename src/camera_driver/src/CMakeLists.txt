set(LIBRARY_SOURCES
	"camera.cpp"
	"camera_utils.cpp"
)

set(LIBRARY_HEADERS
	"../include/camera.h"
	"../include/camera_utils.h"
)

add_library(${LIBRARY_NAME} STATIC
	${LIBRARY_SOURCES}
	${LIBRARY_HEADERS}
)

# Specify the installation path for the library and headers
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
# Specify headers to install
install(DIRECTORY ../include/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
)