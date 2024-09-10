target_include_directories(${LIBRARY_NAME} PUBLIC
	"${CMAKE_CURRENT_SOURCE_DIR}"
	${OpenCV_INCLUDE_DIRS}
)

# Specify headers to install
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)