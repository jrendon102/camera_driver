find_program(CLANG_TIDY_EXE NAMES "clang-tidy")

if(CLANG_TIDY_EXE)
    message(STATUS "Found Clang-Tidy: ${CLANG_TIDY_EXE}")

    message(STATUS "CMAKE_SOURC_DIR =${CMAKE_SOURCE_DIR}")

    set(CMAKE_CXX_CLANG_TIDY
        ${CLANG_TIDY_EXE};
        -header-filter=^${CMAKE_SOURCE_DIR}/src/${PROJECT_NAME}/include;
        --config-file=${CMAKE_SOURCE_DIR}/cmake/.clang-tidy;
        --extra-arg=/EHsc)

else(CLANG_TIDY_EXE)
    message(FATAL_ERROR "Clang-Tidy not found. Clang-Tidy checks cannot be applied")
endif()