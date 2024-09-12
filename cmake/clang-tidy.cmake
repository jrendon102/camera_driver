find_program(CLANG_TIDY_EXE NAMES "clang-tidy")

if(CLANG_TIDY_EXE)
    message(STATUS "Found Clang-Tidy: ${CLANG_TIDY_EXE}")
    message(STATUS "CMAKE_SOURC_DIR =${CMAKE_SOURCE_DIR}")

    # Check if the MSVC compiler is being used
    if (MSVC)
        # Set Clang-Tidy with MSVC-specific flag /EHsc
        set(CMAKE_CXX_CLANG_TIDY
            ${CLANG_TIDY_EXE};
            -header-filter=^${CMAKE_SOURCE_DIR}/src/${PROJECT_NAME}/include;
            --config-file=${CMAKE_SOURCE_DIR}/cmake/.clang-tidy;
            --extra-arg=/EHsc)
        message(STATUS "MSVC detected: Adding /EHsc flag to Clang-Tidy configuration")
    else()
        set(CMAKE_CXX_CLANG_TIDY
            ${CLANG_TIDY_EXE};
            -header-filter=^${CMAKE_SOURCE_DIR}/src/${PROJECT_NAME}/include;
            --config-file=${CMAKE_SOURCE_DIR}/cmake/.clang-tidy)
        message(STATUS "Non-MSVC compiler detected: Clang-Tidy configuration without /EHsc")
    endif()

else(CLANG_TIDY_EXE)
    message(FATAL_ERROR "Clang-Tidy not found. Clang-Tidy checks cannot be applied")
endif()