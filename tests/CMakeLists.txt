# basic GTEST template taken from:
#  https://google.github.io/googletest/quickstart-cmake.html

# include gtests as a project dependency
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  dummy_test
  dummy_test.cpp
)
target_link_libraries(
  dummy_test
  ${PROJECT_NAME}
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(dummy_test)
