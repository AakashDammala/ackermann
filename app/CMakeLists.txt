# app/CMakeLists.txt

# Any C++ source files needed to build this target (shell-app).
add_executable(shell-app
  # list of source cpp files:
  main.cpp
  )

# Any include directories needed to build this target.
# Note: this is not strictly needed because the library targets
# use PUBLIC include directories, but it is harmless.
target_include_directories(shell-app PUBLIC
  # list inclue directories:
  ${CMAKE_SOURCE_DIR}/include
)

# Any dependent libraires needed to build this target.
target_link_libraries(shell-app PUBLIC -static
  # list of libraries
  ackermann   # <-- CHANGED: Use the target name, not "libackermann"
              # We don't need 'pid' here because 'ackermann' links to it
  )