# Created a library called "libackermann" (in Linux, this library is created
# with the name of either libackermann.a or libackermann.so).
add_library (libackermann
  # list of cpp source files:
  src/src.cpp
  )

# list of directories to include when using this library
target_include_directories(libackermann PUBLIC
  # list of directories:
  include
  )

# Link against libpid which provides the PIDController used by this library
target_link_libraries(libackermann PUBLIC
  libpid
)
