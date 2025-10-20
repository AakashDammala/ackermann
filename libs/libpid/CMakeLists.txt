# Created a library called "libpid" (in Linux, this library is created
# with the name of either libpid.a or libpid.so).
add_library (libpid
  # list of cpp source files:
  src/src.cpp
  )

# list of directories to include when using this library
target_include_directories(libpid PUBLIC
  # list of directories:
  include
  )
