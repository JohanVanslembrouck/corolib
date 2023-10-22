project (transform)

foreach(_target
    p0100trf
  )
  add_executable(tr-${_target} ${_target}.cpp)
endforeach()

foreach(_target
    p0200
    p0202
    p0204
    p0206
    p0210
    p0216
  )
  add_executable(tr-${_target} ${_target}.cpp print.cpp tracker.cpp)
endforeach()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_options(/DUSE_LBCOROUTINE=1)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-DUSE_LBCOROUTINE=1)
else()
    message("Unsupported compiler")
endif()

foreach(_target
    p0200
    p0202
    p0204
    p0206
    p0210
    p0216
  )
  add_executable(tr-${_target}lb ${_target}.cpp print.cpp tracker.cpp)
endforeach()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_options(/DUSE_LBCOROUTINE=1 /DUSE_ORIGINAL_RESUME_IMPLEMENTATION=1)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-DUSE_LBCOROUTINE=1 -DUSE_ORIGINAL_RESUME_IMPLEMENTATION=1)
else()
    message("Unsupported compiler")
endif()

foreach(_target
    p0200trf
    p0202trf
    p0204trf
    p0206trf
    p0210trf
    p0216trf
  )
  add_executable(tr-${_target} ${_target}.cpp print.cpp tracker.cpp)
endforeach()
