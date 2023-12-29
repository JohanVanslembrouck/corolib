project (transform)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    foreach(_target
        p0100trf
      )
      add_executable(tr-${_target} ${_target}.cpp)
    endforeach()
endif()

foreach(_target
    p0200
    p0202
    p0204
    p0206
    p0210
    p0216

    p0220
    p0226
    p0230
    p0236

    p0240
    p0246
    p0250
    p0256
  )
  add_executable(tr-${_target} ${_target}.cpp print.cpp tracker.cpp)
endforeach()

foreach(_target
    p0300
    p0302
    p0304
    p0306

    p0320
    p0322
    p0324
    p0326

    p0340
    p0342
    p0344
    p0346
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

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    foreach(_target
        p0200
        p0202
        p0204
        p0206
        p0210
        p0216

        p0220
        p0226
        p0230
        p0236

        p0240
        p0246
        p0250
        p0256
      )
      add_executable(tr-${_target}lb ${_target}.cpp print.cpp tracker.cpp)
    endforeach()

    foreach(_target
        p0300
        p0302
        p0304
        p0306

        p0320
        p0322
        p0324
        p0326

        p0340
        p0342
        p0344
        p0346
       )
       add_executable(tr-${_target}lb ${_target}.cpp print.cpp tracker.cpp)
    endforeach()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_options(/DUSE_LBCOROUTINE=1 /DUSE_ORIGINAL_RESUME_IMPLEMENTATION=1)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-DUSE_LBCOROUTINE=1 -DUSE_ORIGINAL_RESUME_IMPLEMENTATION=1)
else()
    message("Unsupported compiler")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    foreach(_target
        p0200trf
        p0202trf
        p0204trf
        p0206trf
        p0210trf
        p0216trf

        p0220trf
        p0226trf
        p0230trf
        p0236trf

        p0240trf
        p0246trf
        p0250trf
        p0256trf
      )
      add_executable(tr-${_target} ${_target}.cpp print.cpp tracker.cpp)
    endforeach()

     foreach(_target
        p0300trf
        p0302trf
        p0304trf
        p0306trf

        p0320trf
        p0322trf
        p0324trf
        p0326trf

        p0340trf
        p0342trf
        p0344trf
        p0346trf
      )
      add_executable(tr-${_target} ${_target}.cpp print.cpp tracker.cpp)
    endforeach()
endif()

