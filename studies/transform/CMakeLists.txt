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
    p0200vf
    p0202
    p0202vf
    p0204
    p0204vf
    p0206
    p0206vf
    p0210
    p0210vf
    p0216
    p0216vf

    p0220
    p0226
    p0230
    p0236

    p0240
    p0246
    p0250
    p0256

    p1200
    p1204
    p1206
    p1220
    p1226
    p1240
    p1246
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

    p0330
    p0332
    p0334
    p0336

    p0340
    p0342
    p0344
    p0346

    p1300
    p1304
    p1306

    p1320
    p1324
    p1326

    p1340
    p1344
    p1346
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
        p0200vf
        p0202
        p0202vf
        p0204
        p0204vf
        p0206
        p0206vf
        p0210
        p0210vf
        p0216
        p0216vf

        p0220
        p0226
        p0230
        p0236

        p0240
        p0246
        p0250
        p0256

        p1200
        p1204
        p1206
        p1220
        p1226
        p1240
        p1246
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

        p0330
        p0332
        p0334
        p0336

        p0340
        p0342
        p0344
        p0346

        p1300
        p1304
        p1306

        p1320
        p1324
        p1326

        p1340
        p1344
        p1346
       )
       add_executable(tr-${_target}lb ${_target}.cpp print.cpp tracker.cpp)
    endforeach()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_options(/DUSE_LBCOROUTINE=1 /DUSE_TRANSFORMED_CODE=1 /DUSE_ORIGINAL_RESUME_IMPLEMENTATION=1)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-DUSE_LBCOROUTINE=1 -DUSE_TRANSFORMED_CODE=1 -DUSE_ORIGINAL_RESUME_IMPLEMENTATION=1)
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

        p1200
        p1240
      )
      add_executable(tr-${_target}trf ${_target}.cpp print.cpp tracker.cpp)
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

        p0330
        p0332
        p0334
        p0336

        p0340
        p0342
        p0344
        p0346

        p1300
        p1340
      )
      add_executable(tr-${_target}trf ${_target}.cpp print.cpp tracker.cpp)
    endforeach()
endif()

# The vf variants build with MSVC and GHU
foreach(_target
    p0200vf
    p0202vf
    p0204vf
    p0206vf
    p0210vf
    p0216vf
  )
  add_executable(tr-${_target}trf ${_target}.cpp print.cpp tracker.cpp)
endforeach()
