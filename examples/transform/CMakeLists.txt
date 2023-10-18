project (transform)

foreach(_target
    p0100trf
  )
  add_executable(tr-${_target} ${_target}.cpp)
endforeach()

foreach(_target
    p0200 p0200lb p0200trf
    p0202 p0202lb p0202trf
    p0204 p0204lb p0204trf
    p0206 p0206lb p0206trf
  )
  add_executable(tr-${_target} ${_target}.cpp print.cpp tracker.cpp)
endforeach()
