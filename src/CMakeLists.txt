ADD_EXECUTABLE(benchmarks
  benchmark.cc
  pr2-in-iai-kitchen.cc
  pyrene-on-the-ground.cc
  romeo-placard.cc
  )

target_link_libraries(benchmarks hpp-core::hpp-core
  hpp-manipulation-urdf::hpp-manipulation-urdf)
