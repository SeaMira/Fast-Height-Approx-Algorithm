# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "CGAL_Qt5_moc_and_resources_autogen"
  "CMakeFiles\\CGAL_Qt5_moc_and_resources_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\CGAL_Qt5_moc_and_resources_autogen.dir\\ParseCache.txt"
  )
endif()
