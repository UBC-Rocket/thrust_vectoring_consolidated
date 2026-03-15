# CMake generated Testfile for 
# Source directory: /Users/ivan/ubc_rocket/thrust_vectoring_consolidated/embedded-software/firmware/libs/controls/tests
# Build directory: /Users/ivan/ubc_rocket/thrust_vectoring_consolidated/embedded-software/firmware/libs/controls/build-tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[controls_tests]=] "/Users/ivan/ubc_rocket/thrust_vectoring_consolidated/embedded-software/firmware/libs/controls/build-tests/test_controls")
set_tests_properties([=[controls_tests]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/ivan/ubc_rocket/thrust_vectoring_consolidated/embedded-software/firmware/libs/controls/tests/CMakeLists.txt;19;add_test;/Users/ivan/ubc_rocket/thrust_vectoring_consolidated/embedded-software/firmware/libs/controls/tests/CMakeLists.txt;0;")
subdirs("state_estimation")
subdirs("controls")
subdirs("unity")
