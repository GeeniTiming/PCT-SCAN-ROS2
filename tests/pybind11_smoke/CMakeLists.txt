cmake_minimum_required(VERSION 3.16)
project(pct_pybind11_smoke LANGUAGES CXX)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 2.10.4 CONFIG REQUIRED)

pybind11_add_module(pct_pybind11_smoke binding.cpp)

enable_testing()
add_test(
  NAME import_private_pybind11_module
  COMMAND "${Python3_EXECUTABLE}" -c
          "import pct_pybind11_smoke; assert pct_pybind11_smoke.answer() == 42"
)
set_tests_properties(
  import_private_pybind11_module
  PROPERTIES ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:pct_pybind11_smoke>"
)
