#include <pybind11/pybind11.h>

PYBIND11_MODULE(pct_pybind11_smoke, module) {
  module.def("answer", []() { return 42; });
}
