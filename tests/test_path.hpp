#pragma once

#ifndef TEST_DATA_DIR
#error TEST_DATA_DIR not defined
#endif

#include <string>

inline std::string GetTestDataDir() { return TEST_DATA_DIR; }
