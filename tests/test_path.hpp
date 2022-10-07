/**
 * @file test_path.hpp
 * @author Yongchao Wang (ycw.puzzle@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#ifndef TEST_DATA_DIR
#error TEST_DATA_DIR not defined
#endif

#include <string>

inline std::string GetTestDataDir() { return TEST_DATA_DIR; }
