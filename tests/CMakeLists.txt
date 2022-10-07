################################################################################
 # @author Yongchao Wang (ycw.puzzle@hotmail.com)
 # @brief
 # @version 0.1
 # @date 2022-10-07
 #
 #@copyright Copyright (c) 2022
 #
################################################################################

include(add_ut)

add_ut(
    TEST_NAME ut_kdtree
    TEST_SRCS ut_kdtree.cpp
)

add_ut(
    TEST_NAME ut_octree
    TEST_SRCS ut_octree.cpp
)
