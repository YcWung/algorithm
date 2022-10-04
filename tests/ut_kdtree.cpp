#include "kdtree.hpp"
#include <gtest/gtest.h>
#include "test_path.hpp"
#include <Eigen/Geometry>

using namespace alg;

TEST(KdTreeTest, TestBuild)
{
    std::vector<Eigen::Vector3f> pts_1 = {
        {0, 0, 0},
        {1, 1, 1},
        {2, 2, 2},
    };

    KdTree3f kdtree_1{&pts_1[0][0], static_cast<int>(pts_1.size())};
    EXPECT_EQ(kdtree_1.GetNumberOfNodes(), 1);

    std::vector<Eigen::Vector3d> pts_2 = {
        {5, 0, 0}, {4, 0, 0}, {3, 0, 0}, {2, 0, 0}, {1, 0, 0},
    };
    KdTree3d kdtree_2{&pts_2[0][0], static_cast<int>(pts_2.size())};
    EXPECT_EQ(kdtree_2.GetNumberOfNodes(), 3);

    KdTree3f kdtree_3{nullptr, 0};
    EXPECT_EQ(kdtree_3.GetNumberOfNodes(), 0);

    EXPECT_THROW(KdTree3f(nullptr, -1), std::exception);
}