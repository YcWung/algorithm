#include "kdtree.hpp"
#include <gtest/gtest.h>
#include "test_path.hpp"
#include <Eigen/Geometry>

using namespace alg;

const double eps = 1e-4;

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

TEST(KdTreeTest, TestSearch)
{
    std::vector<Eigen::Vector3f> pts_1;
    for (int i = 0; i < 10; ++i)
    {
        pts_1.emplace_back((float)i, (float)i, (float)i);
    }
    for (int i = 0; i < 10; ++i)
    {
        pts_1.emplace_back(-10.f, -(float)i, (float)i);
    }
    for (int i = 0; i < 10; ++i)
    {
        pts_1.emplace_back((float)i, -10.f, -(float)i);
    }

    KdTree3f kdtree_1{&pts_1[0][0], static_cast<int>(pts_1.size())};
    std::vector<int> ids_1;
    std::vector<float> dis2_1;
    kdtree_1.FindKNearest(Eigen::Vector3f{0.f, 0.f, -1.f}, 1, ids_1, dis2_1);
    EXPECT_EQ(ids_1.size(), 1);
    EXPECT_EQ(ids_1[0], 0);
    EXPECT_EQ(dis2_1.size(), 1);
    EXPECT_EQ(dis2_1[0], 1);

    kdtree_1.FindKNearest(Eigen::Vector3f{10.f, 10.f, 10.f}, 3, ids_1, dis2_1);
    EXPECT_EQ(ids_1.size(), 3);
    EXPECT_EQ(ids_1[0], 9);
    EXPECT_EQ(ids_1[1], 8);
    EXPECT_EQ(ids_1[2], 7);
    EXPECT_EQ(dis2_1.size(), 3);
    EXPECT_EQ(dis2_1[0], 3);
    EXPECT_EQ(dis2_1[1], 12);
    EXPECT_EQ(dis2_1[2], 27);

    kdtree_1.FindKNearest(Eigen::Vector3f{-10.f, -5.1f, 5.1f}, 2, ids_1,
                          dis2_1);
    EXPECT_EQ(ids_1.size(), 2);
    EXPECT_EQ(ids_1[0], 15);
    EXPECT_EQ(ids_1[1], 16);
    EXPECT_EQ(dis2_1.size(), 2);
    EXPECT_NEAR(dis2_1[0], 0.02, eps);
    EXPECT_NEAR(dis2_1[1], 1.62, eps);
}