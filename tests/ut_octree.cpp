/**
 * @file ut_octree.cpp
 * @author Yongchao Wang (ycw.puzzle@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "octree.hpp"
#include <gtest/gtest.h>

using namespace alg;
using namespace alg::octree;

const double eps = 1e-4;

TEST(OctreeTest, TestBuild)
{
    std::vector<Eigen::Vector3f> pts = {
        {0, 0, 0},
        {1, 1, 1},
        {2, 2, 2},
        {3, 3, 3},
    };
    ArrayView<float, int> pt_view{&pts[0][0],
                                  static_cast<int>(sizeof(Eigen::Vector3f)),
                                  static_cast<int>(pts.size())};
    OcTreef octree_1(1.f);
    octree_1.SetPointButter(pt_view);
    EXPECT_EQ(octree_1.Count(), 0);
    EXPECT_EQ(octree_1.GetDepth(), -1);

    octree_1.InsertPoint(0);
    EXPECT_EQ(octree_1.Count(), 1);
    EXPECT_EQ(octree_1.GetDepth(), 0);

    octree_1.InsertPoint(1);
    EXPECT_EQ(octree_1.Count(), 3);
    EXPECT_EQ(octree_1.GetDepth(), 1);

    octree_1.InsertPoint(2);
    EXPECT_EQ(octree_1.Count(), 6);
    EXPECT_EQ(octree_1.GetDepth(), 2);

    octree_1.InsertPoint(3);
    EXPECT_EQ(octree_1.Count(), 7);
    EXPECT_EQ(octree_1.GetDepth(), 2);

    EXPECT_EQ(octree_1.DeleteAllNodes(), 7);
}

TEST(OcTreeTest, TestSearch)
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

    ArrayView<float, int> pt_view{&pts_1[0][0],
                                  static_cast<int>(sizeof(Eigen::Vector3f)),
                                  static_cast<int>(pts_1.size())};

    OcTreef octree_1{2.0f};
    octree_1.SetPointButter(pt_view);
    octree_1.InsertAllPoints();

    std::vector<int> ids_1;
    std::vector<float> dis2_1;
    octree_1.FindKNearest(Eigen::Vector3f{0.f, 0.f, -1.f}, 1, ids_1, dis2_1);
    EXPECT_EQ(ids_1.size(), 1);
    EXPECT_EQ(ids_1[0], 0);
    EXPECT_EQ(dis2_1.size(), 1);
    EXPECT_EQ(dis2_1[0], 1);

    octree_1.FindKNearest(Eigen::Vector3f{10.f, 10.f, 10.f}, 3, ids_1, dis2_1);
    EXPECT_EQ(ids_1.size(), 3);
    EXPECT_EQ(ids_1[0], 9);
    EXPECT_EQ(ids_1[1], 8);
    EXPECT_EQ(ids_1[2], 7);
    EXPECT_EQ(dis2_1.size(), 3);
    EXPECT_EQ(dis2_1[0], 3);
    EXPECT_EQ(dis2_1[1], 12);
    EXPECT_EQ(dis2_1[2], 27);

    octree_1.FindKNearest(Eigen::Vector3f{-10.f, -5.1f, 5.1f}, 2, ids_1,
                          dis2_1);
    EXPECT_EQ(ids_1.size(), 2);
    EXPECT_EQ(ids_1[0], 15);
    EXPECT_EQ(ids_1[1], 16);
    EXPECT_EQ(dis2_1.size(), 2);
    EXPECT_NEAR(dis2_1[0], 0.02, eps);
    EXPECT_NEAR(dis2_1[1], 1.62, eps);
}
