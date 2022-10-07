/**
 * @file octree.hpp
 * @author Yongchao Wang (ycw.puzzle@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include "ntree.hpp"
#include "geom.hpp"
#include <iostream>

namespace alg
{

namespace octree
{

/**
 * @brief node for octree
 *
 * @tparam Scalar
 * @tparam Index
 */
template <typename Scalar, typename Index> class OcTreeNode
{
    using Node = OcTreeNode<Scalar, Index>;
    using Cube = Cube<Scalar, 3>;

public:
    Cube cube;
    std::vector<Node *> children;
    std::vector<Index> indices;

    OcTreeNode() {}

    OcTreeNode(const Cube &c) : cube{c} {}

    bool IsLeaf() const { return children.empty(); }

    void SetChild(int i, Node *node)
    {
        if (i < 0 || i >= 8)
        {
            throw std::exception{"i out of range [0, 8)"};
        }
        if (children.size() != 8)
        {
            children.resize(8, nullptr);
        }
        if (children[i] != nullptr)
        {
            delete children[i];
        }
        children[i] = node;
    }

    std::vector<Node *> &GetChildren() { return children; }

    Node *InsertChild(int i)
    {
        if (children.size() != 8)
        {
            children.resize(8, nullptr);
        }
        if (children[i] != nullptr)
        {
            return children[i];
        }
        auto c = cube.Octant(i);
        auto n = new Node;
        n->cube = c;
        children[i] = n;
        return n;
    }
};

/**
 * @brief Octree
 *
 * @tparam Scalar
 * @tparam Index
 */
template <typename Scalar, typename Index = int>
class OcTree : public ntree::NTree<OcTreeNode<Scalar, Index>, 8>
{
public:
    using Node = OcTreeNode<Scalar, Index>;
    using Parent = ntree::NTree<Node, 8>;
    using Vec = Vec<Scalar, 3>;
    using VecCMap = Eigen::Map<const Vec>;
    using Cube = Cube<Scalar, 3>;
    using BBox = typename Cube::BBox;
    using Points = ArrayView<Scalar, Index>;

    OcTree(Scalar leaf_width = (Scalar)1e-3)
        : leaf_half_width{(Scalar)0.5f * leaf_width}
    {
    }

    ~OcTree() { Parent::DeleteAllNodes(); }

    /**
     * @brief Set the buffer of point data
     *
     * This will not create any node in the tree
     *
     * @param pts
     */
    void SetPointButter(const Points &pts) { this->pts = pts; }

    /**
     * @brief Get the depth of the tree
     *
     * -1 means no tree.
     * 0 means only a root node.
     *
     * @return int
     */
    int GetDepth() const { return depth; }

    /**
     * @brief Get the width of each leaf node
     *
     * @return Scalar
     */
    Scalar GetLeafWidth() const { return 2 * leaf_half_width; }

    /**
     * @brief Insert a point into the tree by creating a leaf and all
     * intermediate nodes if neccesary
     *
     * @param id
     */
    void InsertPoint(Index id)
    {
        Vec pt = pts.MapVec<3>(id);
        Enlarge(pt);
        Node *n = this->root;
        for (int i = 1; i <= depth; ++i)
        {
            n = n->InsertChild(n->cube.LocateOctant(pt));
        }
        n->indices.emplace_back(id);
    }

    /**
     * @brief Insert all points in buffer into the tree
     *
     */
    void InsertAllPoints()
    {
        for (Index i = 0; i < pts.num; ++i)
        {
            InsertPoint(i);
        }
    }

    /**
     * @brief Find the nearest k points
     *
     * @param[in] pt
     * @param[in] k
     * @param[out] k_indices
     * @param[out] k_sqr_distances
     */
    void FindKNearest(const Vec &pt, Index k, std::vector<Index> &k_indices,
                      std::vector<Scalar> &k_sqr_distances)
    {
        if (k <= 0)
        {
            throw std::exception{"k must be positive"};
        }
        auto dis2_func = [&](const VecCMap &p) -> Scalar
        { return (p - pt).squaredNorm(); };
        auto box_dis2_func = [&](const BBox &b) -> Scalar
        { return OutDis2(b, pt); };
        std::vector<std::pair<Scalar, Index>> knn;
        Scalar k_dis2 = std::numeric_limits<Scalar>::max();

        auto cb = [&](Node *node) -> std::vector<Node *>
        {
            if (box_dis2_func(node->cube.GetBBox()) >= k_dis2 &&
                knn.size() == k)
            {
                return {};
            }
            for (Index id : node->indices)
            {
                auto p = pts.MapVec<3>(id);
                Scalar dis2 = dis2_func(p);
                if (dis2 < k_dis2 || knn.size() < k)
                {
                    knn.emplace_back(dis2, id);
                    std::push_heap(knn.begin(), knn.end());
                    if (knn.size() > k)
                    {
                        std::pop_heap(knn.begin(), knn.end());
                        knn.resize(k);
                    }
                    if (knn.size() == k && knn.front().first < k_dis2)
                    {
                        k_dis2 = knn.front().first;
                    }
                }
            }
            return node->children;
        };

        Parent::template TraverseFromRoot(cb);

        size_t num_found = knn.size();
        k_indices.resize(num_found);
        k_sqr_distances.resize(num_found);
        for (size_t i = 0; i < num_found; ++i)
        {
            std::pop_heap(knn.begin(), knn.end());
            k_sqr_distances[num_found - 1 - i] = knn.back().first;
            k_indices[num_found - 1 - i] = knn.back().second;
            knn.resize(knn.size() - 1);
        }
    }

protected:
    using Parent::root;

    void Enlarge(const Vec &p)
    {
        if (root == nullptr)
        {
            root = new Node{{p, leaf_half_width}};
            depth = 0;
            return;
        }
        Vec dir = p - root->cube.center;
        dir = OctantDir(dir);
        auto idx = OctantIndex(Vec{-dir});
        while (!root->cube.Contains(p))
        {
            auto node = new Node(root->cube.SubCube(dir, (Scalar)2));
            node->SetChild(idx, root);
            root = node;
            ++depth;
        }
    }

    Points pts;
    Scalar leaf_half_width = (Scalar)1e-3;
    int depth = -1;
};

using OcTreef = OcTree<float, int>;

} // namespace octree

} // namespace alg