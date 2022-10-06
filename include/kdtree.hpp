#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <typeinfo>
#include <algorithm>
#include "queued_ntree.hpp"
#include <numeric>
#include <exception>
#include "geom.hpp"

namespace alg
{

/**
 * @brief node of kdtree
 *
 * @tparam Scalar
 * @tparam Dim
 * @tparam Index
 */
template <typename Scalar, int Dim, typename Index> class KdTreeNode
{
public:
    using Vec = Eigen::Matrix<Scalar, Dim, 1>;
    using VecMap = Eigen::Map<Vec>;
    using VecCMap = Eigen::Map<const Vec>;
    using ThisType = KdTreeNode<Scalar, Dim, Index>;
    using Box = BBox<Scalar, Dim>;

    /**
     * @brief Information used to build each node
     *
     */
    struct BuildInfo
    {
        ArrayIndicesView<Scalar, Index> pts;
        Index max_pts_per_leaf;
    };

    /**
     * @brief leaf node
     *
     */
    class Leaf : public ThisType
    {
    public:
        Index start, num;

        void SetChild(int i, ThisType *c)
        {
            throw std::exception{"Leaf node has no children."};
        }

        std::vector<KdTreeNode *> GetChildren() override { return {}; }
    };

    /**
     * @brief non-leaf node
     *
     */
    class NonLeaf : public ThisType
    {
    public:
        Box box;
        int split_axis;
        ThisType *left = nullptr;
        ThisType *right = nullptr;

        void SetChild(int i, ThisType *c) override
        {
            if (i >= 2)
            {
                throw std::exception{"only 2 children."};
            }
            if (i == 0)
            {
                this->left = c;
            }
            else
            {
                this->right = c;
            }
        }

        std::vector<KdTreeNode *> GetChildren() override
        {
            return {left, right};
        }
    };

    /**
     * @brief check if this node is a leaf
     *
     * @return true
     * @return false
     */
    bool IsLeaf() const { return typeid(*this) == typeid(Leaf); }

    /**
     * @brief The callback used to build each node, it will build a node and
     * generate building information of children nodes
     *
     * @param info
     * @param children_infos
     * @return ThisType*
     */
    static ThisType *Build(BuildInfo &info,
                           std::vector<BuildInfo> &children_infos)
    {
        children_infos.clear();
        if (info.pts.num <= 0)
        {
            return nullptr;
        }
        if (info.pts.num <= info.max_pts_per_leaf)
        {
            auto node = new Leaf;
            node->start = info.pts.start;
            node->num = info.pts.num;
            return node;
        }
        auto node = new NonLeaf;
        node->box = ComputeBBox<Dim>(info.pts);

        Vec box_size = node->box.max() - node->box.min();
        int split_axis = static_cast<int>(
            std::max_element(box_size.data(), box_size.data() + Dim) -
            box_size.data());
        node->split_axis = split_axis;
        Index *s = info.pts.indices + info.pts.start;
        std::nth_element(s, s + info.pts.num / 2, s + info.pts.num,
                         [&](Index i1, Index i2)
                         {
                             return info.pts.MapVec<Dim>(i1)[split_axis] <
                                    info.pts.MapVec<Dim>(i2)[split_axis];
                         });
        children_infos.resize(2);
        children_infos[0] = info;
        children_infos[1] = info;
        children_infos[0].pts.start = info.pts.start;
        children_infos[0].pts.num = info.pts.num / 2;
        children_infos[1].pts.start = info.pts.start + info.pts.num / 2;
        children_infos[1].pts.num = info.pts.num - info.pts.num / 2;

        return node;
    }

    virtual void SetChild(int i, ThisType *c) = 0;
    virtual std::vector<KdTreeNode *> GetChildren() = 0;

    virtual ~KdTreeNode() = default;
};

/**
 * @brief Kd tree
 *
 * @tparam Scalar
 * @tparam Dim
 * @tparam Index
 */
template <typename Scalar, int Dim, typename Index = int>
class KdTree : public QueuedNTree<KdTreeNode<Scalar, Dim, Index>, 2>
{
    // static_assert(std::is_signed_v<Index>);

public:
    using Parent = QueuedNTree<KdTreeNode<Scalar, Dim, Index>, 2>;
    using Node = KdTreeNode<Scalar, Dim, Index>;
    using BuildInfo = typename Node::BuildInfo;
    using Vec = typename Node::Vec;
    using VecCMap = typename Node::VecCMap;

    /**
     * @brief Construct a new Kd Tree. The parameters are almost the same as
     * class ArrayIndicesView
     *
     * @param pts
     * @param pt_num
     * @param indices
     * @param max_pts_per_leaf
     * @param byte_step
     */
    KdTree(const Scalar *pts, Index pt_num, Index *indices = nullptr,
           Index max_pts_per_leaf = 3, Index byte_step = 3 * sizeof(Scalar))
    {
        if (pt_num < 0)
        {
            throw std::exception{"pt num cannot be negative"};
        }
        build_info.pts.Reset(pts, byte_step, indices, (Index)0, pt_num);
        build_info.max_pts_per_leaf = max_pts_per_leaf;
        if (nullptr == indices)
        {
            this->indices.resize(pt_num);
            std::iota(this->indices.begin(), this->indices.end(), (Index)0);
            build_info.pts.indices = const_cast<Index *>(this->indices.data());
        }
        Parent::CreatePreOrder(build_info);
    }

    KdTree(BuildInfo &info) : build_info{info}
    {
        Parent::CreatePreOrder(build_info);
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
        auto box_dis2_func = [&](const Node::Box &b) -> Scalar
        { return OutDis2(b, pt); };
        std::vector<std::pair<Scalar, Index>> knn;
        Scalar k_dis2 = std::numeric_limits<Scalar>::max();

        auto cb = [&](Node *node) -> std::vector<Node *>
        {
            if (node->IsLeaf())
            {
                auto leaf = static_cast<Node::Leaf *>(node);
                for (Index i = 0; i < leaf->num; ++i)
                {
                    Index id = build_info.pts.indices[leaf->start + i];
                    auto p = build_info.pts.MapVec<Dim>(id);
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
                        if (knn.front().first < k_dis2)
                        {
                            k_dis2 = knn.front().first;
                        }
                    }
                }
                return {};
            }

            auto nonleaf = static_cast<Node::NonLeaf *>(node);
            if (box_dis2_func(nonleaf->box) < k_dis2 || knn.size() < k)
            {
                if (OnRightSide(nonleaf->box, pt, nonleaf->split_axis))
                {
                    return {nonleaf->right, nonleaf->left};
                }
                else
                {
                    return {nonleaf->left, nonleaf->right};
                }
            }
            return {};
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
    std::vector<Index> indices;
    BuildInfo build_info;
};

using KdTree3f = KdTree<float, 3, int>;
using KdTree3d = KdTree<double, 3, int>;

} // namespace alg