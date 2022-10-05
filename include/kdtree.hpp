#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <typeinfo>
#include <algorithm>
#include "queued_ntree.hpp"
#include <numeric>
#include <exception>
#include "data_wrap.hpp"

namespace alg
{

template <typename Scalar, int Dim, typename Index> class KdTreeNode
{
public:
    using Vec = Eigen::Matrix<Scalar, Dim, 1>;
    using VecMap = Eigen::Map<Vec>;
    using VecCMap = Eigen::Map<const Vec>;
    using ThisType = KdTreeNode<Scalar, Dim, Index>;

    struct Box
    {
        Vec min, max;
        Box() = default;
        Box(const Vec &min, const Vec &max) : min{min}, max{max} {}
        Scalar OutwardDistance(const Vec &pt) const
        {
            Vec p0;
            for (int i = 0; i < Dim; ++i)
            {
                p0[i] =
                    pt[i] < min[i] ? min[i] : (pt[i] > max[i] ? max[i] : pt[i]);
            }
            return (pt - p0).norm();
        }
    };

    struct BuildInfo
    {
        const Scalar *pts;
        Index *indices;
        Index byte_step;
        Index start, num;
        Index max_pts_per_leaf;

        VecCMap MapPt(Index i) const
        {
            VecCMap pt{reinterpret_cast<const Scalar *>(
                reinterpret_cast<const uint8_t *>(pts) + byte_step * i)};
            return pt;
        }
        VecCMap MapPtI(Index i) const { return MapPt(indices[i]); }
        VecCMap MapPtB(Index i) const { return MapPt(indices[start + i]); }
    };

    class Leaf : public ThisType
    {
    public:
        Index start, num;

        virtual void SetChild(int i, ThisType *c)
        {
            throw std::exception{"Leaf node has no children."};
        }
    };

    class NonLeaf : public ThisType
    {
    public:
        Box box;
        int split_axis;
        ThisType *left = nullptr;
        ThisType *right = nullptr;

        virtual void SetChild(int i, ThisType *c)
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
    };

    bool IsLeaf() const { return typeid(*this) == typeid(Leaf); }

    static Box ComputeBBox(const BuildInfo &info)
    {
        Vec min, max;
        for (int i = 0; i < Dim; ++i)
        {
            min[i] = std::numeric_limits<Scalar>::max();
            max[i] = std::numeric_limits<Scalar>::min();
        }
        for (Index i = 0; i < info.num; ++i)
        {
            VecCMap pt = info.MapPtB(i);
            for (int i = 0; i < Dim; ++i)
            {
                if (pt[i] < min[i])
                {
                    min[i] = pt[i];
                }
                if (pt[i] > max[i])
                {
                    max[i] = pt[i];
                }
            }
        }
        return Box{min, max};
    }

    static ThisType *Build(BuildInfo &info,
                           std::vector<BuildInfo> &children_infos)
    {
        children_infos.clear();
        if (info.num <= 0)
        {
            return nullptr;
        }
        if (info.num <= info.max_pts_per_leaf)
        {
            auto node = new Leaf;
            node->start = info.start;
            node->num = info.num;
            return node;
        }
        auto node = new NonLeaf;
        node->box = ComputeBBox(info);

        Vec box_size = node->box.max - node->box.min;
        int split_axis = static_cast<int>(
            std::max_element(box_size.data(), box_size.data() + Dim) -
            box_size.data());
        node->split_axis = split_axis;
        Index *s = info.indices + info.start;
        std::nth_element(s, s + info.num / 2, s + info.num,
                         [&](Index i1, Index i2) {
                             return info.MapPt(i1)[split_axis] <
                                    info.MapPt(i2)[split_axis];
                         });
        children_infos.resize(2);
        children_infos[0] = info;
        children_infos[1] = info;
        children_infos[0].start = info.start;
        children_infos[0].num = info.num / 2;
        children_infos[1].start = info.start + info.num / 2;
        children_infos[1].num = info.num - info.num / 2;

        return node;
    }

    virtual void SetChild(int i, ThisType *c) = 0;

    virtual ~KdTreeNode() = default;
};

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

    KdTree(const Scalar *pts, Index pt_num, Index *indices = nullptr,
           Index max_pts_per_leaf = 3, Index byte_step = 3 * sizeof(Scalar))
    {
        if (pt_num < 0)
        {
            throw std::exception{"pt num cannot be negative"};
        }
        build_info.pts = pts;
        build_info.start = (Index)0;
        build_info.num = pt_num;
        build_info.byte_step = byte_step;
        build_info.max_pts_per_leaf = max_pts_per_leaf;
        if (indices)
        {
            build_info.indices = indices;
        }
        else
        {
            this->indices.resize(pt_num);
            std::iota(this->indices.begin(), this->indices.end(), (Index)0);
            build_info.indices = const_cast<Index *>(this->indices.data());
        }
        Parent::CreatePreOrder(build_info);
    }

    KdTree(BuildInfo &info) : build_info{info}
    {
        Parent::CreatePreOrder(build_info);
    }

    template <typename Dis2Func> struct TraverseInfo
    {
        std::vector<std::pair<Scalar, Index>> knn;
        Dis2Func dis2_func;

        TraverseInfo(Dis2Func f) : dis2_func{std::move(f)} {}
    };

    void knn(Scalar *pt, Index k, std::vector<Index> &k_indices,
             std::vector<Scalar> &k_sqr_distances)
    {
        VecCMap pt_map{pt};
        auto dis2_func = [&](const VecCMap &p) -> Scalar
        { return (p - pt_map).sqauredNorm(); };
        auto info = TraverseInfo(dis2_func);

        auto cb = [this, &](Node *node) -> std::vector<Node *>
        {
            if (node->IsLeaf())
            {
                auto leaf = static_cast<Node::Leaf *>(node);
                for (Index i = 0; i < leaf->num; ++i)
                {
                    auto p = build_info.MapPtI[leaf->start + i];
                    Scalar dis2 = dis2_func(p);
                }
                return {};
            }
        };
        TraverseFromRoot(cb);
    }

protected:
    std::vector<Index> indices;
    BuildInfo build_info;
};

using KdTree3f = KdTree<float, 3, int>;
using KdTree3d = KdTree<double, 3, int>;

} // namespace alg