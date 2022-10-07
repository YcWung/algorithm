/**
 * @file geom.hpp
 * @author Yongchao Wang (ycw.puzzle@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <Eigen/Geometry>
#include "data_wrap.hpp"

namespace alg
{

/*================================ typedefs ===============================*/

/**
 * @brief Bounding Box
 *
 * @tparam Scalar
 * @tparam Dim
 */
template <typename Scalar, int Dim> using BBox = Eigen::AlignedBox<Scalar, Dim>;

/**
 * @brief Math Vector
 *
 * @tparam Scalar
 * @tparam Dim
 */
template <typename Scalar, int Dim> using Vec = Eigen::Matrix<Scalar, Dim, 1>;
template <typename Scalar, int Dim>
using VecBase = Eigen::MatrixBase<Vec<Scalar, Dim>>;

/*=================================== concepts ============================*/

template <typename F, typename Scalar, int Dim, typename Index>
concept vec_get_t = requires(F f, Vec<Scalar, Dim> v)
{
    v = f((Index)0);
};

/*================================== BBox ================================*/

/**
 * @brief Compute the bounding box of a set of points
 *
 * @tparam Dim
 * @tparam Scalar
 * @tparam Index
 * @param pts
 * @return BBox<Scalar, Dim>
 */
template <int Dim, typename Scalar, typename Index>
BBox<Scalar, Dim> ComputeBBox(const ArrayIndicesView<Scalar, Index> &pts)
{
    return ComputeBBox<Dim, Scalar>(
        [&](Index i) { return pts.MapVecS<Dim>(i); }, pts.num);
}

template <int Dim, typename Scalar, typename Index>
BBox<Scalar, Dim> ComputeBBox(const ArrayView<Scalar, Index> &pts)
{
    return ComputeBBox<Dim, Scalar>([&](Index i) { return pts.MapVec<Dim>(i); },
                                    pts.num);
}

template <int Dim, typename Scalar, typename F, typename Index>
BBox<Scalar, Dim>
ComputeBBox(F get_pt, Index count) requires vec_get_t<F, Scalar, Dim, Index>
{
    BBox<Scalar, Dim> box;
    auto &min = box.min();
    auto &max = box.max();
    for (int i = 0; i < Dim; ++i)
    {
        min[i] = std::numeric_limits<Scalar>::max();
        max[i] = std::numeric_limits<Scalar>::min();
    }
    for (Index i = 0; i < count; ++i)
    {
        const auto &pt = get_pt(i);
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
    return box;
}

/**
 * @brief Compute the distance of a box and a outside point
 *
 * @tparam Scalar
 * @tparam Dim
 * @param box
 * @param pt
 * @return Scalar
 */
template <typename Scalar, int Dim>
Scalar OutDis2(const BBox<Scalar, Dim> &box, const Vec<Scalar, Dim> &pt)
{
    Vec<Scalar, Dim> p0;
    auto &min = box.min();
    auto &max = box.max();
    for (int i = 0; i < Dim; ++i)
    {
        p0[i] = pt[i] < min[i] ? min[i] : (pt[i] > max[i] ? max[i] : pt[i]);
    }
    return (pt - p0).squaredNorm();
}

/**
 * @brief Check if the point lies on the right hand side of a box
 *
 * @tparam Scalar
 * @tparam Dim
 * @param box
 * @param pt
 * @param axis
 * @return true
 * @return false
 */
template <typename Scalar, int Dim>
bool OnRightSide(const BBox<Scalar, Dim> &box, const Vec<Scalar, Dim> &pt,
                 int axis)
{
    return box.max()[axis] - pt[axis] > pt[axis] - box.min()[axis];
}

/*================================ Octant ================================*/

/**
 * @brief get octant direction of a vector. each component of the result vector
 * is +1 or -1
 *
 * @tparam Scalar
 * @tparam Dim
 * @param v
 * @return Vec<Scalar, Dim>
 */
template <typename Scalar, int Dim>
Vec<Scalar, Dim> OctantDir(const VecBase<Scalar, Dim> &v)
{
    Vec<Scalar, Dim> dir;
    for (int i = 0; i < Dim; ++i)
    {
        dir[i] = v[i] >= 0 ? (Scalar)1 : (Scalar)-1;
    }
    return dir;
}

/**
 * @brief get octant direction from octant index. Each component of the result
 * vector is either +1 or -1
 *
 * @tparam Scalar
 * @tparam Dim
 * @param idx octant index. Octant indices show a order of octants, for example,
 * (1,1), (-1, 1), (1, -1), (-1, -1)
 * @return Vec<Scalar, Dim>
 */
template <typename Scalar, int Dim> Vec<Scalar, Dim> OctantDir(int idx)
{
    Vec<Scalar, Dim> dir;
    for (int i = 0; i < Dim; ++i)
    {
        dir[i] = ((idx >> i) & 1) ? (Scalar)-1 : (Scalar)1;
    }
    return dir;
}

/**
 * @brief Get the octant index of a vector
 *
 * Octant indices show a order of octants, for example,
 * (1,1), (-1, 1), (1, -1), (-1, -1)
 *
 * @tparam Scalar
 * @tparam Dim
 * @param v
 * @return int
 */
template <typename Scalar, int Dim> int OctantIndex(const Vec<Scalar, Dim> &v)
{
    int idx = 0;
    for (int i = 0; i < Dim; ++i)
    {
        if (v[i] < 0)
        {
            idx |= (1 << i);
        }
    }
    return idx;
}

/*================================ Cube ==================================*/

/**
 * @brief N-dimentional cube with a center and a half_width
 *
 * @tparam Scalar
 * @tparam Dim
 */
template <typename Scalar, int Dim> class Cube
{
public:
    using Vec = Vec<Scalar, Dim>;
    using BBox = BBox<Scalar, Dim>;
    Vec center;
    Scalar half_width;

    Cube(const Vec &c = Vec::Zero(), Scalar hw = 1) : center{c}, half_width{hw}
    {
    }

    Cube(const BBox &box)
    {
        center = box.center();
        Vec hw = box.max() - center;
        auto it = std::max_element(hw.data(), hw.data() + Dim);
        half_width = *it;
    }

    Cube SubCube(const Vec &ct, Scalar w) const
    {
        Cube c;
        c.half_width = half_width * w;
        c.center = center + half_width * ct;
        return c;
    }

    Cube Octant(int i) const
    {
        Vec ct = OctantDir<Scalar, Dim>(i) * static_cast<Scalar>(0.5f);
        return SubCube(ct, (Scalar)0.5f);
    }

    Vec Min() const { return center - half_width * Vec::Ones(); }

    Vec Max() const { return center + half_width * Vec::Ones(); }

    BBox GetBBox() const { return {Min(), Max()}; }

    int LocateOctant(const Vec &p) const
    {
        return OctantIndex(Vec{p - center});
    }

    bool Contains(const Vec &p) const
    {
        bool ok = true;
        for (int i = 0; i < Dim; ++i)
        {
            if (std::abs(p[i] - center[i]) > half_width)
            {
                ok = false;
                break;
            }
        }
        return ok;
    }
};

} // namespace alg