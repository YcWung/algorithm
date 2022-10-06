#pragma once

#include <Eigen/Geometry>
#include "data_wrap.hpp"

namespace alg
{

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
    BBox<Scalar, Dim> box;
    auto &min = box.min();
    auto &max = box.max();
    for (int i = 0; i < Dim; ++i)
    {
        min[i] = std::numeric_limits<Scalar>::max();
        max[i] = std::numeric_limits<Scalar>::min();
    }
    for (Index i = 0; i < pts.num; ++i)
    {
        auto pt = pts.MapVecS<Dim>(i);
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

template <typename Scalar, int Dim> class Cube
{
public:
    using Vec = Vec<Scalar, Dim>;
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

    Cube SubCube(const Vec &start, Scalar w)
    {
        Cube c;
        c.half_width = half_width * w;
        c.center = center + w * start;
        return c;
    }
};

} // namespace alg