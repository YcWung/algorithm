#pragma once
#include <Eigen/Dense>

namespace alg
{

template <typename Scalar> class ArrayView
{
public:
    const Scalar *arr;
    int byte_step;

    template <int Dim> using Vec = Eigen::Matrix<Scalar, Dim, 1>;

    template <int Dim, typename Index = int>
    Eigen::Map<const Vec<Dim>> MapVec(Index i) const
    {
        return Eigen::Map<const Vec<Dim>>(reinterpret_cast<const Scalar *>(
            reinterpret_cast<const uint8_t *>(arr) + i * byte_step));
    }
};

template <typename Scalar, typename Index = int>
class ArrayIdicesView : public ArrayView<Scalar>
{
public:
    Index *indices;
    template <int Dim> using Vec = Eigen::Matrix<Scalar, Dim, 1>;

    template <int Dim, typename Index2 = int>
    Eigen::Map<const Vec<Dim>> MapVecI(Index2 i) const
    {
        return MapVec(indices[i]);
    }
};

} // namespace alg