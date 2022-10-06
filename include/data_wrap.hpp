#pragma once
#include <Eigen/Dense>

namespace alg
{

/**
 * @brief View a raw pointer as an array of math vector
 *
 * @tparam Scalar
 * @tparam Index
 */
template <typename Scalar, typename Index> class ArrayView
{
public:
    const Scalar *ptr;
    int byte_step;
    Index num;

    ArrayView() { Reset(nullptr, 0, 0); }

    ArrayView(const Scalar *ptr, int byte_step, Index num)
    {
        Reset(ptr, byte_step, num);
    }

    void Reset(const Scalar *ptr, int byte_step, Index num)
    {
        this->ptr = ptr;
        this->byte_step = byte_step;
        this->num = num;
    }

    template <int Dim> using Vec = Eigen::Matrix<Scalar, Dim, 1>;

    template <int Dim, typename Index2 = int>
    Eigen::Map<const Vec<Dim>> MapVec(Index2 i) const
    {
        return Eigen::Map<const Vec<Dim>>(reinterpret_cast<const Scalar *>(
            reinterpret_cast<const uint8_t *>(ptr) + i * byte_step));
    }
};

/**
 * @brief Add down-sampling capibility to ArrayView
 *
 * @tparam Scalar
 * @tparam Index
 */
template <typename Scalar, typename Index = int>
class ArrayIndicesView : public ArrayView<Scalar, Index>
{
    using Parent = ArrayView<Scalar, Index>;

public:
    Index *indices;
    Index start = 0;
    template <int Dim> using Vec = Eigen::Matrix<Scalar, Dim, 1>;

    ArrayIndicesView() { Reset(nullptr, 0, nullptr, 0, 0); }

    ArrayIndicesView(const Scalar *ptr, int byte_step, Index *indices,
                     Index start, Index num)
    {
        Reset(ptr, byte_step, indices, start, num);
    }

    void Reset(const Scalar *ptr, int byte_step, Index *indices, Index start,
               Index num)
    {
        this->ptr = ptr;
        this->byte_step = byte_step;
        this->num = num;
        this->indices = indices;
        this->num = num;
    }

    template <int Dim, typename Index2 = int>
    Eigen::Map<const Vec<Dim>> MapVecI(Index2 i) const
    {
        return Parent::MapVec<Dim, Index2>(indices[i]);
    }

    template <int Dim, typename Index2 = int>
    Eigen::Map<const Vec<Dim>> MapVecS(Index2 i) const
    {
        return Parent::MapVec<Dim, Index2>(indices[i + start]);
    }
};

} // namespace alg