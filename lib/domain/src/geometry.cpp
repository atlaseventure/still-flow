#include "stillflow/domain/geometry.hpp"
#include <Eigen/Dense>
#include <stdexcept>

namespace stillflow {
Matrix3 Matrix3::inv() const {
    using Matrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    const Eigen::Map<const Matrix> input(values.data());
    if (input.determinant() == 0)
        throw std::runtime_error("Singular motion transform");
    Matrix3 result;
    Eigen::Map<Matrix> output(result.values.data());
    output = input.inverse();
    return result;
}
Matrix3 Matrix3::operator*(const Matrix3& other) const {
    Matrix3 result;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            for (int k = 0; k < 3; ++k)
                result(r, c) += (*this)(r, k) * other(k, c);
    return result;
}
} // namespace stillflow
