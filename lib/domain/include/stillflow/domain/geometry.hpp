#pragma once
#include <array>
#include <cmath>

namespace stillflow {
struct Size { int width = 0, height = 0; };
struct Point {
    double x = 0, y = 0;
    Point operator+(Point p) const { return {x + p.x, y + p.y}; }
    Point operator-(Point p) const { return {x - p.x, y - p.y}; }
    Point operator*(double s) const { return {x * s, y * s}; }
    double dot(Point p) const { return x * p.x + y * p.y; }
};
inline double norm(Point p) { return std::sqrt(p.dot(p)); }
struct Matrix3 {
    std::array<double, 9> values{};
    double& operator()(int row, int col) { return values[row * 3 + col]; }
    double operator()(int row, int col) const { return values[row * 3 + col]; }
    static Matrix3 eye() { return {1, 0, 0, 0, 1, 0, 0, 0, 1}; }
    Matrix3 inv() const;
    Matrix3 operator*(const Matrix3& other) const;
};
inline Point transform(const Matrix3& m, Point p) {
    const double w = m(2, 0) * p.x + m(2, 1) * p.y + m(2, 2);
    return {(m(0, 0) * p.x + m(0, 1) * p.y + m(0, 2)) / w,
            (m(1, 0) * p.x + m(1, 1) * p.y + m(1, 2)) / w};
}
} // namespace stillflow
