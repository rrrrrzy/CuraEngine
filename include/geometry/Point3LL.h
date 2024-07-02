// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_POINT3LL_H
#define GEOMETRY_POINT3LL_H

#include <cassert>
#include <cmath> //For sqrt.
#include <iostream> //Auto-serialization.
#include <limits> //For numeric_limits::min and max.
#include <type_traits> // for operations on any arithmetic number type

#include "utils/Coord_t.h"
#include "utils/types/generic.h"

/*
Point3LL 是一个表示三维坐标的类,定义如下:

    成员变量:
        x_: 表示 x 坐标
        y_: 表示 y 坐标
        z_: 表示 z 坐标

    构造函数:
        默认构造函数 Point3LL()
        接收 3 个 coord_t 类型的参数的构造函数 Point3LL(const coord_t x, const coord_t y, const coord_t z)
        拷贝构造函数和移动构造函数

    重载运算符:
        加法运算符 operator+
        取负运算符 operator-
        减法运算符 operator-
        元素级乘法运算符 operator*
        除法运算符 operator/
        复合赋值运算符 operator+=, operator-=, operator*=, operator/=
        关系运算符 operator<=>
        流输出运算符 operator<<

    其他成员函数:
        max(): 返回三个坐标值中的最大值
        testLength(coord_t len): 检查三个坐标值是否都在 [-len, len] 范围内,并且向量长度的平方不超过 len^2
        vSize2(): 返回向量长度的平方
        vSize(): 返回向量长度
        vSizeMM(): 返回向量长度(单位为 mm)
        dot(const Point3LL& p): 返回两个向量的点积

    全局函数:
        operator*(const T i, const Point3LL& rhs): 实现 Point3LL 与标量的乘法运算

    特化 std::hash
        提供了 Point3LL 类型的哈希函数实现,用于在哈希表中存储 Point3LL 对象。

总的来说,Point3LL 类提供了一个方便地表示和操作三维坐标的数据结构,支持常见的数学运算,并且可以用于哈希表等数据结构中。这在三维几何计算中会非常有用。
*/



namespace cura
{

class Point3LL
{
public:
    coord_t x_{};
    coord_t y_{};
    coord_t z_{};

    Point3LL() = default;

    Point3LL(const coord_t x, const coord_t y, const coord_t z)
        : x_(x)
        , y_(y)
        , z_(z)
    {
    }

    Point3LL(Point3LL&& point) = default;
    Point3LL(const Point3LL& point) = default;
    Point3LL& operator=(const Point3LL& point) = default;
    Point3LL& operator=(Point3LL&& point) = default;

    virtual ~Point3LL() = default;

    Point3LL operator+(const Point3LL& p) const;
    Point3LL operator-() const;
    Point3LL operator-(const Point3LL& p) const;
    Point3LL operator*(const Point3LL& p) const; //!< Element-wise multiplication. For dot product, use .dot()!
    Point3LL operator/(const Point3LL& p) const;

    template<utils::numeric T>
    Point3LL operator*(const T& i) const
    {
        return { std::llround(static_cast<T>(x_) * i), std::llround(static_cast<T>(y_) * i), std::llround(static_cast<T>(z_) * i) };
    }

    template<utils::numeric T>
    Point3LL operator/(const T& i) const
    {
        return { x_ / i, y_ / i, z_ / i };
    }

    template<utils::numeric T>
    Point3LL operator%(const T& i) const
    {
        return { x_ % i, y_ % i, z_ % i };
    }

    Point3LL& operator+=(const Point3LL& p);
    Point3LL& operator-=(const Point3LL& p);
    Point3LL& operator*=(const Point3LL& p);
    Point3LL& operator/=(const Point3LL& p);

    template<utils::numeric T>
    Point3LL& operator*=(const T i)
    {
        x_ *= i;
        y_ *= i;
        z_ *= i;
        return *this;
    }

    template<utils::numeric T>
    Point3LL& operator/=(const T i)
    {
        x_ /= i;
        y_ /= i;
        z_ /= i;
        return *this;
    }

    auto operator<=>(const Point3LL&) const = default;

    template<class CharT, class TraitsT>
    friend std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& os, const Point3LL& p)
    {
        return os << "(" << p.x_ << ", " << p.y_ << ", " << p.z_ << ")";
    }

    [[nodiscard]] coord_t max() const
    {
        if (x_ > y_ && x_ > z_)
        {
            return x_;
        }
        if (y_ > z_)
        {
            return y_;
        }
        return z_;
    }

    [[nodiscard]] bool testLength(coord_t len) const
    {
        if (x_ > len || x_ < -len)
        {
            return false;
        }
        if (y_ > len || y_ < -len)
        {
            return false;
        }
        if (z_ > len || z_ < -len)
        {
            return false;
        }
        return vSize2() <= len * len;
    }

    [[nodiscard]] coord_t vSize2() const
    {
        return x_ * x_ + y_ * y_ + z_ * z_;
    }

    [[nodiscard]] coord_t vSize() const
    {
        return std::llrint(sqrt(static_cast<double>(vSize2())));
    }

    [[nodiscard]] double vSizeMM() const
    {
        double fx = INT2MM(x_);
        double fy = INT2MM(y_);
        double fz = INT2MM(z_);
        return sqrt(fx * fx + fy * fy + fz * fz);
    }

    [[nodiscard]] coord_t dot(const Point3LL& p) const
    {
        return x_ * p.x_ + y_ * p.y_ + z_ * p.z_;
    }

    coord_t& operator[](const size_t index)
    {
        assert(index < 3);
        switch (index)
        {
        case 0:
            return x_;
        case 1:
            return y_;
        default:
            return z_;
        }
    }
    const coord_t& operator[](const size_t index) const
    {
        return const_cast<Point3LL*>(this)->operator[](index);
    }
};

/*!
 * \brief Placeholder coordinate point (3D).
 *
 * Its value is something that is rarely used.
 */
static Point3LL no_point3(std::numeric_limits<coord_t>::min(), std::numeric_limits<coord_t>::min(), std::numeric_limits<coord_t>::min());

template<utils::numeric T>
inline Point3LL operator*(const T i, const Point3LL& rhs)
{
    return rhs * i;
}

} // namespace cura


namespace std
{
template<>
struct hash<cura::Point3LL>
{
    size_t operator()(const cura::Point3LL& pp) const noexcept
    {
        static int prime = 31;
        int result = 89;
        result = static_cast<int>(result * prime + pp.x_);
        result = static_cast<int>(result * prime + pp.y_);
        result = static_cast<int>(result * prime + pp.z_);
        return static_cast<size_t>(result);
    }
};
} // namespace std


#endif // GEOMETRY_POINT3LL_H
