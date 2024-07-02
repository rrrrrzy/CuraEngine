// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "mesh.h"

#include <numbers>

#include <spdlog/spdlog.h>

#include "utils/Point3D.h"

/*
    Mesh 类的定义:
        Mesh 类是一个基本的 3D 模型表示,包含了所有的 MeshFace 对象。
        Mesh 类包含了两个主要的数据成员:
            vertices_: 存储所有顶点的列表
            faces_: 存储所有面的列表

    相关变量和函数:
        vertex_hash_map_:           一个哈希映射,用于快速查找具有相同位置的顶点。
        aabb_:                      3D 模型的轴对齐包围盒。
        settings_:                  与 Mesh 相关的设置。
        mesh_name_:                 Mesh 的名称。
        addFace():                  向 Mesh 添加一个新的面,但不设置其连接的面。
        clear():                    清除 Mesh 中的所有数据。
        finish():                   完成 Mesh 的构建,设置面之间的连接关系。
        min(), max() 和 getAABB():  获取 Mesh 的最小和最大坐标以及轴对齐包围盒。
        translate():                平移整个 Mesh。
        transform():                对 Mesh 应用仿射变换。
        isPrinted():                检查 Mesh 是否为可打印的网格。
        canInterlock():             检查 Mesh 是否可与其他网格互锁。
        findIndexOfVertex():        查找或创建一个给定点附近的顶点。
        getFaceIdxWithPoints():     根据给定的顶点索引查找共享边的相邻面的索引。

    功能和用处:
        Mesh 类提供了一种基本的 3D 模型表示方式,用于在计算机图形学和计算几何学领域进行各种操作,如网格压缩、简化、匹配等。
        该类支持对 Mesh 进行平移、旋转、缩放等变换,并提供了一些辅助函数,如获取包围盒、查找顶点等。
        该类还区分了可打印的网格和其他类型的网格(如填充网格、切片网格等),并提供了判断网格是否可互锁的功能。
        总的来说,Mesh 类是一个基础的 3D 模型表示,为上层的计算机图形学算法和应用提供了基础的数据结构和操作。

*/
namespace cura
{

const int vertex_meld_distance = MM2INT(0.03);
/*!
 * 下面给出"vertex_meld_distance"的定义: 
 * vertex_meld_distance 这个术语通常出现在计算机图形学和计算几何学领域,它是一种度量两个三角网格或网格顶点之间相似性的指标。

    具体来说:

    顶点融合距离 (Vertex Meld Distance):
        这是一种用于评估两个三角网格或网格之间相似性的度量方法。
        它通过计算网格中对应顶点之间的距离来衡量两个网格的相似程度。
        如果两个网格中对应顶点的位置非常接近,则它们的顶点融合距离会很小,表示两个网格高度相似。

    应用场景:
        在网格压缩、网格简化、网格匹配等计算机图形学领域广泛应用。
        可用于评估两个网格之间的差异,以决定是否需要进一步的网格优化或处理。
        也可用于在网格动画中检测网格变形,以确保动画的连续性和逼真性。

    计算方法:
        通常通过计算两个网格中对应顶点之间的欧几里德距离来得到顶点融合距离。
        也可以考虑其他因素,如法向量、纹理坐标等来增强度量的有效性。

    总之,vertex_meld_distance 是一种用于评估网格相似性的重要指标,在计算机图形学中有广泛的应用。它能帮助开发者更好地理解和处理三维网格数据。

 * returns a hash for the location, but first divides by the vertex_meld_distance,
 * so that any point within a box of vertex_meld_distance by vertex_meld_distance would get mapped to the same hash.
   这个哈希函数会返回一个位置的哈希值,但在此之前先会将该位置的坐标值除以一个叫做 vertex_meld_distance 的值。
   这样做的目的是,使得在一个 vertex_meld_distance 乘 vertex_meld_distance 大小的立体内的任意点,都会被映射到同一个哈希值。
 */
static inline uint32_t pointHash(const Point3LL& p)
/*
    下面说明类"Point3LL"的定义和作用：
    Point3LL 是一个3D点的数据结构。它包含三个成员变量 x、y 和 z，表示该点在3D空间中的坐标。
    这个类提供了一系列运算符重载,使得我们可以方便地对3D点进行各种数学运算,如加、减、乘、除等。
    它还提供了一些有用的方法,比如计算点的长度、点积等。

    这个类在3D建模、图形学、机器人学等领域都有广泛的应用。
    比如在3D打印中,需要处理各种3D模型,这些模型的顶点信息就可以用Point3LL来表示。
    在计算机图形学中,Point3LL可以用来表示3D场景中的顶点。
*/
{
    return ((p.x_ + vertex_meld_distance / 2) / vertex_meld_distance) ^ (((p.y_ + vertex_meld_distance / 2) / vertex_meld_distance) << 10)
         ^ (((p.z_ + vertex_meld_distance / 2) / vertex_meld_distance) << 20);
}

/*
    语法说明：初始化列表
    初始化列表是 C++ 中一个非常重要的语法特性,它用于在对象构造时初始化类成员变量。

    定义:
    初始化列表是位于构造函数头部的一个逗号分隔的列表,用于初始化对象的成员变量。它的语法形式如下:

    ClassName(parameters) : member1(expression1), member2(expression2), ... { 
        // 函数体
    }

    用法:
    初始化列表主要用于以下几种情况:

    初始化 const 成员变量:因为 const 成员变量必须在初始化时就赋值,而不能在函数体内赋值。
    初始化引用类型成员变量:引用必须在初始化时绑定到一个对象,不能在函数体内绑定。
    调用基类构造函数:当派生类需要调用基类的构造函数时,需要在初始化列表中进行调用。
    初始化 C 风格数组成员:数组成员必须在初始化列表中进行初始化。

    意义:

    提高初始化效率:初始化列表在对象构造时就完成了成员变量的初始化,避免了在函数体内再次赋值,提高了初始化效率。
    语义更清晰:初始化列表明确地表达了成员变量的初始化顺序和初始化方式,使代码更加易读和易维护。
    满足语言要求:某些情况下,如 const 成员变量和引用类型成员变量的初始化,只能通过初始化列表来完成。

*/
Mesh::Mesh(Settings& parent)
    : settings_(parent)
    , has_disconnected_faces(false)
    , has_overlapping_faces(false)
{
}

Mesh::Mesh()
    : settings_()
    , has_disconnected_faces(false)
    , has_overlapping_faces(false)
{
}

void Mesh::addFace(Point3LL& v0, Point3LL& v1, Point3LL& v2)
{
    int vi0 = findIndexOfVertex(v0);
    int vi1 = findIndexOfVertex(v1);
    int vi2 = findIndexOfVertex(v2);
    if (vi0 == vi1 || vi1 == vi2 || vi0 == vi2)
        return; // the face has two vertices which get assigned the same location. Don't add the face.

    int idx = faces_.size(); // index of face to be added
    faces_.emplace_back();
    MeshFace& face = faces_[idx];
    face.vertex_index_[0] = vi0;
    face.vertex_index_[1] = vi1;
    face.vertex_index_[2] = vi2;
    vertices_[face.vertex_index_[0]].connected_faces_.push_back(idx);
    vertices_[face.vertex_index_[1]].connected_faces_.push_back(idx);
    vertices_[face.vertex_index_[2]].connected_faces_.push_back(idx);
}

void Mesh::clear()
{
    faces_.clear();
    vertices_.clear();
    vertex_hash_map_.clear();
}

void Mesh::finish()
{
    // Finish up the mesh, clear the vertex_hash_map, as it's no longer needed from this point on and uses quite a bit of memory.
    vertex_hash_map_.clear();

    // For each face, store which other face is connected with it.
    for (unsigned int i = 0; i < faces_.size(); i++)
    {
        MeshFace& face = faces_[i];
        // faces are connected via the outside
        face.connected_face_index_[0] = getFaceIdxWithPoints(face.vertex_index_[0], face.vertex_index_[1], i, face.vertex_index_[2]);
        face.connected_face_index_[1] = getFaceIdxWithPoints(face.vertex_index_[1], face.vertex_index_[2], i, face.vertex_index_[0]);
        face.connected_face_index_[2] = getFaceIdxWithPoints(face.vertex_index_[2], face.vertex_index_[0], i, face.vertex_index_[1]);
    }
}

Point3LL Mesh::min() const
{
    return aabb_.min_;
}
Point3LL Mesh::max() const
{
    return aabb_.max_;
}
AABB3D Mesh::getAABB() const
{
    return aabb_;
}
void Mesh::expandXY(int64_t offset)
{
    if (offset)
    {
        aabb_.expandXY(offset);
    }
}

void Mesh::transform(const Matrix4x3D& transformation)
{
    for (MeshVertex& v : vertices_)
    {
        v.p_ = transformation.apply(v.p_);
    }
    aabb_.min_ = transformation.apply(aabb_.min_);
    aabb_.max_ = transformation.apply(aabb_.max_);
}


bool Mesh::isPrinted() const
{
    return ! settings_.get<bool>("infill_mesh") && ! settings_.get<bool>("cutting_mesh") && ! settings_.get<bool>("anti_overhang_mesh");
}

bool Mesh::canInterlock() const
{
    return ! settings_.get<bool>("infill_mesh") && ! settings_.get<bool>("anti_overhang_mesh");
}

int Mesh::findIndexOfVertex(const Point3LL& v)
{
    uint32_t hash = pointHash(v);

    for (unsigned int idx = 0; idx < vertex_hash_map_[hash].size(); idx++)
    {
        if ((vertices_[vertex_hash_map_[hash][idx]].p_ - v).testLength(vertex_meld_distance))
        {
            return vertex_hash_map_[hash][idx];
        }
    }
    vertex_hash_map_[hash].push_back(vertices_.size());
    vertices_.emplace_back(v);

    aabb_.include(v);

    return vertices_.size() - 1;
}

/*!
Returns the index of the 'other' face connected to the edge between vertices with indices idx0 and idx1.
In case more than two faces are connected via the same edge, the next face in a counter-clockwise ordering (looking from idx1 to idx0) is returned.

\cond DOXYGEN_EXCLUDE
    [NON-RENDERED COMENTS]
    For two faces abc and abd with normals n and m, we have that:
    \f{eqnarray*}{
    n &=& \frac{ab \times ac}{\|ab \times ac\|}     \\
    m &=& \frac{ab \times ad}{\|ab \times ad\|}     \\
    n \times m &=& \|n\| \cdot \|m\| \mathbf{p} \sin \alpha  \\
    && (\mathbf{p} \perp n \wedge \mathbf{p} \perp m) \\
    \sin \alpha &=& \|n \times m \|
    &=& \left\| \frac{(ab \times ac) \times (ab \times ad)}{\|ab \times ac\| \cdot \|ab \times ad\|}  \right\|    \\
    &=& \left\| \frac{ (ab \cdot (ac \times ad)) ab  }{\|ab \times ac\| \cdot \|ab \times ad\|}  \right\|    \\
    &=&  \frac{ (ab \cdot (ac \times ad)) \left\| ab   \right\| }{\|ab\| \|ac\| \sin bac \cdot \|ab\| \|ad\| \sin bad}    \\
    &=&  \frac{  ab \cdot (ac \times ad)  }{\|ab\| \|ac\| \|ad\|  \sin bac \sin bad}    \\
    \f}}
\endcond

See <a href="http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors">Direct way of computing clockwise angle between 2 vectors</a>


*/
int Mesh::getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx, int notFaceVertexIdx) const
{
    std::vector<int> candidateFaces; // in case more than two faces meet at an edge, multiple candidates are generated
    for (int f : vertices_[idx0].connected_faces_) // search through all faces connected to the first vertex and find those that are also connected to the second
    {
        if (f == notFaceIdx)
        {
            continue;
        }
        if (faces_[f].vertex_index_[0] == idx1 // && faces[f].vertex_index[1] == idx0 // next face should have the right direction!
            || faces_[f].vertex_index_[1] == idx1 // && faces[f].vertex_index[2] == idx0
            || faces_[f].vertex_index_[2] == idx1 // && faces[f].vertex_index[0] == idx0
        )
            candidateFaces.push_back(f);
    }

    if (candidateFaces.size() == 0)
    {
        spdlog::debug("Couldn't find face connected to face {}", notFaceIdx);
        if (! has_disconnected_faces)
        {
            spdlog::warn("Mesh has disconnected faces!");
        }
        has_disconnected_faces = true;
        return -1;
    }
    if (candidateFaces.size() == 1)
    {
        return candidateFaces[0];
    }


    if (candidateFaces.size() % 2 == 0)
    {
        spdlog::debug("Edge with uneven number of faces connecting it!({})\n", candidateFaces.size() + 1);
        if (! has_disconnected_faces)
        {
            spdlog::warn("Mesh has disconnected faces!");
        }
        has_disconnected_faces = true;
    }

    Point3D vn = vertices_[idx1].p_ - vertices_[idx0].p_;
    Point3D n = vn / vn.vSize(); // the normal of the plane in which all normals of faces connected to the edge lie => the normalized normal
    Point3D v0 = vertices_[idx1].p_ - vertices_[idx0].p_;

    // the normals below are abnormally directed! : these normals all point counterclockwise (viewed from idx1 to idx0) from the face, irrespective of the direction of the face.
    Point3D n0 = Point3D(vertices_[notFaceVertexIdx].p_ - vertices_[idx0].p_).cross(v0);

    if (n0.vSize() <= 0)
    {
        spdlog::debug("Face {} has zero area!", notFaceIdx);
    }

    double smallestAngle = 1000; // more then 2 PI (impossible angle)
    int bestIdx = -1;

    for (int candidateFace : candidateFaces)
    {
        int candidateVertex;
        { // find third vertex belonging to the face (besides idx0 and idx1)
            for (candidateVertex = 0; candidateVertex < 3; candidateVertex++)
                if (faces_[candidateFace].vertex_index_[candidateVertex] != idx0 && faces_[candidateFace].vertex_index_[candidateVertex] != idx1)
                    break;
        }

        Point3D v1 = vertices_[faces_[candidateFace].vertex_index_[candidateVertex]].p_ - vertices_[idx0].p_;
        Point3D n1 = v0.cross(v1);

        double dot = n0 * n1;
        double det = n * n0.cross(n1);
        double angle = std::atan2(det, dot);
        if (angle < 0)
            angle += 2 * std::numbers::pi; // 0 <= angle < 2* std::numbers::pi

        if (angle == 0)
        {
            spdlog::debug("Overlapping faces: face {} and face {}.", notFaceIdx, candidateFace);
            if (! has_overlapping_faces)
            {
                spdlog::warn("Mesh has overlapping faces!");
            }
            has_overlapping_faces = true;
        }
        if (angle < smallestAngle)
        {
            smallestAngle = angle;
            bestIdx = candidateFace;
        }
    }
    if (bestIdx < 0)
    {
        spdlog::debug("Couldn't find face connected to face {}.", notFaceIdx);
        if (! has_disconnected_faces)
        {
            spdlog::warn("Mesh has disconnected faces!");
        }
        has_disconnected_faces = true;
    }
    return bestIdx;
}

} // namespace cura
