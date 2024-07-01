// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "MeshGroup.h"

#include <limits>
#include <stdio.h>
#include <string.h>

#include <fmt/format.h>
#include <range/v3/view/enumerate.hpp>
#include <scripta/logger.h>
#include <spdlog/spdlog.h>

#include "settings/types/Ratio.h" //For the shrinkage percentage and scale factor.
#include "utils/Matrix4x3D.h" //To transform the input meshes for shrinkage compensation and to align in command line mode.
#include "utils/Point3F.h" //To accept incoming meshes with floating point vertices.
#include "utils/gettime.h"
#include "utils/section_type.h"
#include "utils/string.h"

namespace cura
{

FILE* binaryMeshBlob = nullptr;

/*  Custom fgets function to support Mac line-ends in Ascii STL files. OpenSCAD produces this when used on Mac 
    从文件中读取一行数据（直到\n或\r或达到指定长度len），并将其作为以 \0 结束的字符串存储在 ptr 所指向的内存中。
*/
void* fgets_(char* ptr, size_t len, FILE* f)
{
    while (len && fread(ptr, 1, 1, f) > 0)
    {
        if (*ptr == '\n' || *ptr == '\r')
        {
            *ptr = '\0';
            return ptr;
        }
        ptr++;
        len--;
    }
    return nullptr;
}

Point3LL MeshGroup::min() const
{
    if (meshes.size() < 1)
    {
        return Point3LL(0, 0, 0);
    }
    Point3LL ret(std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max());
    for (const Mesh& mesh : meshes)
    {
        if (mesh.settings_.get<bool>("infill_mesh") || mesh.settings_.get<bool>("cutting_mesh")
            || mesh.settings_.get<bool>("anti_overhang_mesh")) // Don't count pieces that are not printed.
        {
            continue;
        }
        Point3LL v = mesh.min();
        ret.x_ = std::min(ret.x_, v.x_);
        ret.y_ = std::min(ret.y_, v.y_);
        ret.z_ = std::min(ret.z_, v.z_);
    }
    return ret;
}

Point3LL MeshGroup::max() const
{
    if (meshes.size() < 1)
    {
        return Point3LL(0, 0, 0);
    }
    Point3LL ret(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min());
    for (const Mesh& mesh : meshes)
    {
        if (mesh.settings_.get<bool>("infill_mesh") || mesh.settings_.get<bool>("cutting_mesh")
            || mesh.settings_.get<bool>("anti_overhang_mesh")) // Don't count pieces that are not printed.
        {
            continue;
        }
        Point3LL v = mesh.max();
        ret.x_ = std::max(ret.x_, v.x_);
        ret.y_ = std::max(ret.y_, v.y_);
        ret.z_ = std::max(ret.z_, v.z_);
    }
    return ret;
}

void MeshGroup::clear()
{
    for (Mesh& m : meshes)
    {
        m.clear();
    }
}

void MeshGroup::finalize()
{
    // If the machine settings have been supplied, offset the given position vertices to the center of vertices (0,0,0) is at the bed center.
    Point3LL meshgroup_offset(0, 0, 0);
    if (! settings.get<bool>("machine_center_is_zero"))
    {
        meshgroup_offset.x_ = settings.get<coord_t>("machine_width") / 2;
        meshgroup_offset.y_ = settings.get<coord_t>("machine_depth") / 2;
    }

    // If a mesh position was given, put the mesh at this position in 3D space.
    for (Mesh& mesh : meshes)
    {
        Point3LL mesh_offset(mesh.settings_.get<coord_t>("mesh_position_x"), mesh.settings_.get<coord_t>("mesh_position_y"), mesh.settings_.get<coord_t>("mesh_position_z"));
        if (mesh.settings_.get<bool>("center_object"))
        {
            Point3LL object_min = mesh.min();
            Point3LL object_max = mesh.max();
            Point3LL object_size = object_max - object_min;
            mesh_offset += Point3LL(-object_min.x_ - object_size.x_ / 2, -object_min.y_ - object_size.y_ / 2, -object_min.z_);
        }
        mesh.translate(mesh_offset + meshgroup_offset);
    }
    scaleFromBottom(
        settings.get<Ratio>("material_shrinkage_percentage_xy"),
        settings.get<Ratio>("material_shrinkage_percentage_z")); // Compensate for the shrinkage of the material.
    for (const auto& [idx, mesh] : meshes | ranges::views::enumerate)
    {
        scripta::log(fmt::format("mesh_{}", idx), mesh, SectionType::NA);
    }
}

void MeshGroup::scaleFromBottom(const Ratio factor_xy, const Ratio factor_z)
{
    const Point3LL center = (max() + min()) / 2;
    const Point3LL origin(center.x_, center.y_, 0);

    const Matrix4x3D transformation = Matrix4x3D::scale(factor_xy, factor_xy, factor_z, origin);
    for (Mesh& mesh : meshes)
    {
        mesh.transform(transformation);
    }
}

bool loadMeshSTL_ascii(Mesh* mesh, const char* filename, const Matrix4x3D& matrix)
{
    FILE* f = fopen(filename, "rt");
    char buffer[1024];
    Point3F vertex;
    int n = 0;
    Point3LL v0(0, 0, 0), v1(0, 0, 0), v2(0, 0, 0);
    while (fgets_(buffer, sizeof(buffer), f)) // 每次读取1024个字节且当返回值不为nullptr时
    {
        if (sscanf(buffer, " vertex %f %f %f", &vertex.x_, &vertex.y_, &vertex.z_) == 3) 
        // 总之，这一段读入了所有的坐标信息
        {
            n++;
            switch (n)
            {
            case 1:
                v0 = matrix.apply(vertex.toPoint3d());
                break;
            case 2:
                v1 = matrix.apply(vertex.toPoint3d());
                break;
            case 3:
                v2 = matrix.apply(vertex.toPoint3d());
                mesh->addFace(v0, v1, v2);
                n = 0;
                break;
            }
        }
    }
    fclose(f);
    mesh->finish();
    return true;
}

bool loadMeshSTL_binary(Mesh* mesh, const char* filename, const Matrix4x3D& matrix)
{
    FILE* f = fopen(filename, "rb");

    fseek(f, 0L, SEEK_END); // 将文件指针f移动到文件末尾
    long long file_size = ftell(f); // The file size is the position of the cursor(光标) after seeking to the end.
    rewind(f); // Seek back to start.
    size_t face_count = (file_size - 80 - sizeof(uint32_t)) / 50; 
    // Subtract the size of the header. Every face uses exactly 50 bytes.

    char buffer[80];
    // Skip the header
    if (fread(buffer, 80, 1, f) != 1)
    {
        fclose(f);
        return false;
    }

    uint32_t reported_face_count;
    // Read the face count. We'll use it as a sort of redundancy code to check for file corruption.
    if (fread(&reported_face_count, sizeof(uint32_t), 1, f) != 1)
    {
        fclose(f);
        return false;
    }
    if (reported_face_count != face_count)
    {
        spdlog::warn("Face count reported by file ({}) is not equal to actual face count ({}). File could be corrupt!", reported_face_count, face_count);
    }

    // For each face read:
    // float(x,y,z) = normal, float(X,Y,Z)*3 = vertexes, uint16_t = flags
    //  Every Face is 50 Bytes: Normal(3*float), Vertices(9*float), 2 Bytes Spacer
    mesh->faces_.reserve(face_count);
    mesh->vertices_.reserve(face_count);
    for (size_t i = 0; i < face_count; i++)
    {
        if (fread(buffer, 50, 1, f) != 1)
        {
            fclose(f);
            return false;
        }
        float* v = reinterpret_cast<float*>(buffer) + 3;

        Point3LL v0 = matrix.apply(Point3F(v[0], v[1], v[2]).toPoint3d());
        Point3LL v1 = matrix.apply(Point3F(v[3], v[4], v[5]).toPoint3d());
        Point3LL v2 = matrix.apply(Point3F(v[6], v[7], v[8]).toPoint3d());
        mesh->addFace(v0, v1, v2);
    }
    fclose(f);
    mesh->finish();
    return true;
}

bool loadMeshSTL(Mesh* mesh, const char* filename, const Matrix4x3D& matrix)
{
    FILE* f = fopen(filename, "rb");
    if (f == nullptr)
    {
        return false;
    }

    // assign filename to mesh_name
    mesh->mesh_name_ = filename;

    // Skip any whitespace at the beginning of the file.
    unsigned long long num_whitespace = 0; // Number of whitespace characters.
    unsigned char whitespace;
    if (fread(&whitespace, 1, 1, f) != 1) // 预先读取一个空白符，如果有则返回1，没有说明文件为空
    {
        fclose(f);
        return false;
    }
    while (isspace(whitespace)) // 总之，这一段读取文件头部的所有空白符并计数，直到遇到第一个非空白符
    {
        num_whitespace++;
        if (fread(&whitespace, 1, 1, f) != 1)
        {
            fclose(f);
            return false;
        }
    }
    fseek(f, num_whitespace, SEEK_SET); // Seek to the place after all whitespace (we may have just read too far).
                                        // 这样之后，就跳转到了第一个非空白符的位置
    char buffer[6];
    if (fread(buffer, 5, 1, f) != 1)    // 读取一个5字节大小的数据，如果读取失败，退出
    {
        fclose(f);
        return false;
    }
    fclose(f);

    buffer[5] = '\0';                   // 把字符串数组中最后一个元素定义为终止元素“\0”，避免下面进行strcmp时发生错误
    if (stringcasecompare(buffer, "solid") == 0) // 该情况表明读取的5字节数据是“solid”
    {
        bool load_success = loadMeshSTL_ascii(mesh, filename, matrix); // 进一步读取以ascii码存储的stl文件
        if (! load_success)
            return false;

        // This logic is used to handle the case where the file starts with
        // "solid" but is a binary file.
        // 分析一下这里是怎么判断出来的：二进制模式下会导致loadMeshSTL_ascii函数读取不了任何信息，
        // 但这个函数无论如何都会返回true，因此mesh->faces_.size()如果小于1就表明没有读取到任何坐标信息，故为二进制模式
        if (mesh->faces_.size() < 1)
        {
            mesh->clear();
            return loadMeshSTL_binary(mesh, filename, matrix); 
        }
        return true;
    }
    return loadMeshSTL_binary(mesh, filename, matrix);
}

bool loadMeshIntoMeshGroup(MeshGroup* meshgroup, const char* filename, const Matrix4x3D& transformation, Settings& object_parent_settings)
{
    TimeKeeper load_timer;

    const char* ext = strrchr(filename, '.');
    if (ext && (strcmp(ext, ".stl") == 0 || strcmp(ext, ".STL") == 0))
    {
        Mesh mesh(object_parent_settings);
        if (loadMeshSTL(&mesh, filename, transformation)) // Load it! If successful...
        {
            meshgroup->meshes.push_back(mesh);
            spdlog::info("loading '{}' took {:03.3f} seconds", filename, load_timer.restart());
            return true;
        }
    }
    spdlog::warn("Unable to recognize the extension of the file. Currently only .stl and .STL are supported.");
    return false;
}

} // namespace cura
