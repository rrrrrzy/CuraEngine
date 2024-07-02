# CuraEngine 结构分析

> 引言：本文用于分析 CuraEngine 的代码结构以及功能分析，行文思路参考用户操作的流程

### 一、模型文件的导入

​	直接的导入函数在 src 目录下的 MeshGroup.cpp 里，程序会首先调用名为: `loadMeshIntoMeshGroup` 的函数，在该函数中又主要调用了 `loadMeshSTL`、`loadMeshSTL_ascii`和`loadMeshSTL_binary `下面分析这几个函数的参数及功能

```c++
bool loadMeshIntoMeshGroup(
  MeshGroup* meshgroup, // 初步认定为：CuraEngine将每个3D模型文件分为几个group，而他们对mesh的定义如下：A Mesh is the most basic representation of a 3D model. It contains all the faces as MeshFaces.  mesh是一个3d模型最基本的部分
  
  const char* filename, // 文件名称
  const Matrix4x3D& transformation, // Matrix4x3D定义为一个缩放矩阵(亦称“网格旋转矩阵”)，它存储了模型的坐标信息和缩放信息
  Settings& object_parent_settings
)
{
    TimeKeeper load_timer;

    const char* ext = strrchr(filename, '.');
    if (ext && (strcmp(ext, ".stl") == 0 || strcmp(ext, ".STL") == 0)) // 匹配文件后缀名
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
```
```c++
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
```
```c++
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
```
```c++
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
```

这里附一段最基本的立方体的 stl 文件描述 (ascii)

```stl
solid 10mmCube
  facet normal -0 0 1
    outer loop
      vertex 0 10 10
      vertex 10 0 10
      vertex 10 10 10
    endloop
  endfacet
  facet normal 0 0 1
    outer loop
      vertex 10 0 10
      vertex 0 10 10
      vertex 0 0 10
    endloop
  endfacet
  facet normal 0 0 -1
    outer loop
      vertex 0 0 0
      vertex 10 10 0
      vertex 10 0 0
    endloop
  endfacet
  facet normal -0 0 -1
    outer loop
      vertex 10 10 0
      vertex 0 0 0
      vertex 0 10 0
    endloop
  endfacet
  facet normal 0 -1 0
    outer loop
      vertex 0 0 0
      vertex 10 0 10
      vertex 0 0 10
    endloop
  endfacet
  facet normal 0 -1 -0
    outer loop
      vertex 10 0 10
      vertex 0 0 0
      vertex 10 0 0
    endloop
  endfacet
  facet normal 1 -0 0
    outer loop
      vertex 10 0 10
      vertex 10 10 0
      vertex 10 10 10
    endloop
  endfacet
  facet normal 1 0 0
    outer loop
      vertex 10 10 0
      vertex 10 0 10
      vertex 10 0 0
    endloop
  endfacet
  facet normal 0 1 -0
    outer loop
      vertex 10 10 0
      vertex 0 10 10
      vertex 10 10 10
    endloop
  endfacet
  facet normal 0 1 0
    outer loop
      vertex 0 10 10
      vertex 10 10 0
      vertex 0 10 0
    endloop
  endfacet
  facet normal -1 0 0
    outer loop
      vertex 0 0 0
      vertex 0 10 10
      vertex 0 10 0
    endloop
  endfacet
  facet normal -1 -0 0
    outer loop
      vertex 0 10 10
      vertex 0 0 0
      vertex 0 0 10
    endloop
  endfacet
endsolid 10mmCube

```

