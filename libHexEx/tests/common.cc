#include "common.hh"

#include <iostream>

bool check_that_all_halffaces_referenced_by_cells_exists(HexEx::TetrahedralMesh& mesh)
{
  for (auto ch : mesh.cells())
  {
    auto hfs = mesh.cell(ch).halffaces();
    for (auto hfh : hfs)
      if (hfh.idx() >= (int)mesh.n_halffaces())
      {
        std::cout << "cell " << ch << " is bad. Halface " << hfh << " is incident even though there are only " << mesh.n_halffaces() << " halffaces." << std::endl;
        return false;
      }
  }

  return true;
}

HexEx::Vec3d getRandomVector()
{
    static std::default_random_engine e1(3);
    static std::uniform_int_distribution<int> uniform_dist1(-10, 10);

    auto res = HexEx::Vec3d(0,0,0);
    for (unsigned int i = 0; i < 3; ++i)
        res[i] = uniform_dist1(e1);

    return res;
}


HexEx::Vec3d getRandomVectorDouble()
{
    static std::default_random_engine e1(3);
    static std::uniform_real_distribution<double> uniform_dist1(-10, 10);

    auto res = HexEx::Vec3d(0,0,0);
    for (unsigned int i = 0; i < 3; ++i)
        res[i] = uniform_dist1(e1);

    return res;
}


HexEx::Matrix4x4d getRandomMatrix()
{
    static std::default_random_engine e1(7);
    static std::default_random_engine e2(8);
    static std::uniform_int_distribution<int> uniform_dist1(0, 1);
    static std::uniform_int_distribution<int> uniform_dist2(0, 2);

    auto value = uniform_dist1(e1)*2-1;
    auto pos   = uniform_dist2(e2);

    auto res = HexEx::Matrix4x4d();
    res.clear();

    res(0,pos) = value;

    int nextPos = pos;
    while (nextPos == pos)
        nextPos = uniform_dist2(e2);

    value = uniform_dist1(e1)*2-1;

    res(1,nextPos) = value;

    auto lastPos = (pos+1)%3;
    if (lastPos == nextPos)
        lastPos = (lastPos+1)%3;

    res(2,lastPos) = 1;
    res(3,3) = 1;

    if (res.determinant() < 0)
        res(2, lastPos) = -1;

    auto vec = getRandomVector();
    for (unsigned int i = 0; i < 3; ++i)
        res(i,3) = vec[i];

    return res;
}


