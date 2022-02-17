/*
 * Copyright 2019 Computer Graphics Group, RWTH Aachen University
 * Author: Max Lyon <lyon@cs.rwth-aachen.de>
 *
 * This file is part of HexEx.
 *
 * HexEx is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * HexEx is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with HexEx.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>
#include "common.hh"
#include <HexExtractor.hh>
#include <FileAccessor.hh>
#include <sstream>
#include <stdio.h>

using namespace HexEx;

TEST(FileAccess, StreamTest) {

    for (auto transition : {false, true})
        for (auto size : {1.0,2.2,3 + 1.0/3.0,4.0,5.0})
        {

            TetrahedralMesh mesh;
            if (transition)
                createCubeWithTransition(mesh, size);
            else
                createCube(mesh, size);

            OpenVolumeMesh::CellPropertyT<std::map<VertexHandle, HexEx::Vec3d>> parametrization = mesh.template request_cell_property<std::map<OpenVolumeMesh::VertexHandle, HexEx::Vec3d>>("Parametrization");

            std::stringstream ss;
            ASSERT_NO_FATAL_FAILURE(HexEx::writeToStream(ss, mesh, parametrization));

            TetrahedralMesh mesh2;
            OpenVolumeMesh::CellPropertyT<std::map<VertexHandle, HexEx::Vec3d>> parametrization2 = mesh2.template request_cell_property<std::map<OpenVolumeMesh::VertexHandle, HexEx::Vec3d>>("Parametrization");

            ASSERT_NO_FATAL_FAILURE(HexEx::readFromStream(ss, mesh2, parametrization2));

            ASSERT_EQ(mesh.n_vertices(), mesh2.n_vertices());
            ASSERT_EQ(mesh.n_cells(), mesh2.n_cells());

            for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
                EXPECT_EQ(mesh.vertex(*v_it), mesh2.vertex(*v_it));

            for (auto c_it = mesh.cells_begin(); c_it != mesh.cells_end(); ++c_it)
                for (auto cv_it = mesh.cv_iter(*c_it); cv_it.valid(); ++cv_it)
                    EXPECT_EQ(parametrization[*c_it][*cv_it], parametrization2[*c_it][*cv_it]);

        }

}

TEST(FileAccess, FileTest) {

    std::string fileName = "asdfahdfgsdfgadsfggfadgfgvnvhkilprt.test";

    for (auto transition : {false, true})
        for (auto size : {1.0,2.2,3 + 1.0/3.0,4.0,5.0})
        {

            TetrahedralMesh mesh;
            if (transition)
                createCubeWithTransition(mesh, size);
            else
                createCube(mesh, size);

            OpenVolumeMesh::CellPropertyT<std::map<VertexHandle, HexEx::Vec3d>> parametrization = mesh.template request_cell_property<std::map<OpenVolumeMesh::VertexHandle, HexEx::Vec3d>>("Parametrization");

            ASSERT_NO_FATAL_FAILURE(HexEx::writeToFile(fileName, mesh, parametrization));

            TetrahedralMesh mesh2;
            OpenVolumeMesh::CellPropertyT<std::map<VertexHandle, HexEx::Vec3d>> parametrization2 = mesh2.template request_cell_property<std::map<OpenVolumeMesh::VertexHandle, HexEx::Vec3d>>("Parametrization");

            ASSERT_NO_FATAL_FAILURE(HexEx::readFromFile(fileName, mesh2, parametrization2));

            ASSERT_EQ(mesh.n_vertices(), mesh2.n_vertices());
            ASSERT_EQ(mesh.n_cells(), mesh2.n_cells());

            for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
                EXPECT_EQ(mesh.vertex(*v_it), mesh2.vertex(*v_it));

            for (auto c_it = mesh.cells_begin(); c_it != mesh.cells_end(); ++c_it)
                for (auto cv_it = mesh.cv_iter(*c_it); cv_it.valid(); ++cv_it)
                  EXPECT_EQ(parametrization[*c_it][*cv_it], parametrization2[*c_it][*cv_it]);

        }

    std::remove(fileName.c_str());

}
