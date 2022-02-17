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

using namespace HexEx;

class NavigationTest : public ::testing::Test {
protected:
    virtual void SetUp() {

        vhs.push_back(mesh.add_vertex(Vec3d(0,0,0)));
        vhs.push_back(mesh.add_vertex(Vec3d(1,0,0)));
        vhs.push_back(mesh.add_vertex(Vec3d(1,0,-1)));
        vhs.push_back(mesh.add_vertex(Vec3d(0,1,0)));

        hfhs.push_back(mesh.halfface_handle(mesh.add_face({vhs[0],vhs[1],vhs[2]}), 0));
        hfhs.push_back(mesh.halfface_handle(mesh.add_face({vhs[0],vhs[2],vhs[3]}), 0));
        hfhs.push_back(mesh.halfface_handle(mesh.add_face({vhs[0],vhs[3],vhs[1]}), 0));
        hfhs.push_back(mesh.halfface_handle(mesh.add_face({vhs[1],vhs[3],vhs[2]}), 0));

        ch  = mesh.add_cell({hfhs[0],hfhs[1],hfhs[2],hfhs[3]},true);

        OpenVolumeMesh::CellPropertyT<std::map<OpenVolumeMesh::VertexHandle, Vec3d>> parametrization = mesh.request_cell_property<std::map<OpenVolumeMesh::VertexHandle, Vec3d>>("Parametrization");
        mesh.set_persistent(parametrization);

        for (auto vh : mesh.vertices())
            parametrization[ch][vh] = mesh.vertex(vh);

        hexExtractor = new HexExtractor(mesh, parametrization);

        hexExtractor->parameter(ch,vhs[0]) = Parameter(  0, 0, 0);

  }

    virtual void TearDown() { delete hexExtractor; }

    HexExtractor* hexExtractor;
    std::vector<OpenVolumeMesh::VertexHandle> vhs;
    std::vector<OpenVolumeMesh::HalfFaceHandle> hfhs;
    OpenVolumeMesh::CellHandle ch;

    TetrahedralMesh mesh;
    TetrahedralMesh paramesh;
    PolyhedralMesh outputMesh;
    PolyhedralMesh irregEdglesMesh;
    PolyhedralMesh hportMesh;
    PolyhedralMesh intermediateMesh;
    HexahedralMesh realHexMesh;
};


TEST_F(NavigationTest, HalfedgeRotation) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0,-1);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0, 1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5,-1, 0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5, 1, 0);

    auto halfedge = mesh.halfedge(vhs[2],vhs[3]);

    auto traceHfh = hexEx.rotateAroundHalfedge(ch, halfedge, true);

    EXPECT_EQ(hfhs[3], traceHfh);

}


TEST_F(NavigationTest, HalfedgeRotationInverted) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0,-1);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0, 1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5, 1, 0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,-1, 0);

    auto halfedge = mesh.halfedge(vhs[2],vhs[3]);

    auto traceHfh = hexEx.rotateAroundHalfedge(ch, halfedge, true);

    EXPECT_EQ(hfhs[3], traceHfh);

}


TEST_F(NavigationTest, SimpleFacePiercing) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0, 0);
    hexEx.parameter(ch,vhs[1]) = Parameter(0.5,-1, 1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5,-1,-1);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5, 1, 0);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,1,0));


    EXPECT_EQ(hfhs[3], traceHfh);

}


TEST_F(NavigationTest, SimpleFacePiercingInverted) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0, 0);
    hexEx.parameter(ch,vhs[1]) = Parameter(0.5,-1, 1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5, 1, 0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,-1,-1);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,1,0));

    EXPECT_EQ(hfhs[3], traceHfh);

}



TEST_F(NavigationTest, EdgePiercing) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0,-1);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0, 1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5,-1, 0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5, 1, 0);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,-1,0));

    EXPECT_EQ(hfhs[3], traceHfh);

}

TEST_F(NavigationTest, EdgePiercingInverted) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0,-1);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0, 1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5, 1, 0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,-1, 0);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,-1,0));

    EXPECT_EQ(hfhs[3], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,1,0));

    EXPECT_EQ(hfhs[3], traceHfh);


    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0, 1);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0,-1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5,-1, 0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5, 1, 0);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,-1,0));

    EXPECT_EQ(hfhs[1], traceHfh);

}



TEST_F(NavigationTest, DoubleEdgePiercing) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0,-1);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0, 1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5,-1, 0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5, 1, 0);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,1));

    EXPECT_EQ(hfhs[3], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,-1));

    EXPECT_EQ(hfhs[1], traceHfh);

}

TEST_F(NavigationTest, DoubleEdgePiercingInverted) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0,-1);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0, 1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5, 1, 0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,-1, 0);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,1));

    EXPECT_EQ(hfhs[3], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,-1));

    EXPECT_EQ(hfhs[1], traceHfh);



    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0, 1);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0,-1);
    hexEx.parameter(ch,vhs[2]) = Parameter(0.5,-1, 0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5, 1, 0);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,1));

    EXPECT_EQ(hfhs[1], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,-1));

    EXPECT_EQ(hfhs[3], traceHfh);

}

TEST_F(NavigationTest, VertexPiercingSimple) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0,-0.5, 0.5);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0,-0.5,-0.5);
    hexEx.parameter(ch,vhs[2]) = Parameter(  0, 0.5,   0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,   0,   0);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,-1,0));

    EXPECT_EQ(hfhs[1], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,1,0));

    EXPECT_EQ(hfhs[1], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,-1), Direction(0,-1,0));

    EXPECT_EQ(hfhs[3], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,-1), Direction(0,1,0));

    EXPECT_EQ(hfhs[3], traceHfh);

}


TEST_F(NavigationTest, VertexPiercingSimpleInverted) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0.5, 0.5);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0.5,-0.5);
    hexEx.parameter(ch,vhs[2]) = Parameter(  0,-0.5,   0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,   0,   0);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,-1,0));

    EXPECT_EQ(hfhs[1], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,1,0));

    EXPECT_EQ(hfhs[1], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,-1), Direction(0,-1,0));

    EXPECT_EQ(hfhs[3], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,-1), Direction(0,1,0));

    EXPECT_EQ(hfhs[3], traceHfh);


    hexEx.parameter(ch,vhs[0]) = Parameter(  0,-0.5,-0.5);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0,-0.5, 0.5);
    hexEx.parameter(ch,vhs[2]) = Parameter(  0, 0.5,   0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,   0,   0);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,-1,0));

    EXPECT_EQ(hfhs[3], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,1), Direction(0,1,0));

    EXPECT_EQ(hfhs[3], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,-1), Direction(0,-1,0));

    EXPECT_EQ(hfhs[1], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,0,-1), Direction(0,1,0));

    EXPECT_EQ(hfhs[1], traceHfh);

}



TEST_F(NavigationTest, VertexPiercingWithEdge) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0,-0.5, 0.5);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0,-0.5,-0.5);
    hexEx.parameter(ch,vhs[2]) = Parameter(  0, 0.5,   0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,   0,   0);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,1));

    EXPECT_EQ(hfhs[1], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,-1));

    EXPECT_EQ(hfhs[3], traceHfh);

}



TEST_F(NavigationTest, VertexPiercingWithEdgeInverted) {

    auto& hexEx = *hexExtractor;

    hexEx.parameter(ch,vhs[0]) = Parameter(  0, 0.5, 0.5);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0, 0.5,-0.5);
    hexEx.parameter(ch,vhs[2]) = Parameter(  0,-0.5,   0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,   0,   0);

    auto traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,-1,0), Direction(0,0,-1));

    EXPECT_EQ(hfhs[3], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,-1,0), Direction(0,0,1));

    EXPECT_EQ(hfhs[1], traceHfh);


    hexEx.parameter(ch,vhs[0]) = Parameter(  0,-0.5,-0.5);
    hexEx.parameter(ch,vhs[1]) = Parameter(  0,-0.5, 0.5);
    hexEx.parameter(ch,vhs[2]) = Parameter(  0, 0.5,   0);
    hexEx.parameter(ch,vhs[3]) = Parameter(0.5,   0,   0);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,1));

    EXPECT_EQ(hfhs[3], traceHfh);

    traceHfh = hexEx.alpha0NextFace(HalfFaceHandle(), ch, Parameter(0,0,0), Direction(1,0,0), Direction(0,1,0), Direction(0,0,-1));

    EXPECT_EQ(hfhs[1], traceHfh);

}

