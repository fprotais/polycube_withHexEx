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
#include <DerivedExactPredicates.hh>
#include <Direction.hh>

using namespace HexEx;

TEST(Predicates, PointsIntoTriangleFromEdgeTest) {
    using Vec3 = Vec3d;
    using Direction = HexEx::Direction;

    auto u = Vec3(4,5,0);
    auto v = Vec3(2,3,0);
    auto w = Vec3(0,0,0);
    auto p = Vec3(3,4,0);
    auto dir = Direction(1,0,0);

    EXPECT_TRUE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,dir));

}

TEST(Predicates, PointsIntoTriangleFromEdgeTest2) {
    using Vec3 = Vec3d;
    using Direction = HexEx::Direction;

    auto u = Vec3(2,0,0);
    auto v = Vec3(2,0,2);
    auto w = Vec3(4,0,0);
    auto p = Vec3(2,0,1);

    EXPECT_TRUE( pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 1, 0, 0)));
    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction(-1, 0, 0)));
    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 0, 1, 0)));
    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 0,-1, 0)));
    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 0, 0, 1)));
    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 0, 0,-1)));

}

TEST(Predicates, PointsIntoTriangleFromEdgeTest3) {
    using Vec3 = Vec3d;
    using Direction = HexEx::Direction;

    auto u = Vec3(4,0,0);
    auto v = Vec3(2,0,0);
    auto w = Vec3(0,0,0);
    auto p = Vec3(3,0,0);
    auto dir = Direction(1,0,0);

    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,dir));

}


TEST(Predicates, PointsIntoTriangleFromEdgeTest4) {
    using Vec3 = Vec3d;
    using Direction = HexEx::Direction;

    auto u = Vec3(4,0,0);
    auto v = Vec3(2,0,0);
    auto w = Vec3(0,-1,0);
    auto p = Vec3(3,0,0);
    auto dir = Direction(1,0,0);

    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,dir));
}



TEST(Predicates, PointsIntoTriangleFromEdgeTest5) {
    using Vec3 = Vec3d;
    using Direction = HexEx::Direction;

    auto u = Vec3(2,2,0);
    auto v = Vec3(2,2,2);
    auto w = Vec3(2,0,2);
    auto p = Vec3(2,2,1);

    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 1, 0, 0)));
    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction(-1, 0, 0)));
    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 0, 1, 0)));
    EXPECT_TRUE( pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 0,-1, 0)));
    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 0, 0, 1)));
    EXPECT_FALSE(pointsIntoProjectedTriangleFromEdge(u,v,w,p,Direction( 0, 0,-1)));
}

TEST(Predicates, FaceIntersectionTest)
{
    using Vec3 = Vec3d;

    auto u = Vec3(-1,0,-1);
    auto v = Vec3(0,-1,-1);
    auto w = Vec3(-1,-1,0);
    auto start = Vec3(-1,0,-1);
    auto end = Vec3(-1,0,0);

    EXPECT_TRUE(HexEx::intersectsTriangleRelaxed(u,v,w,start,end));

}


