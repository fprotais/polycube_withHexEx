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

#include <chrono>
#include <gtest/gtest.h>
#include "common.hh"
#include "Utils.hh"
#include <HexExtractor.hh>
#include <GridIsomorphism.hh>

using namespace HexEx;

TEST(GridIsomorphismTest, matrixTest) {

    double entries[] = {1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        3, 2, 0, 1 };
    auto t1 = Matrix4x4d(entries);

    Vec3d test = Vec3d(1,2,3);

    auto test2 = test;

    test2 = t1.transform_point(test2);

    EXPECT_EQ(test + Vec3d(3,2,0), test2);
}


TEST(GridIsomorphismTest, coversionTest) {

    for (auto i = 0; i < 24; i++)
    {
        auto rr = HexEx::RestrictedRotation(i);
        auto m = rr.toMatrix();

        auto rr2 = HexEx::RestrictedRotation(m);

        EXPECT_EQ(rr, rr2);
    }

}

TEST(GridIsomorphismTest, transformationTest) {

    for (auto i = 0; i < 24; i++)
    {
        auto rr = HexEx::RestrictedRotation(i);
        auto m = rr.toMatrix();

        for (auto j = 0; j < 100; ++j)
        {
            auto randomVec = HexEx::getRandomVector(10);
            EXPECT_EQ(m.transform_point(randomVec), rr.transform(randomVec)) << i;
        }
    }

}

TEST(GridIsomorphismTest, inversionTest) {

    for (char i = 0; i < 24; i++)
    {
        auto rr = HexEx::RestrictedRotation(i);
        auto m = rr.toMatrix();
        m.invert();

        auto rr2 = rr;
        rr2.invert();
        auto m2 = rr2.toMatrix();

        EXPECT_EQ(m2, m) << i;

    }

}

TEST(GridIsomorphismTest, multiplicationTest) {

    for (auto i = 0u; i < 24; i++)
        for (auto j = 0u; j < 24; j++)
        {
            auto rr1 = HexEx::RestrictedRotation(i);
            auto rr2 = HexEx::RestrictedRotation(j);

            auto res = rr1 * rr2;


            EXPECT_EQ(res.toMatrix(), rr1.toMatrix()*rr2.toMatrix()) << i << " " << j;

        }

}

TEST(GridIsomorphismTest, isomorphismInversionTest) {

    for (auto j = 0; j < 3; ++j)
        for (auto k = 0; k < 3; ++k)
            for (auto l = 0; l < 3; ++l)
            {
                auto t = HexEx::GridIsomorphism::Translation(j,k,l);
                for (auto i = 0; i < 24; i++)
                {
                    auto gi = HexEx::GridIsomorphism(i, t);
                    auto m = gi.toMatrix();
                    m.invert();

                    auto gi2 = gi.inverted();
                    auto m2 = gi2.toMatrix();

                    EXPECT_EQ(m2, m) << i;

                }
            }

}

//TEST(GridIsomorphismTest, performanceMulitplicitionTest) {

//    using namespace std::chrono;

//    auto N = 1000000;
//#ifdef DEBUG
//    N /= 100;
//#endif
//    auto gis = std::vector<HexEx::GridIsomorphism>();
//    for (auto i = 0; i < 24; ++i)
//        gis.push_back(HexEx::GridIsomorphism(i));

//    auto matrices = std::vector<Matrix4x4dd>();
//    for (auto i = 0; i < 24; ++i)
//        matrices.push_back(gis[i].toMatrix());

//    auto startGI = steady_clock::now();

//    auto res = HexEx::GridIsomorphism(0);
//    for (auto n = 0; n < N; ++n)
//    {
//        for (auto i = 0u; i < 24; i++)
//            for (auto j = 0u; j < 24; j++)
//            {
//                res = gis[i] * gis[j];
//            }
//    }

//    auto stopGI = steady_clock::now();

//    auto durationGI = duration_cast<duration<double>>(stopGI - startGI);

//    std::cout << res.transform_point(Vec3d(0,0,0)) << std::endl;

//    auto startMatrices = steady_clock::now();

//    auto res2 = Matrix4x4dd();
//    for (auto n = 0; n < N; ++n)
//    {
//        for (auto i = 0u; i < 24; i++)
//            for (auto j = 0u; j < 24; j++)
//            {
//                res2 = matrices[i] * matrices[j];
//            }
//    }

//    auto stopMatrices = steady_clock::now();

//    auto durationMatrices = duration_cast<duration<double>>(stopMatrices - startMatrices);

//    std::cout << res2.transform_point(Vec3d(0,0,0)) << std::endl;

//    std::cout << "Duration GridIsomorphism: " << durationGI.count() << std::endl;
//    std::cout << "Duration Matrices       : " << durationMatrices.count() << std::endl;
//    std::cout << "Ratio                   : " << static_cast<double>(durationGI.count())/durationMatrices.count() << std::endl;

//}




//TEST(GridIsomorphismTest, performanceInversionTest) {

//    using namespace std::chrono;

//    auto N = 10000000;
//#ifdef DEBUG
//    N /= 100;
//#endif
//    auto gis = std::vector<HexEx::GridIsomorphism>();
//    for (auto i = 0; i < 24; ++i)
//        gis.push_back(HexEx::GridIsomorphism(i));

//    auto matrices = std::vector<Matrix4x4dd>();
//    for (size_t i = 0; i < 24; ++i)
//        matrices.push_back(gis[i].toMatrix());

//    auto startGI = steady_clock::now();

//    for (auto n = 0; n < N; ++n)
//    {
//        for (auto i = 0u; i < 24; i++)
//            gis[i].invert();
//    }

//    auto stopGI = steady_clock::now();

//    auto durationGI = duration_cast<duration<double>>(stopGI - startGI);

//    auto startMatrices = steady_clock::now();

//    for (auto n = 0; n < N; ++n)
//    {
//        for (auto i = 0u; i < 24; i++)
//            matrices[i].invert();
//    }

//    auto stopMatrices = steady_clock::now();

//    auto durationMatrices = duration_cast<duration<double>>(stopMatrices - startMatrices);

//    std::cout << "Duration GridIsomorphism: " << durationGI.count() << std::endl;
//    std::cout << "Duration Matrices       : " << durationMatrices.count() << std::endl;
//    std::cout << "Ratio                   : " << (double)durationGI.count()/durationMatrices.count() << std::endl;

//}


//TEST(GridIsomorphismTest, performanceTransformationTest) {

//    using namespace std::chrono;

//    auto N = 10000000;
//#ifdef DEBUG
//    N /= 100;
//#endif
//    auto gis = std::vector<HexEx::GridIsomorphism>();
//    for (auto i = 0; i < 24; ++i)
//        gis.push_back(HexEx::GridIsomorphism(i));

//    auto matrices = std::vector<Matrix4x4dd>();
//    for (auto i = 0; i < 24; ++i)
//        matrices.push_back(gis[i].toMatrix());

//    auto startGI = steady_clock::now();

//    auto res = Vec3d(3,3,3);
//    for (auto n = 0; n < N; ++n)
//    {
//        for (auto i = 0u; i < 24; i++)
//            res += gis[i].transform_point(res);
//    }

//    auto stopGI = steady_clock::now();

//    auto durationGI = duration_cast<duration<double>>(stopGI - startGI);
//    std::cout << res << std::endl;

//    auto startMatrices = steady_clock::now();

//    auto res2 = Vec3d(3,3,3);
//    for (auto n = 0; n < N; ++n)
//    {
//        for (auto i = 0u; i < 24; i++)
//            res2 += matrices[i].transform_point(res2);
//    }

//    auto stopMatrices = steady_clock::now();

//    auto durationMatrices = duration_cast<duration<double>>(stopMatrices - startMatrices);

//    std::cout << res2 << std::endl;


//    std::cout << "Duration GridIsomorphism: " << durationGI.count() << std::endl;
//    std::cout << "Duration Matrices       : " << durationMatrices.count() << std::endl;
//    std::cout << "Ratio                   : " << (double)durationGI.count()/durationMatrices.count() << std::endl;

//}
