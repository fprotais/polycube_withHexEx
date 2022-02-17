#include <string>
#include <iostream>
#include <fstream>
#include <ultimaille/all.h>
#include <OpenNL_psm/OpenNL_psm.h>

#include "HexExWrapper.h"

using namespace UM;
#define FOR(i, n) for(int i = 0; i < n; i++)


void float_cubecover(const Tetrahedra & m, const CellFacetAttribute<int>&flag, PointAttribute<vec3>&U) {
	FOR(v, m.nverts()) U[v] = m.points[v];
	std::cerr << "Integrating with float boundary...\n";
	DisjointSet ds(m.nverts() * 3);
	FOR(c, m.ncells()) FOR(cf, 4) if (flag[4 * c + cf] != -1) {
		int d = flag[4 * c + cf] / 2;
		FOR(cfv, 3) ds.merge(d * m.nverts() + m.facet_vert(c, cf, cfv), d * m.nverts() + m.facet_vert(c, cf, (cfv + 1) % 3));
	}
	std::vector<int> idmap;
	int nb_var = ds.get_sets_id(idmap);

	auto context = nlNewContext();
	nlSolverParameteri(NL_LEAST_SQUARES, NL_TRUE);
	nlSolverParameteri(NL_NB_VARIABLES, NLint(nb_var));

	nlBegin(NL_SYSTEM);
	nlEnable(NL_VERBOSE);
	nlBegin(NL_MATRIX);
	FOR(c, m.ncells()) {
		int v[4] = { m.vert(c,0) , m.vert(c,1), m.vert(c,2), m.vert(c,3) };
		UM::mat3x3 M = { m.points[v[1]] - m.points[v[0]], m.points[v[2]] - m.points[v[0]], m.points[v[3]] - m.points[v[0]] };
		UM::mat3x3 invM = M.invert();
		invM = invM.transpose();
		mat<4, 3> grad_coef = { -invM[0] - invM[1] - invM[2], invM[0], invM[1], invM[2] };
		FOR(dim, 3) {
			FOR(dim2, 3) {
				vec3  e(0, 0, 0); e[dim2] = 1;
				nlBegin(NL_ROW);
				FOR(dim_e, 3) FOR(point, 4) {
					nlCoefficient(idmap[dim * m.nverts() + v[point]], e[dim_e] * grad_coef[point][dim_e]);
				}
				if (dim == dim2) nlRightHandSide(1);
				nlEnd(NL_ROW);
			}
		}
	}


	nlEnd(NL_MATRIX);
	nlEnd(NL_SYSTEM);
	nlSolve();

	FOR(v, m.nverts()) FOR(dim, 3) U[v][dim] = nlGetVariable(idmap[m.nverts() * dim + v]);

	nlDeleteContext(context);
	std::cerr << " Done.\n";

}


void integer_cubecover(const Tetrahedra & m, const CellFacetAttribute<int>&flag, const PointAttribute<vec3>& U, PointAttribute<vec3>& int_U) {
	std::cerr << "Integrating with int boundary...\n";
	DisjointSet ds(m.nverts() * 3);
	FOR(c, m.ncells()) FOR(cf, 4) if (flag[4 * c + cf] != -1) {
		int d = flag[4 * c + cf] / 2;
		FOR(cfv, 3) ds.merge(d * m.nverts() + m.facet_vert(c, cf, cfv), d * m.nverts() + m.facet_vert(c, cf, (cfv + 1) % 3));
	}
	std::vector<int> idmap;
	int nb_var = ds.get_sets_id(idmap);

	auto context = nlNewContext();
	nlSolverParameteri(NL_LEAST_SQUARES, NL_TRUE);
	nlSolverParameteri(NL_NB_VARIABLES, NLint(nb_var));

	nlBegin(NL_SYSTEM);
	FOR(c, m.ncells()) FOR(cf, 4) if (flag[4 * c + cf] != -1) {
		int d = flag[4 * c + cf] / 2;
		FOR(cfv, 3) {
			nlSetVariable(idmap[d * m.nverts() + m.facet_vert(c, cf, cfv)], std::round(U[m.facet_vert(c, cf, cfv)][d]));
			nlLockVariable(idmap[d * m.nverts() + m.facet_vert(c, cf, cfv)]);
		}
	}
	nlEnable(NL_VERBOSE);
	nlBegin(NL_MATRIX);
	FOR(c, m.ncells()) {
		int v[4] = { m.vert(c,0) , m.vert(c,1), m.vert(c,2), m.vert(c,3) };
		UM::mat3x3 M = { m.points[v[1]] - m.points[v[0]], m.points[v[2]] - m.points[v[0]], m.points[v[3]] - m.points[v[0]] };
		UM::mat3x3 invM = M.invert();
		invM = invM.transpose();
		mat<4, 3> grad_coef = { -invM[0] - invM[1] - invM[2], invM[0], invM[1], invM[2] };
		FOR(dim, 3) {
			FOR(dim2, 3) {
				vec3  e(0, 0, 0); e[dim2] = 1;
				nlBegin(NL_ROW);
				FOR(dim_e, 3) FOR(point, 4) {
					nlCoefficient(idmap[dim * m.nverts() + v[point]], e[dim_e] * grad_coef[point][dim_e]);
				}
				if (dim == dim2) nlRightHandSide(1);
				nlEnd(NL_ROW);
			}
		}
	}


	nlEnd(NL_MATRIX);
	nlEnd(NL_SYSTEM);
	nlSolve();

	FOR(v, m.nverts()) FOR(dim, 3) int_U[v][dim] = nlGetVariable(idmap[m.nverts() * dim + v]);

	nlDeleteContext(context);
	std::cerr << " Done.\n";
}

double rescaling(Tetrahedra& m) {
	double size = 0;
	FOR(c, m.ncells()) FOR(cf, 4) FOR(cfv, 3) {
		size += (m.points[m.facet_vert(c, cf, cfv)] - m.points[m.facet_vert(c, cf, (cfv + 1) % 3)]).norm();
	}
	size /= m.ncells() * 12;
	FOR(v, m.nverts()) m.points[v] /= size;
	return size;
}

void revert_rescaling(PointSet& m, double sizing) {
	FOR(v, m.size()) m[v] *= sizing;
}

int main(int argc, char** argv) {

	if (argc != 4) {
		std::cout << "Usage is: " << argv[0] << " tetmesh.ext flagfile deformedoutput.ext" << std::endl;
		return 1;
	}
	std::string inputfile = argv[1];
	std::string flagfile = argv[2];
	std::string outputfile = argv[3];

	
	Tetrahedra m;
	read_by_extension(inputfile, m);
	m.delete_isolated_vertices();
	CellFacetAttribute<int> flag(m);
	std::ifstream ifs(flagfile);
	if (!ifs.is_open()) {
		std::cerr << "Failed opening of flags at : " << flagfile << std::endl;
		abort();
	}
	FOR(cf, m.ncells() * 4) {
		if (ifs.eof()) flag[cf] = -1;
		else ifs >> flag[cf];
	}
	ifs.close();


	PointAttribute<vec3> U(m), int_U(m);
	double sizing = rescaling(m);

	float_cubecover(m, flag, U);

	integer_cubecover(m, flag, U, int_U);

	revert_rescaling(m.points, sizing);


	write_by_extension("Param.geogram", m, { {{"U", U.ptr}, {"int_U", int_U.ptr}},{},{},{{"flag", flag.ptr}}});
	FOR(v, m.nverts()) std::swap(int_U[v], m.points[v]);
	write_by_extension("Polycube.geogram", m, { {{"original", int_U.ptr}},{},{},{{"flag", flag.ptr}} });
	FOR(v, m.nverts()) std::swap(int_U[v], m.points[v]);

	Hexahedra hex;
	std::vector<vec3> corner_param(m.ncorners());
	FOR(c, m.ncells()) FOR(cc, 4) corner_param[4 * c + cc] = int_U[m.vert(c, cc)];
	run_HexEx(m, corner_param, hex);
	write_by_extension(outputfile, hex);

	return 0;
}

