#include "HexExWrapper.h"

#define HEXEX_VERBOSE
#include <HexEx.hh>
//#include <OpenVolumeMesh/Mesh/HexahedralMeshIterators.hh>
//#include <OpenVolumeMesh/FileManager/FileManager.hh>
using namespace UM;



inline vec3 geom_swap(const HexEx::Vec3d& v) {
	return { v[0], v[1], v[2] };
}
inline HexEx::Vec3d geom_swap(const vec3& v) {
	return { v[0], v[1], v[2] };
}
inline void HexEx2Ultimaille(const HexEx::HexahedralMesh& in, Hexahedra& out) {
	out.clear();
	out.points.create_points(in.n_vertices());
	int ct = 0;
	for (auto it = in.vertices_begin(); it != in.vertices_end(); it++) {
		out.points[ct++] = geom_swap(in.vertex(*it));
	}
	out.create_cells(in.n_cells());
	constexpr int transition[8] = { 0, 1, 5, 4, 2, 6, 7, 3 };
	for (auto c_it = in.cells_begin(); c_it != in.cells_end(); ++c_it) {
		ct = 0;
		for (auto cv_it = OpenVolumeMesh::HexVertexIter(*c_it, &in); cv_it.valid(); ++cv_it) {
			out.vert(c_it->idx(), transition[ct++]) = cv_it->idx(); 
		}
	}
}
bool run_HexEx(const UM::Tetrahedra& m, const std::vector<UM::vec3>& corner_param, UM::Hexahedra& out) {
	HexEx::TetrahedralMesh tetMesh;
	std::vector<OpenVolumeMesh::VertexHandle> corners;
	for (int i : vert_iter(m)) {
		corners.push_back(tetMesh.add_vertex(geom_swap(m.points[i])));
	}
	auto parametrization = tetMesh.request_cell_property<std::map<OpenVolumeMesh::VertexHandle, HexEx::Vec3d> >("Parametrization");
	tetMesh.set_persistent(parametrization);
	// not doing it in function because i don't know the auto just up, and because of strange ordering, better to set the param as this: 
	for (int c : cell_iter(m)) {
		int T[4];
		for (int cv : range(4)) {
			T[cv] = m.vert(c, cv);
		}
		auto chexex = tetMesh.add_cell(corners[T[0]], corners[T[1]], corners[T[2]], corners[T[3]]);
		for (int cv : range(4)) {
			parametrization[chexex][corners[T[cv]]] = geom_swap(corner_param[4 * c + cv]);
		}
	}
	HexEx::HexahedralMesh hexMesh;
	std::cerr << "Running Hexex... ";
	HexEx::extractHexMesh(tetMesh, parametrization, hexMesh);
	std::cerr << "Done!" << std::endl;
	std::cout << "The extracted hex mesh has " << hexMesh.n_cells() << " cells, "
		<< hexMesh.n_faces() << " faces, " << hexMesh.n_edges() << " edges, and "
		<< hexMesh.n_vertices() << " vertices." << std::endl;

	HexEx2Ultimaille(hexMesh, out);
	return true;
}


