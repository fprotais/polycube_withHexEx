# polycube with HexEx

Constructs a hexmesh from a flagged tetrahedral mesh with a polycube, using [HexEx](https://gitlab.vci.rwth-aachen.de:9000/HexEx/libHexEx). Somehow OpenVolumeMesh and HexEx don't look compatible anymore on my MVSC? I did some tweaking, and there is so much warning, it is a mystery that it works, but i guess it does...

The code take as input a tetmesh with a file containing the flagging of each cell's facet as an int (0 -> 5, {+X,-X,+Y,-Y,+Z,-Z}, -1 is no flagging). You can flag inner facets (differently that -1, they should always be in the flagging file). For the numbering of the facets, I use the following vertex numbering `{1,2,3}, {0,3,2}, {0,1,3}, {0,2,1}`.

# Use CMake to build the project:
```sh
git clone --recurse-submodules https://github.com/fprotais/polycube_withHexEx &&
cd polycube_withHexEx &&
mkdir build &&
cd build &&
cmake -DCMAKE_BUILD_TYPE=Release .. &&
make -j 
```

# Running the code :

```sh
./polycube ../S1.mesh ../S1.flags hexmesh.mesh 1.
```
For the supported mesh formats, see [ultimaille](https://github.com/ssloy/ultimaille). 
