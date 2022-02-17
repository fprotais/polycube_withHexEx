# polycube with HexEx

Constructs a hexmesh from a flagged tetrahedral mesh with a polycube, using [HexEx](https://gitlab.vci.rwth-aachen.de:9000/HexEx/libHexEx). Somehow OpenVolumeMesh and HexEx don't look compatible anymore on my MVSC? I did some tweaking, and there is so much warning, it is a mystery that it works, but i guess it does...

The code take as input a tetmesh with a file containing the flagging of each facet as an int (0 -> 5, {+X,-X,+Y,-Y,+Z,-Z}, -1 no flagging). You can flag inner facets. For the numbering of the facets, I use the following vertex numbering `{1,2,3}, {0,3,2}, {0,1,3}, {0,2,1}}`

# Use CMake to build the project:
```sh
git clone --recurse-submodules https://github.com/fprotais/polycube_withHexEx &&
cd fastbndpolycube &&
mkdir build &&
cd build &&
cmake .. &&
make -j 
```

# Running the code :

```sh
./polycube ../S1.mesh ../S1.flags hexmesh.mesh
```
For the supported mesh formats, see [ultimaille](https://github.com/ssloy/ultimaille). 
