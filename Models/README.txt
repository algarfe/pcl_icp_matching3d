In order to create an OBJ file with normals which can be loaded into PCL PointNormal PC type, 
it is necessary to make sure that all normals are associated to vertices and not to faces.
If you are generating the OBJ file with blender, as it was this repository's case, these are the steps you will need to follow:
1-Select the object you want to export as OBJ and apply smooth shading (Tools -> Shading -> Smooth). Otherwise, all normals will be face-wise and not vertex-wise.
2-Export as OBJ file, selecting only the following options: 
	-Include Edges
	-Write Materials
	-Objects as OBJ Objects (not sure if this one is necessary though, but not harmful at least)
