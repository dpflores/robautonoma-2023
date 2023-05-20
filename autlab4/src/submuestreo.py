#!/usr/bin/env python3
import pcl
import os

# Cargar la nube de puntos
cloud = pcl.load_XYZRGB(os.path.join(os.path.dirname(__file__), '../data/mesa.pcd'))

# Crear un filtro VoxelGrid para la nube de puntos
fvox = cloud.make_voxel_grid_filter()
# Tamaño de voxel
VOXEL_SIZE = 0.005
# Asignar el tamaño del voxel al filtro
fvox.set_leaf_size(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE)

# Ejecutar el filtro
cloud_filtered = fvox.filter()

# Grabar el resultado en disco
filename = os.path.join(os.path.dirname(__file__), '../data/mesa_downsampled.pcd')
pcl.save(cloud_filtered, filename)

