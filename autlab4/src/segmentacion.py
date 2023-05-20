# -*- coding: utf-8 -*-

import pcl
import os

# Cargar la nube de puntos
cloud = pcl.load_XYZRGB(os.path.join(os.path.dirname(__file__), '../data/mesa_filtered.pcd'))

# Crear el objeto para la segmentación
seg = cloud.make_segmenter()

# Asignar el modelo que se desea ajustar
seg.set_model_type(pcl.SACMODEL_PLANE)
# Uso de RANSAC
seg.set_method_type(pcl.SAC_RANSAC)

# Máxima distancia
max_distance = 2
seg.set_distance_threshold(max_distance)
# Función de segmentación con RANSAC para obtener los índices de los inliers
inliers, coefficients = seg.segment()

# Extracción de  inliers
cloud_inliers = cloud.extract(inliers, negative=False)

# Grabar el resultado en disco

filename = os.path.join(os.path.dirname(__file__), '../data/mesa_segmented.pcd')
pcl.save(cloud_inliers, filename)