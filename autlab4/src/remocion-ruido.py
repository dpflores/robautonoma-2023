# Import PCL module
# -*- coding: utf-8 -*-
import os
import pcl

# Load Point Cloud file
cloud = pcl.load(os.path.join(os.path.dirname(__file__), '../data/escena2.pcd'))

# ===================================================
# Filtro de remoción de outliers
# ===================================================

# Objeto de filtro
outlier_filter = cloud.make_statistical_outlier_filter()

# Número de puntos del vecindario para analizar 
outlier_filter.set_mean_k(50)

# Thresold (factor de escala)
x = 1

# Cualquier punto con distancia media mayor que la global (distancia
# media+x*std_dev) será considerado un outlier
outlier_filter.set_std_dev_mul_thresh(x)

# Aplicar el filtro
cloud_filtered = outlier_filter.filter()

filename = os.path.join(os.path.dirname(__file__), '../data/escena2_filtered_noise.pcd')
pcl.save(cloud_filtered, filename)