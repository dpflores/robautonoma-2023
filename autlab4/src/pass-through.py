# -*- coding: utf-8 -*-

import pcl
import os

# Cargar la nube de puntosW
cloud = pcl.load_XYZRGB(os.path.join(os.path.dirname(__file__), '../data/mesa_downsampled.pcd'))

# Crear un filtro PassThrough
passthrough = cloud.make_passthrough_filter()

# Assignar el eje y el rango para el filtro.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
min_val = 0.5
max_val = 1
passthrough.set_filter_limits(min_val, max_val)

# Usar el filtro para obtener la nube de puntos resultante
cloud_filtered = passthrough.filter()

# Grabar el resultado en disco

filename = os.path.join(os.path.dirname(__file__), '../data/mesa_pass_through.pcd')
pcl.save(cloud_filtered, filename)