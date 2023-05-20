#!/usr/bin/env python3
import pcl
import os

class Cloud():
    def __init__(self, path):
        self.cloud = pcl.load(path)

    def downsample(self, voxel_size):
        # Crear un filtro VoxelGrid para la nube de puntos
        fvox = self.cloud.make_voxel_grid_filter()
        # Tamaño de voxel
        VOXEL_SIZE = voxel_size
        # Asignar el tamaño del voxel al filtro
        fvox.set_leaf_size(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE)

        # Ejecutar el filtro
        cloud_filtered = fvox.filter()

        self.cloud = cloud_filtered

    def pass_through(self, filter_axis, min_val, max_val ):
        # Crear un filtro PassThrough
        passthrough = self.cloud.make_passthrough_filter()

        # Assignar el eje y el rango para el filtro.
        
        passthrough.set_filter_field_name(filter_axis)

        passthrough.set_filter_limits(min_val, max_val)

        # Usar el filtro para obtener la nube de puntos resultante
        cloud_filtered = passthrough.filter()

        self.cloud = cloud_filtered

    def ransac(self, max_distance, mode=False):
        # Crear el objeto para la segmentación
        seg = self.cloud.make_segmenter()

        # Asignar el modelo que se desea ajustar
        seg.set_model_type(pcl.SACMODEL_PLANE)
        # Uso de RANSAC
        seg.set_method_type(pcl.SAC_RANSAC)

        # Máxima distancia
        
        seg.set_distance_threshold(max_distance)
        # Función de segmentación con RANSAC para obtener los índices de los inliers
        inliers, coefficients = seg.segment()

        # Extracción de  inliers
        # mode False for inliers and true for outliers
        cloud_inliers = self.cloud.extract(inliers, negative=mode)

        self.cloud = cloud_inliers

    def noise_reduction(self, x, k):
    # Objeto de filtro
        outlier_filter = self.cloud.make_statistical_outlier_filter()
        # Número de puntos del vecindario para analizar 
        outlier_filter.set_mean_k(k)

        # Thresold (factor de escala)
        

        # Cualquier punto con distancia media mayor que la global (distancia
        # media+x*std_dev) será considerado un outlier
        outlier_filter.set_std_dev_mul_thresh(x)

        # Aplicar el filtro
        cloud_filtered = outlier_filter.filter()
        self.cloud = cloud_filtered

# Cargar la nube de puntos

cloud = Cloud(os.path.join(os.path.dirname(__file__), '../data/escena2_filtered_noise.pcd'))

# Downsample
cloud.downsample(0.008)

# Pass through

cloud.pass_through( "y", 0.0, 0.9)

# Ransac
cloud.ransac(0.01, False)

# Grabar el resultado en disco
filename = os.path.join(os.path.dirname(__file__), '../data/mesa_only_escena2.pcd')
pcl.save(cloud.cloud, filename)


