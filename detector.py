import numpy as np
try:
    # We will use a simple clustering algorithm
    from sklearn.cluster import DBSCAN
except ImportError:
    print("Warning: scikit-learn not found. Please install with 'conda install scikit-learn'")


class Detector:
    def __init__(self):
        # DBSCAN parameters:
        # eps: The maximum distance between two samples for one to be considered
        #      as in the neighborhood of the other. (e.g., 0.5 meters)
        # min_samples: The number of samples in a neighborhood for a point
        #              to be considered as a core point.
        self.clustering = DBSCAN(eps=0.7, min_samples=10)
        
        # Define the 3 class types for the lab
        self.class_map = {
            'vehicle': 0,
            'pedestrian': 1,
            'cyclist': 2
        }


    def sensors(self):  # pylint: disable=no-self-use
        """
        Define the sensor suite required by the detector.
        We only need LIDAR for this approach, but we'll keep the
        camera for visualization.
        """
        sensors = [
            # 1. Main Front Camera (for visualization)
            {'type': 'sensor.camera.rgb', 
             'x': 2.0, 'y': 0.0, 'z': 1.5,
             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 1280, 'height': 720, 'fov': 100, 
             'id': 'Camera_Front'},

            # 2. Top-Mounted LIDAR (for detection)
            {'type': 'sensor.lidar.ray_cast', 
             'x': 0.0, 'y': 0.0, 'z': 2.0,
             'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'range': 50, 
             'rotation_frequency': 20, 
             'channels': 64,
             'upper_fov': 4, 
             'lower_fov': -20, 
             'points_per_second': 2304000,
             'id': 'LIDAR_Top'},

            # 3. GNSS (GPS) Sensor
            {'type': 'sensor.other.gnss', 
             'x': 0.0, 'y': 0.0, 'z': 0.0, 
             'id': 'GPS'}
        ]

        return sensors

    def detect(self, sensor_data):
        """
        Detects objects by clustering the LIDAR point cloud.
            Input: sensor_data, a dictionary containing all sensor data.
            Output: a dictionary of detected objects in the SENSOR'S LOCAL COORDINATE FRAME
        """
        
        # 1. Check if LIDAR data is present
        if 'LIDAR_Top' not in sensor_data or 'sklearn' not in globals():
            return {}

        # 2. Get LIDAR data
        frame_id, lidar_data = sensor_data['LIDAR_Top']
        # lidar_data is (N, 4) [x, y, z, intensity]
        
        # 3. Pre-process: Filter out the ground plane
        # This is a simple filter: assume any point below a certain z-value
        # (relative to the sensor) is ground.
        # Since the LIDAR is at z=2.0, let's filter points below z=-1.5 (0.5m above ground)
        points_no_ground = lidar_data[lidar_data[:, 2] > -1.5]

        if len(points_no_ground) < self.clustering.min_samples:
            return {}

        # 4. Run Clustering
        # We only need to cluster on x, y, z coordinates
        self.clustering.fit(points_no_ground[:, :3])
        labels = self.clustering.labels_
        
        # -1 label is "noise" (points not in a cluster), we ignore it
        unique_labels = set(labels) - {-1}

        det_boxes = []
        det_class = []
        det_score = []

        # 5. Post-process: Create Bounding Boxes
        for label in unique_labels:
            cluster_points = points_no_ground[labels == label]
            
            if len(cluster_points) < self.clustering.min_samples:
                continue

            # Find the min/max points of the cluster to create a box
            min_point = np.min(cluster_points[:, :3], axis=0)
            max_point = np.max(cluster_points[:, :3], axis=0)

            # Get center and dimensions
            center = (min_point + max_point) / 2
            dims = max_point - min_point
            
            # --- Classify cluster (simple heuristic) ---
            # We assume large clusters are vehicles, small ones are pedestrians
            # A vehicle is typically > 2.5m long/wide and > 1.2m high
            # A pedestrian is typically < 1m long/wide and ~1.8m high
            
            # Check volume and dimensions
            volume = dims[0] * dims[1] * dims[2]
            
            if (dims[0] > 1.5 or dims[1] > 1.5) and volume > 1.0:
                det_class.append(self.class_map['vehicle'])
            else:
                det_class.append(self.class_map['pedestrian'])
            
            # Add a constant score (since this isn't a probablistic model)
            det_score.append(0.9)

            # Create the 8 corners of the 3D box
            dx, dy, dz = dims / 2
            corners = np.array([
                [center[0]+dx, center[1]+dy, center[2]+dz],
                [center[0]+dx, center[1]-dy, center[2]+dz],
                [center[0]-dx, center[1]-dy, center[2]+dz],
                [center[0]-dx, center[1]+dy, center[2]+dz],
                [center[0]+dx, center[1]+dy, center[2]-dz],
                [center[0]+dx, center[1]-dy, center[2]-dz],
                [center[0]-dx, center[1]-dy, center[2]-dz],
                [center[0]-dx, center[1]+dy, center[2]-dz]
            ])
            det_boxes.append(corners)

        if not det_boxes:
            return {}

        return {
            'det_boxes': np.array(det_boxes),
            'det_class': np.array(det_class).reshape(-1, 1),
            'det_score': np.array(det_score).reshape(-1, 1)
        }


