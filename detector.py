import warnings

# Try to import scikit-learn. If it fails, print a warning.
try:
    from sklearn.cluster import DBSCAN
except ImportError:
    warnings.warn(
        "scikit-learn not found. Please install with 'conda install scikit-learn'"
    )

import numpy as np

# CARLA 0.9.16+ compatibility
try:
    import carla
except ImportError:
    pass


class Detector:
    def __init__(self):
        """
        Initializes the DBSCAN clustering model.
        """
        # We will only try to use DBSCAN if the import was successful
        if 'DBSCAN' in locals():
            # eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
            # min_samples: The number of samples in a neighborhood for a point to be considered as a core point.
            self.clustering = DBSCAN(eps=0.7, min_samples=10)
        else:
            self.clustering = None

    def sensors(self):  # pylint: disable=no-self-use
        """
        Defines the sensor suite. We only need a camera for visualization
        and a LIDAR for detection.
        """
        sensors = [
            # Main Front Camera (for visualization)
            {'type': 'sensor.camera.rgb',
             'x': 2.0, 'y': 0.0, 'z': 1.5,
             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 1280, 'height': 720, 'fov': 100,
             'id': 'Camera_Front'},

            # Top-Mounted LIDAR (for detection)
            {'type': 'sensor.lidar.ray_cast',
             'x': 0.0, 'y': 0.0, 'z': 2.0,  # Centered, 2m high
             'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'range': 50,
             'rotation_frequency': 20,
             'channels': 64,
             'upper_fov': 4,
             'lower_fov': -20,
             'points_per_second': 2304000,
             'id': 'LIDAR_Top'},

            # GNSS (GPS) Sensor
            {'type': 'sensor.other.gnss',
             'x': 0.0, 'y': 0.0, 'z': 0.0,
             'id': 'GPS'}
        ]

        return sensors

    def detect(self, sensor_data):
        """
        Detects objects in the LIDAR point cloud using DBSCAN clustering.
        Outputs boxes in the SENSOR'S LOCAL COORDINATE FRAME.
        """
        if self.clustering is None:
            print("DBSCAN not initialized. scikit-learn may not be installed.")
            return {}

        if 'LIDAR_Top' not in sensor_data:
            return {}

        # ---
        # BUG FIX 1: Unpack the (frame_id, data_object) tuple
        # The data_object is a numpy array, not a carla.LidarMeasurement
        # ---
        frame_id, lidar_data = sensor_data['LIDAR_Top']

        # lidar_data is an (N, 4) numpy array [x, y, z, intensity]
        # These are already in the LIDAR's local coordinate frame.
        points_local = lidar_data[:, :3]  # Extract (x, y, z)

        # ---
        # BUG FIX 2: Correct Ground Filtering
        # The sensor is at z=2.0, so the ground is at z=-2.0.
        # This filter keeps all points ~30cm or more above the ground.
        # ---
        ground_filter = points_local[:, 2] > -1.7
        points_filtered = points_local[ground_filter]

        if len(points_filtered) < self.clustering.min_samples:
            return {}  # Not enough points to cluster

        # Perform clustering
        self.clustering.fit(points_filtered)
        labels = self.clustering.labels_

        # -1 label is noise, 0+ are valid clusters
        valid_labels = set(labels) - {-1}
        if not valid_labels:
            return {}

        det_boxes = []
        det_scores = []
        det_classes = []

        for label in valid_labels:
            cluster_points = points_filtered[labels == label]

            if len(cluster_points) < self.clustering.min_samples:
                continue

            # --- Create an axis-aligned bounding box (AABB) ---
            # Find the min/max (x, y, z) for the cluster
            min_coords = np.min(cluster_points, axis=0)
            max_coords = np.max(cluster_points, axis=0)
            
            # Get center and dimensions
            center = (min_coords + max_coords) / 2.0
            dims = max_coords - min_coords

            # --- Convert to 8-corner format for eval.py ---
            # Create the 8 corners in local space
            c = np.array([
                [center[0] - dims[0]/2, center[1] - dims[1]/2, center[2] - dims[2]/2],
                [center[0] + dims[0]/2, center[1] - dims[1]/2, center[2] - dims[2]/2],
                [center[0] + dims[0]/2, center[1] + dims[1]/2, center[2] - dims[2]/2],
                [center[0] - dims[0]/2, center[1] + dims[1]/2, center[2] - dims[2]/2],
                [center[0] - dims[0]/2, center[1] - dims[1]/2, center[2] + dims[2]/2],
                [center[0] + dims[0]/2, center[1] - dims[1]/2, center[2] + dims[2]/2],
                [center[0] + dims[0]/2, center[1] + dims[1]/2, center[2] + dims[2]/2],
                [center[0] - dims[0]/2, center[1] + dims[1]/2, center[2] + dims[2]/2],
            ])

            det_boxes.append(c)
            # Create a dummy score (e.g., number of points in cluster)
            det_scores.append(len(cluster_points))
            # Assume everything is a vehicle (class 0)
            det_classes.append(0)

        if not det_boxes:
            return {}

        # ---
        # BUG FIX 3: Fix the TypeError from eval.py
        # Return det_score as a 1D array, not 2D
        # ---
        return {
            'det_boxes': np.array(det_boxes),
            'det_class': np.array(det_classes).reshape(-1, 1),
            'det_score': np.array(det_scores) # Must be 1D
        }