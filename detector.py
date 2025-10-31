class Detector:
    def __init__(self):
        # Add your initialization logic here
        pass

    def sensors(self):  # pylint: disable=no-self-use
        """
        Define the sensor suite required by the detector. The location is defined with respect to the actor center
        -- x axis is longitudinal (forward-backward)
        -- y axis is lateral (left and right)
        -- z axis is vertical
        Unit is in meters

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}
        ]

        """
        sensors = [
            # 1. Main Front Camera (for visualization)
            # This is needed to see the world and draw the 3D boxes on.
            {'type': 'sensor.camera.rgb', 
             'x': 2.0, 'y': 0.0, 'z': 1.5,  # 2m forward, 1.5m high
             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 1280, 'height': 720, 'fov': 100, 
             'id': 'Camera_Front'},

            # 2. Top-Mounted LIDAR (as requested)
            {'type': 'sensor.lidar.ray_cast', 
             'x': 0.0, 'y': 0.0, 'z': 2.0,  # Centered, 2m high on the roof
             'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'range': 50, 
             'rotation_frequency': 20, 
             'channels': 64,
             'upper_fov': 4, 
             'lower_fov': -20, 
             'points_per_second': 2304000,
             'id': 'LIDAR_Top'},

            # 3. Front RADAR (as requested)
            {'type': 'sensor.other.radar', 
             'x': 2.0, 'y': 0.0, 'z': 1.0,  # Front bumper
             'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, # Pointing forward
             'horizontal_fov': 30,  # 30-degree cone
             'vertical_fov': 30,
             'range': 100,  # 100m range
             'points_per_second': 1500,
             'id': 'RADAR_Front'},

            # 4. Back RADAR (as requested)
            {'type': 'sensor.other.radar', 
             'x': -2.0, 'y': 0.0, 'z': 1.0, # Rear bumper
             'yaw': 180.0, 'pitch': 0.0, 'roll': 0.0, # Pointing backward
             'horizontal_fov': 30,
             'vertical_fov': 30,
             'range': 100,
             'points_per_second': 1500,
             'id': 'RADAR_Back'},

            # 5. Left RADAR (as requested)
            {'type': 'sensor.other.radar', 
             'x': 0.0, 'y': -0.5, 'z': 1.0, # Left side
             'yaw': -90.0, 'pitch': 0.0, 'roll': 0.0, # Pointing left
             'horizontal_fov': 30,
             'vertical_fov': 30,
             'range': 100,
             'points_per_second': 1500,
             'id': 'RADAR_Left'},

            # 6. Right RADAR (as requested)
            {'type': 'sensor.other.radar', 
             'x': 0.0, 'y': 0.5, 'z': 1.0,  # Right side
             'yaw': 90.0, 'pitch': 0.0, 'roll': 0.0, # Pointing right
             'horizontal_fov': 30,
             'vertical_fov': 30,
             'range': 100,
             'points_per_second': 1500,
             'id': 'RADAR_Right'},
            
            # 7. GNSS (GPS) Sensor
            # Generally useful for global positioning.
            {'type': 'sensor.other.gnss', 
             'x': 0.0, 'y': 0.0, 'z': 0.0, 
             'id': 'GPS'}
        ]

        return sensors

    def detect(self, sensor_data):
        """
        Add your detection logic here
            Input: sensor_data, a dictionary containing all sensor data. Key: sensor id. Value: tuple of frame id and data. For example
                'Right' : (frame_id, numpy.ndarray)
                    The RGBA image, shape (H, W, 4)
                'Left' : (frame_id, numpy.ndarray)
                    The RGBA image, shape (H, W, 4)
                'LIDAR' : (frame_id, numpy.ndarray)
                    The lidar data, shape (N, 4)
            Output: a dictionary of detected objects in global coordinates
                det_boxes : numpy.ndarray
                    The detection bounding box, shape (N, 8, 3) or (N, 4, 2).
                det_class : numpy.ndarray
                    The object class for each predicted bounding box, shape (N, 1) corresponding to the above bounding box. 
                    0 for vehicle, 1 for pedestrian, 2 for cyclist.
                det_score : numpy.ndarray
                    The confidence score for each predicted bounding box, shape (N, 1) corresponding to the above bounding box.
        """
        return {}

    