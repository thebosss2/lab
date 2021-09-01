class Frame:
    def __init__(self, x, y, z, qx, qy, qz, qw, frame_id, frame_center_x=0, frame_center_y=0, frame_center_z=0):
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        self.frame_id = frame_id
        self.points = []
        self.frame_center_x = frame_center_x
        self.frame_center_y = frame_center_y
        self.frame_center_z = frame_center_z
