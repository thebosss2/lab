class Room:
    def __init__(self, points=None, rectangle=None, exit_points=None):
        self.points = points if points is not None else []
        self.rectangle = rectangle
        self.exit_points = exit_points if exit_points is not None else []
        self.frames = {}
