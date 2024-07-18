import numpy as np
import placo

class Trajectory():
    
    def __init__(self, start_frame: np.ndarray, end_frame: np.ndarray, duration: float, smoothing_ratio: float = -1.):
        self.spline = placo.CubicSpline()
        self.start_frame = start_frame
        self.end_frame = end_frame
        self.duration = duration
        self.spline.add_point(0.0, 0.0, 0.0)
        
        if smoothing_ratio > 0.0:
            self.spline.add_point(duration * smoothing_ratio, 0.0, 0.0)
            self.spline.add_point(duration - duration * smoothing_ratio, 1.0, 0.0)
            
        self.spline.add_point(duration, 1.0, 0.0)
    
    def __call__(self, t: float) -> np.ndarray:
        return placo.interpolate_frames(self.start_frame, self.end_frame, self.spline.pos(t))
    
    def clear(self) -> None:
        self.spline.clear()
        self.duration = 0.0
        del self.start_frame
        del self.end_frame