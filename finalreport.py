import matplotlib.pyplot as plt
import matplotlib as mt

self.bdy_center = [40.2672305, -111.635524]
self.bdy_R = [98.353826748828595, 127.49653449395105]
self.bdy_theta = -15*pi/180.0


def ellipse(self, pt):
    """check if point is outside of boundary ellipse (ell > 1)"""

    center = self.bdy_center
    R = self.bdy_R
    theta = self.bdy_theta

    dx, dy, _ = self.distance(center, pt)
    ct = np.cos(theta)
    st = np.sin(theta)
    ell = ((dx*ct + dy*st)/R[0])**2 + ((dx*st - dy*ct)/R[1])**2

    return ell
