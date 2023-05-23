import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d.art3d import Line3D
from mpl_toolkits.mplot3d import proj3d
import numpy as np
import matplotlib.transforms as transforms
from matplotlib.widgets import Slider

from stabalization_platform import StabalizationPlatform

class Simulation:

    def __init__(self) -> None:
        """Constructor
        """
        # Platform specific initialization
        self.rps_platform_3dof = StabalizationPlatform(5.0, 4.0, 3.0, 5.0)
        self.rps_platform_3dof.update_solve_pose(4.0, 4.0, 4.0)

        # Visualization
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.line_objects = []
        viz_limits = 1.5 * max(self.rps_platform_3dof.baseplate_radius,
                         self.rps_platform_3dof.topplate_radius)
        self.ax.set_xlim([-viz_limits, viz_limits])
        self.ax.set_ylim([-viz_limits, viz_limits])
        self.ax.set_zlim([0, viz_limits])

        # Slider parameters
        l1, l2, l3 = self.rps_platform_3dof.actuator_lengths
        slider_min = self.rps_platform_3dof.actuator_min
        slider_max = self.rps_platform_3dof.actuator_max
        self.ax_L1 = plt.axes([0.25, 0.15, 0.65, 0.03])
        self.ax_L2 = plt.axes([0.25, 0.10, 0.65, 0.03])
        self.ax_L3 = plt.axes([0.25, 0.05, 0.65, 0.03])
        self.slider_L1 = Slider(self.ax_L1, 'L1', slider_min, slider_max, valinit=l1)
        self.slider_L2 = Slider(self.ax_L2, 'L2', slider_min, slider_max, valinit=l2)
        self.slider_L3 = Slider(self.ax_L3, 'L3', slider_min, slider_max, valinit=l3)

        # Connect the update function to each slider
        self.slider_L1.on_changed(self.update_slider)
        self.slider_L2.on_changed(self.update_slider)
        self.slider_L3.on_changed(self.update_slider)

        # set lines
        self.set_lines()


    def set_lines(self):
        # Base of platform
        base_position_vectors = self.rps_platform_3dof.actuator_base_position_vectors
        base_line_vectors = [   base_position_vectors[1]-base_position_vectors[0], \
                                base_position_vectors[2]-base_position_vectors[1], \
                                base_position_vectors[0]-base_position_vectors[2]
                            ]
        for vector, position in zip(base_line_vectors, base_position_vectors):
            self.plot_vector(vector, position, color='k')
        
        # Top plate of platform
        top_position_vectors = self.rps_platform_3dof.actuator_top_position_vectors
        top_line_vectors =  [    top_position_vectors[1]-top_position_vectors[0], \
                                top_position_vectors[2]-top_position_vectors[1], \
                                top_position_vectors[0]-top_position_vectors[2]
                            ]
        for vector, position in zip(top_line_vectors, top_position_vectors):
            self.plot_vector(vector, position, color='k')
    
        # Actuators
        for vector, position in zip(self.rps_platform_3dof.actuator_vectors, self.rps_platform_3dof.actuator_base_position_vectors):
            self.plot_vector(vector, position, color='r')

        self.fig.canvas.draw_idle()


    def plot_vector(self, vector, position=[0.0,0.0,0.0], color='b'):
        x, y, z = position
        u, v, w = vector
        line = Line3D([x, x + u], [y, y + v], [z, z + w], color=color)
        self.ax.add_line(line)
        self.line_objects.append(line)


    def update_slider(self, val):
        l1 = self.slider_L1.val
        l2 = self.slider_L2.val
        l3 = self.slider_L3.val        

        self.rps_platform_3dof.update_solve_pose_on_change(l1, l2, l3)

        # set updated values to slider
        l1, l2, l3 = self.rps_platform_3dof.actuator_lengths

        # Remove old lines
        while self.line_objects:
            line = self.line_objects.pop()
            line.remove()
        
        # Redraw lines
        self.set_lines()

        self.fig.canvas.draw_idle()


    def show(self):
        plt.grid(True)
        plt.show()