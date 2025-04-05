import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import random

class BrownianRobot:
    def __init__(self, arena_size=10, speed=0.01):  
        """
        Initialize the Brownian motion robot simulation
        
        Parameters:
        - arena_size: size of the square arena (default 10)
        - speed: movement speed of the robot (default 0.01)
        """
        self.arena_size = arena_size
        self.speed = speed
        
        self.x = arena_size / 2
        self.y = arena_size / 2
        
        self.direction = random.uniform(0, 2*np.pi)
        
        self.target_x = None
        self.target_y = None
        self.set_new_target()
        
        self.trajectory = [(self.x, self.y)]
        self.collision_points = []
    
    def set_new_target(self):
        """calclate the next collision point and set it as target"""
        
        t_values = []
        
        if np.cos(self.direction) > 0:
            t_x = (self.arena_size - self.x) / (np.cos(self.direction) + 1e-10)
        elif np.cos(self.direction) < 0:
            t_x = (0 - self.x) / (np.cos(self.direction) + 1e-10)
        else:
            t_x = float('inf')
        
        if np.sin(self.direction) > 0:
            t_y = (self.arena_size - self.y) / (np.sin(self.direction) + 1e-10)
        elif np.sin(self.direction) < 0:
            t_y = (0 - self.y) / (np.sin(self.direction) + 1e-10)
        else:
            t_y = float('inf')
        
        t = min(t_x, t_y)
        
        self.target_x = self.x + np.cos(self.direction) * t
        self.target_y = self.y + np.sin(self.direction) * t
    
    def update(self, dt):
        """
        update the robot s position based on its current state
        dt: time since last update
        """
        distance = self.speed * dt
        dx = np.cos(self.direction) * distance
        dy = np.sin(self.direction) * distance
        
        remaining_x = self.target_x - self.x
        remaining_y = self.target_y - self.y
        
        
        if (dx**2 + dy**2) >= (remaining_x**2 + remaining_y**2):
            
            self.x = self.target_x
            self.y = self.target_y
            self.trajectory.append((self.x, self.y))
            self.collision_points.append((self.x, self.y))
            
            if self.x <= 0 or self.x >= self.arena_size:
                self.direction = np.pi - self.direction  
            if self.y <= 0 or self.y >= self.arena_size:
                self.direction = -self.direction  
            
            self.direction += random.uniform(-np.pi/4, np.pi/4)
            
            self.direction = self.direction % (2*np.pi)
            
            self.set_new_target()
        else:
            
            self.x += dx
            self.y += dy
            self.trajectory.append((self.x, self.y))
    
    def get_position(self):
        """return the current (x, y) position of the robot"""
        return (self.x, self.y)
    
    def get_direction(self):
        """return the current direction angle in radians
        """
        return self.direction
    
    def get_trajectory(self):
        """return the complete trajectory history as a list of (x, y) tuples"""
        return self.trajectory


def run_simulation(duration=30, fps=30, arena_size=10, speed=0.01):
    """
    Run and visualize the Brownian motion robot simulation.
    
    Parameters:
    - duration: simulation time in seconds (default 30)
    - fps: frames per second for visualization (default 30)
    - arena_size: size of the square arena (default 10)
    - speed: movement speed of the robot (default 0.01)
    """
    
    robot = BrownianRobot(arena_size=arena_size, speed=speed)
    fig, ax = plt.subplots(figsize=(8, 8))
    
    ax.set_xlim(0, arena_size)
    ax.set_ylim(0, arena_size)
    ax.set_aspect('equal')
    ax.set_title(f'Brownian Motion Robot Simulation (Speed: {speed})')
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    
    robot_circle = Circle((robot.x, robot.y), 0.1, color='red')
    ax.add_patch(robot_circle)
    
    direction_line, = ax.plot([], [], 'b-', lw=2)
    
    trajectory_line, = ax.plot([], [], 'g-', alpha=0.5)
    
    collision_scatter = ax.scatter([], [], c='r', marker='x', s=100)
    
    
    dt = 1 / fps
    
    def init():
        """Initialize the animation"""
        robot_circle.center = (robot.x, robot.y)
        direction_line.set_data([robot.x, robot.x + np.cos(robot.direction)], 
                               [robot.y, robot.y + np.sin(robot.direction)])
        trajectory_line.set_data([], [])
        collision_scatter.set_offsets(np.empty((0, 2)))
        return robot_circle, direction_line, trajectory_line, collision_scatter
    
    def animate(i):
        """Updat the animation frame"""
        
        robot.update(dt)
        
        robot_circle.center = (robot.x, robot.y)
        direction_line.set_data([robot.x, robot.x + 0.5 * np.cos(robot.direction)], 
                               [robot.y, robot.y + 0.5 * np.sin(robot.direction)])
        show_points = min(1000, len(robot.trajectory))
        x_vals, y_vals = zip(*robot.trajectory[-show_points:])
        trajectory_line.set_data(x_vals, y_vals)
        
        
        if len(robot.collision_points) > 0:
            collision_scatter.set_offsets(robot.collision_points)
        
        return robot_circle, direction_line, trajectory_line, collision_scatter
    
    
    frames = int(duration * fps)
    anim = animation.FuncAnimation(
        fig, animate, frames=frames, init_func=init,
        blit=True, interval=1000/fps, repeat=False
    )
    
    plt.grid()
    plt.show()
    
    return anim


if __name__ == "__main__":
    run_simulation(duration=30, fps=30, speed=3)  