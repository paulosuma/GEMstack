import matplotlib.pyplot as plt
# import numpy as np

class Evaluator:
    """A performance evaluator for evaluating the controllers."""
    def __init__(self, ref_path, actual_path):
        # Check if both paths have the same length
        if len(ref_path) != len(actual_path):
            raise ValueError("Reference path and actual path must be of the same length.")
        self.ref_path = ref_path
        self.actual_path = actual_path

    def plot(self):
        # Unzip the paths into separate lists for plotting
        ref_x, ref_y, _ = zip(*self.ref_path)
        actual_x, actual_y, _ = zip(*self.actual_path)
        
        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.plot(ref_x, ref_y, label='Reference Path', marker='o')
        plt.plot(actual_x, actual_y, label='Actual Path', marker='x')
        plt.title('Reference vs Actual Path')
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.show()

    def calculate_position_error(self):
        # Calculate the MSE of the position error
        position_errors = [(ref[0] - act[0])**2 + (ref[1] - act[1])**2 
                           for ref, act in zip(self.ref_path, self.actual_path)]
        mse_position = sum(position_errors) / len(self.ref_path)
        return mse_position

    def calculate_orientation_error(self):
        # Calculate the MSE of the orientation error
        orientation_errors = [(ref[2] - act[2])**2 
                              for ref, act in zip(self.ref_path, self.actual_path)]
        mse_orientation = sum(orientation_errors) / len(self.ref_path)
        return mse_orientation

# if __name__ == "__main__":

    



