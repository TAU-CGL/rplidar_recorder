import reccalib
import numpy as np
import matplotlib.pyplot as plt



def test_reccalib_fit_circle_least_squares():
    success = 0
    num_experiences = 10000
    for _ in range(num_experiences):
        # Test the circle fitting function with a few example points
        #sample N = 100 points on a circle
        theta1 = np.random.uniform(0, 2 * np.pi)
        theta2 = np.random.uniform(theta1 + 30 / 180 * np.pi, 2 * np.pi) # At least 10 degrees apart
        Cx, Cy, Cr = np.random.uniform(-0.5, 0.5, 3)
        Cr = 0.5 * (Cr + 1)
        thetas = np.linspace(theta1, theta2, 100)
        points = np.zeros((len(thetas), 2))
        points[:, 0] = Cr * np.cos(thetas) + Cx
        points[:, 1] = Cr * np.sin(thetas) + Cy
        points += np.random.normal(0, 0.0005, points.shape)

        # Fit the circle using the least squares method
        ls = reccalib.LidarSnapshot(points=points, device_id="test_device", timestamp=0)
        circle = reccalib.fit_circle_least_squares(ls)

        Corg = np.array([Cx, Cy, Cr])
        Cfit = np.array([circle.center[0], circle.center[1], circle.radius])
        err = np.linalg.norm(Corg - Cfit)
        success += 1 if err < 0.01 else 0

        # plt.scatter(points[:,0], points[:,1], c="k")
        # circle = plt.Circle((Cx, Cy), Cr, color='r', fill=False)
        # plt.gca().add_artist(circle)
        # fitted_circle = plt.Circle(center, radius, color='b', fill=False)
        # plt.gca().add_artist(fitted_circle)
        # plt.xlim(-2, 2)
        # plt.ylim(-2, 2)
        # plt.show()
    assert success > 0.95 * num_experiences, f"Failed to fit circle in {success} out of {num_experiences} attempts."



if __name__ == "__main__":
    # while True:
    #     test_reccalib_fit_circle_least_squares()
    pass