import numpy as np
from termcolor import cprint


def main():
    # The b vector represents measured sensor values (in the form of projected x
    # and y coordinates onto a plane unit distance away from camera).
    b = np.array(
        [
            -0.2926,
            -0.0822,
            0.3296,
            0.9955,
            0.6459,
            0.4924,
            0.1919,
            -0.2940,
            0.5138,
            0.0796,
            0.0948,
            0.4814,
            0.3403,
            0.1154,
            -0.0404,
            0.4376,
        ]
    )
    # The rows of the p matrix represent the coordinates of reference points in
    # in the local coordinates of the object being tracked.
    p = [
        [-42, 25, 0],
        [42, 25, 0],
        [42, -25, 0],
        [-42, -25, 0],
        [0, -25, 10],
        [0, 25, 10],
        [0, -25, -10],
        [0, 25, -10],
    ]
    # The A matrix relates the measured sensor values (i.e., the b vector) and
    # the parameters we're trying to estimate (i.e., the h vector).
    A = np.empty((16, 11))
    for i in range(8):
        x, y, z = p[i]
        x_n = b[2 * i]
        y_n = b[2 * i + 1]
        A[2 * i] = [x, y, z, 1, 0, 0, 0, 0, -x * x_n, -y * x_n, -z * x_n]
        A[2 * i + 1] = [0, 0, 0, 0, x, y, z, 1, -x * y_n, -y * y_n, -z * y_n]
    # Estimate the h vector.
    h = np.linalg.inv(A.T @ A) @ A.T @ b
    # Reformat the h vector as a 3x4 matrix.
    h_augmented = np.append(h, 1).reshape(3, 4)
    cprint("Unscaled Homography Matrix", attrs=["dark", "underline"])
    print(h_augmented)
    print()
    # Compute the scale factor.
    s = 3 / (
        np.linalg.norm(h_augmented[:, 0])
        + np.linalg.norm(h_augmented[:, 1])
        + np.linalg.norm(h_augmented[:, 2])
    )
    cprint("Scale Factor", attrs=["dark", "underline"])
    print(f"s: {s}")
    print()
    # Compute the translation vector.
    cprint("Translation Vector", attrs=["dark", "underline"])
    print(f"t: {s * h_augmented[:, 3]}")
    print()
    # Compute the rotation matrix.
    Q, _ = np.linalg.qr(h_augmented[:, :3])
    Q = -Q # TODO: why is this necessary?
    Q[2, :] = -Q[2, :] # TODO: why is this necessary?
    cprint("Rotation Matrix", attrs=["dark", "underline"])
    print(Q)
    print()
    # Compute Euler angles.
    cprint("Euler Angles", attrs=["dark", "underline"])
    print(f"Roll:\t{np.rad2deg(np.arctan2(-Q[0][1], Q[1][1]))}")
    print(f"Pitch:\t{np.rad2deg(np.arcsin(Q[2][1]))}")
    print(f"Yaw:\t{np.rad2deg(np.arctan2(-Q[2][0], Q[2][2]))}")


if __name__ == "__main__":
    main()
