import numpy as np

P0 = np.array([-42, 25, 0])
P1 = np.array([42, 25, 0])
P2 = np.array([42, -25, 0])
P3 = np.array([-42, -25, 0])

# Euler angles (in degrees)
YAW = 0
PITCH = 45
ROLL = 45

T = np.array([10, 10, -50])

CPU_CLOCK_FREQUENCY = 48e6


def rotate_yaw(yaw):
    """Yaw is rotation about the y axis."""
    sine = np.sin(np.deg2rad(yaw))
    cosine = np.cos(np.deg2rad(yaw))
    return np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]])


def rotate_pitch(pitch):
    """Pitch is the rotation about the x axis."""
    sine = np.sin(np.deg2rad(pitch))
    cosine = np.cos(np.deg2rad(pitch))
    return np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]])


def rotate_roll(roll):
    """Roll is the the rotation about the z axis."""
    sine = np.sin(np.deg2rad(roll))
    cosine = np.cos(np.deg2rad(roll))
    return np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])


def rotate(yaw, pitch, roll):
    """Constructs a rotation matrix (yaw -> pitch -> roll)."""
    return rotate_roll(roll) @ rotate_pitch(pitch) @ rotate_yaw(yaw)


def horizontal_angle(world_coordinates):
    """Returns the horizontal angle (in degrees)."""
    x, _, z = world_coordinates
    return np.rad2deg(np.arctan2(x, -z))


def vertical_angle(world_coordinates):
    """Returns the vertical angle (in degrees)."""
    _, y, z = world_coordinates
    return np.rad2deg(np.arctan2(y, -z))


def horizontal_angle_to_clock_ticks(horizontal_angle):
    return ((90 - horizontal_angle) / (60 * 360)) * CPU_CLOCK_FREQUENCY


def vertical_angle_to_clock_ticks(vertical_angle):
    return ((vertical_angle + 90) / (60 * 360)) * CPU_CLOCK_FREQUENCY


def angle_to_lateral_coordinates(angle):
    return np.tan(np.deg2rad(angle))


def construct_A_matrix(nlc, local_coordinates):
    assert len(nlc) == 4
    assert len(local_coordinates) == 4
    A = np.empty((8, 8))
    for i in range(4):
        x, y, _ = local_coordinates[i]
        x_n, y_n = nlc[i]
        A[2 * i] = [x, y, 1, 0, 0, 0, -x * x_n, -y * x_n]
        A[2 * i + 1] = [0, 0, 0, x, y, 1, -x * y_n, -y * y_n]
    return A


def main():
    R = rotate(YAW, PITCH, ROLL)

    print("Rotation Matrix")
    print(R)
    print()

    P0_world = R @ P0 + T
    P1_world = R @ P1 + T
    P2_world = R @ P2 + T
    P3_world = R @ P3 + T

    print("World Coordinates")
    print(f"P0: {P0_world}")
    print(f"P1: {P1_world}")
    print(f"P2: {P2_world}")
    print(f"P3: {P3_world}")
    print()

    P0_h_angle = horizontal_angle(P0_world)
    P1_h_angle = horizontal_angle(P1_world)
    P2_h_angle = horizontal_angle(P2_world)
    P3_h_angle = horizontal_angle(P3_world)

    print("Horizontal Angles")
    print(f"P0: {P0_h_angle}")
    print(f"P1: {P1_h_angle}")
    print(f"P2: {P2_h_angle}")
    print(f"P3: {P3_h_angle}")
    print()

    print("Horizontal Clock Ticks")
    print(f"P0: {horizontal_angle_to_clock_ticks(P0_h_angle)}")
    print(f"P1: {horizontal_angle_to_clock_ticks(P1_h_angle)}")
    print(f"P2: {horizontal_angle_to_clock_ticks(P2_h_angle)}")
    print(f"P3: {horizontal_angle_to_clock_ticks(P3_h_angle)}")
    print()

    P0_v_angle = vertical_angle(P0_world)
    P1_v_angle = vertical_angle(P1_world)
    P2_v_angle = vertical_angle(P2_world)
    P3_v_angle = vertical_angle(P3_world)

    print("Vertical Angles")
    print(f"P0: {P0_v_angle}")
    print(f"P1: {P1_v_angle}")
    print(f"P2: {P2_v_angle}")
    print(f"P3: {P3_v_angle}")
    print()

    print("Vertical Clock Ticks")
    print(f"P0: {vertical_angle_to_clock_ticks(P0_v_angle)}")
    print(f"P1: {vertical_angle_to_clock_ticks(P1_v_angle)}")
    print(f"P2: {vertical_angle_to_clock_ticks(P2_v_angle)}")
    print(f"P3: {vertical_angle_to_clock_ticks(P3_v_angle)}")
    print()

    P0_nlc = np.array(
        [
            angle_to_lateral_coordinates(P0_h_angle),
            angle_to_lateral_coordinates(P0_v_angle),
        ]
    )
    P1_nlc = np.array(
        [
            angle_to_lateral_coordinates(P1_h_angle),
            angle_to_lateral_coordinates(P1_v_angle),
        ]
    )
    P2_nlc = np.array(
        [
            angle_to_lateral_coordinates(P2_h_angle),
            angle_to_lateral_coordinates(P2_v_angle),
        ]
    )
    P3_nlc = np.array(
        [
            angle_to_lateral_coordinates(P3_h_angle),
            angle_to_lateral_coordinates(P3_v_angle),
        ]
    )

    print("Normalized Lateral Coordinates")
    print(f"P0: {P0_nlc}")
    print(f"P1: {P1_nlc}")
    print(f"P2: {P2_nlc}")
    print(f"P3: {P3_nlc}")
    print()

    A = construct_A_matrix((P0_nlc, P1_nlc, P2_nlc, P3_nlc), (P0, P1, P2, P3))

    print("The A Matrix")
    print(A)
    print()

    print("Singular Values")
    _, s, _ = np.linalg.svd(A)
    print(s)
    print()

    print("Condition Number")
    print(f"K(A): {max(s) / min(s)}")
    print()


if __name__ == "__main__":
    main()
