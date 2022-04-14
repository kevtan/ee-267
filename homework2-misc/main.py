import numpy as np
from termcolor import cprint
from typing import Tuple

WIDTH = 6
HEIGHT = 6
T_1 = -6 / 5
T_2 = -22 / 5
E_1 = -1
PROJECTION_MATRIX = np.array(
    [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, T_1, T_2], [0, 0, E_1, 0]]
)


def window_to_ndc(
    window: Tuple[float, float, float], w: int, h: int
) -> Tuple[float, float, float]:
    x_window, y_window, z_window = window
    x_ndc = (2 * x_window) / w - 1
    y_ndc = (2 * y_window) / h - 1
    z_ndc = 2 * z_window - 1
    return x_ndc, y_ndc, z_ndc


def ndc_to_clip(
    ndc: Tuple[float, float, float], T1: float, T2: float, E1: float
) -> Tuple[float, float, float, float]:
    x_ndc, y_ndc, z_ndc = ndc
    w_clip = T2 / (z_ndc - (T1 / E1))
    x_clip = x_ndc * w_clip
    y_clip = y_ndc * w_clip
    z_clip = z_ndc * w_clip
    return x_clip, y_clip, z_clip, w_clip


def clip_to_view(
    clip: Tuple[float, float, float, float], projection_matrix: np.ndarray
):
    inverse_projection_matrix = np.linalg.inv(projection_matrix)
    return inverse_projection_matrix @ clip


def main():
    # Vertices in window space
    v1_window = (1.5, 4, 0.7)
    v2_window = (0.5, 1, 0.5)
    v3_window = (4.5, 0.5, 0.3)
    # Vertices in NDC
    cprint("Normalized Device Coordinates (NDC)", attrs=["dark", "underline"])
    v1_ndc = window_to_ndc(v1_window, WIDTH, HEIGHT)
    v2_ndc = window_to_ndc(v2_window, WIDTH, HEIGHT)
    v3_ndc = window_to_ndc(v3_window, WIDTH, HEIGHT)
    print(f"v1_ndc: {v1_ndc}")
    print(f"v2_ndc: {v2_ndc}")
    print(f"v3_ndc: {v3_ndc}")
    print()
    # Vertices in clip space
    cprint("Clip Space", attrs=["dark", "underline"])
    v1_clip = ndc_to_clip(v1_ndc, T_1, T_2, E_1)
    v2_clip = ndc_to_clip(v2_ndc, T_1, T_2, E_1)
    v3_clip = ndc_to_clip(v3_ndc, T_1, T_2, E_1)
    print(f"v1_clip: {v1_clip}")
    print(f"v2_clip: {v2_clip}")
    print(f"v3_clip: {v3_clip}")
    print()
    # Vertices in view space
    cprint("View Space", attrs=["dark", "underline"])
    v1_view = clip_to_view(v1_clip, PROJECTION_MATRIX)
    v2_view = clip_to_view(v2_clip, PROJECTION_MATRIX)
    v3_view = clip_to_view(v3_clip, PROJECTION_MATRIX)
    print(f"v1_view: {v1_view}")
    print(f"v2_view: {v2_view}")
    print(f"v3_view: {v3_view}")


if __name__ == "__main__":
    main()
