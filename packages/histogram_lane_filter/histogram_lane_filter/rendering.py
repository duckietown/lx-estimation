"""Rendering of the belief histogram as an image.

This is a port of ``dt_state_estimation.lane_filter.rendering.plot_belief`` (lib-dt-state-estimation
0.0.9) without the ``reprep`` dependency: ``reprep`` pulls in ``PyContracts``, which does not import on
Python 3.12 (``inspect.ArgSpec`` was removed). The drawing is the same.
"""
import io
from typing import Optional, Sequence, Tuple

import cv2
import matplotlib
import numpy as np
import numpy.ma as ma

matplotlib.use("Agg")  # headless rendering inside the node
import matplotlib.pyplot as plt  # noqa: E402

BGRImage = np.ndarray


def _scale(value: np.ndarray, min_value: float = 0.0) -> np.ndarray:
    """``reprep.graphics.filter_scale.scale`` equivalent: map ``[min_value, max]`` to a gray level
    (255 = white at the minimum, 0 = black at the maximum). NaNs stay NaN."""
    max_value = np.nanmax(value) if np.isfinite(value).any() else min_value
    if max_value <= min_value:
        return np.full(value.shape, 255.0)
    normalized = (value - min_value) / (max_value - min_value)
    return 255.0 * (1.0 - np.clip(normalized, 0.0, 1.0))


def plot_belief(
    filter,
    bgcolor: Tuple[int, int, int] = (0, 204, 255),
    dpi: int = 150,
    other_phi: Optional[Sequence[float]] = None,
    other_d: Optional[Sequence[float]] = None,
) -> BGRImage:
    """Returns a BGR image of the belief histogram of ``filter`` (an ``ILaneFilter``)."""
    # get estimate
    d, phi = filter.get_estimate()

    bgcolor = tuple(x / 255.0 for x in bgcolor)
    figure = plt.figure(facecolor=bgcolor)

    f_d = lambda x: 100 * x  # noqa: E731
    f_phi = np.rad2deg
    # Where are the lanes?
    lane_width = filter.lanewidth
    d_max = filter.d_max
    d_min = filter.d_min
    phi_max = filter.phi_max
    phi_min = filter.phi_min
    delta_d = filter.delta_d
    delta_phi = filter.delta_phi

    belief = np.array(filter.belief, dtype=float, copy=True)
    zeros = belief == 0
    belief[zeros] = np.nan

    z = ma.masked_array(_scale(belief, min_value=0), zeros)

    x = f_d(filter.d_pcolor)
    y = f_phi(filter.phi_pcolor)

    plt.pcolor(x, y, np.ones(z.shape), cmap="Pastel1")
    plt.pcolor(x, y, z, cmap="gray")

    if other_phi is not None:
        for _phi, _d in zip(other_phi, other_d):
            plt.plot(f_d(_d), f_phi(_phi), "go", markersize=15, markeredgecolor="none", markeredgewidth=3,
                     markerfacecolor="blue")

    plt.plot(f_d(d), f_phi(phi), "go", markersize=20, markeredgecolor="magenta", markeredgewidth=3,
             markerfacecolor="none")
    plt.plot(f_d(d), f_phi(phi), "o", markersize=2, markeredgecolor="none", markeredgewidth=0,
             markerfacecolor="magenta")

    W = f_d(lane_width / 2)
    width_white = f_d(filter.linewidth_white)
    width_yellow = f_d(filter.linewidth_yellow)

    plt.plot([-W, -W], [f_phi(phi_min), f_phi(phi_max)], "w-")
    plt.plot([-W - width_white, -W - width_white], [f_phi(phi_min), f_phi(phi_max)], "k-")
    plt.plot([0, 0], [f_phi(phi_min), f_phi(phi_max)], "k-")
    plt.plot([+W, +W], [f_phi(phi_min), f_phi(phi_max)], "-", color="yellow")
    plt.plot([+W + width_yellow, +W + width_yellow], [f_phi(phi_min), f_phi(phi_max)], "-", color="yellow")

    s = f"status = {filter.status.value}"
    s += f"\nphi = {f_phi(phi):.1f} deg"
    s += f"\nd = {f_d(d):.1f} cm"
    s += f"\nentropy = {filter.get_entropy():.4f}"
    s += f"\nmax = {np.nanmax(belief):.4f}"
    s += f"\nmin = {np.nanmin(belief):.4f}"
    if other_phi is not None:
        s += "\n Other answers:"
        for _phi, _d in zip(other_phi, other_d):
            s += f"\nphi = {f_phi(_phi):.1f} deg"
            s += f"\nd = {f_d(_d):.1f} cm"

    y_text = f_phi(phi_max) - 10
    args = dict(rotation=-90, color="white")
    plt.annotate(s, xy=(0.05, 0.99), xycoords="figure fraction")
    plt.annotate("in middle of right lane", xy=(0, y_text), **args)
    plt.annotate("on right white tape", xy=(-W, y_text), **args)
    plt.annotate("on left yellow tape", xy=(+W, y_text), **args)
    plt.annotate("in other lane", xy=(+W * 1.3, y_text), **args)

    plt.axis([f_d(d_min), f_d(d_max), f_phi(phi_min), f_phi(phi_max)])
    plt.ylabel(f"phi: orientation (deg); cell = {f_phi(delta_phi):.1f} deg")
    plt.xlabel(f"d: distance from center line (cm); cell = {f_d(delta_d):.1f} cm")
    plt.gca().invert_xaxis()

    bgr = _plt_to_bgr(figure, dpi)
    plt.close(figure)
    return bgr


def _plt_to_bgr(figure, dpi) -> BGRImage:
    """Convert a Matplotlib figure to a BGR OpenCV image."""
    buf = io.BytesIO()
    figure.savefig(buf, format="png", dpi=dpi, bbox_inches="tight", pad_inches=0.15, transparent=True,
                   facecolor=figure.get_facecolor())
    buf.seek(0)
    png = np.asarray(bytearray(buf.read()), dtype=np.uint8)
    return cv2.imdecode(png, cv2.IMREAD_COLOR)


__all__ = ["plot_belief"]
