from types import SimpleNamespace
import numpy as np
import pytest
from PyQt5.QtGui import QImage
from amr_qt_panel.model.image_convert import image_to_qimage


def _msg(arr, encoding):
    h, w = arr.shape[0], arr.shape[1]
    ch = 1 if arr.ndim == 2 else arr.shape[2]
    return SimpleNamespace(height=h, width=w, encoding=encoding,
                           step=w * ch, data=arr.tobytes())


def test_rgb8():
    arr = np.zeros((4, 5, 3), np.uint8)
    arr[..., 0] = 255
    img = image_to_qimage(_msg(arr, 'rgb8'))
    assert img.width() == 5 and img.height() == 4
    assert img.format() == QImage.Format_RGB888
    assert img.pixelColor(0, 0).red() == 255


def test_mono8():
    arr = np.full((3, 3), 7, np.uint8)
    img = image_to_qimage(_msg(arr, 'mono8'))
    assert img.width() == 3 and img.height() == 3


def test_unsupported():
    arr = np.zeros((2, 2, 4), np.uint8)
    with pytest.raises(ValueError):
        image_to_qimage(_msg(arr, 'rgba8'))
