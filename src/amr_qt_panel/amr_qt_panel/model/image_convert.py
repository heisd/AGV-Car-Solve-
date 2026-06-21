"""sensor_msgs/Image -> QImage，纯 numpy 实现（不依赖 cv2 / cv_bridge）。"""
from PyQt5.QtGui import QImage


def image_to_qimage(msg) -> QImage:
    """Convert sensor_msgs/Image to QImage.

    msg must have height, width, encoding, step, data. Supports rgb8/bgr8/mono8;
    other encodings raise ValueError. Returns a copied QImage detached from the
    input buffer.
    """
    enc = msg.encoding
    w, h, step = msg.width, msg.height, msg.step
    buf = bytes(msg.data)

    if enc == 'rgb8':
        channels = 3
        expected = w * channels
        if step < expected:
            raise ValueError(f"step {step} too small for {w}x{channels}ch")
        img = QImage(buf, w, h, step, QImage.Format_RGB888)
    elif enc == 'bgr8':
        channels = 3
        expected = w * channels
        if step < expected:
            raise ValueError(f"step {step} too small for {w}x{channels}ch")
        fmt = getattr(QImage, 'Format_BGR888', None)
        if fmt is not None:
            img = QImage(buf, w, h, step, fmt)
        else:
            img = QImage(buf, w, h, step, QImage.Format_RGB888).rgbSwapped()
    elif enc == 'mono8':
        channels = 1
        expected = w * channels
        if step < expected:
            raise ValueError(f"step {step} too small for {w}x{channels}ch")
        img = QImage(buf, w, h, step, QImage.Format_Grayscale8)
    else:
        raise ValueError(f"unsupported image encoding: {enc}")

    return img.copy()  # 脱离 buf，避免悬垂引用
