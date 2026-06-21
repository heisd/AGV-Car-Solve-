"""sensor_msgs/Image -> QImage，纯 numpy 实现（不依赖 cv2 / cv_bridge）。"""
from PyQt5.QtGui import QImage


def image_to_qimage(msg) -> QImage:
    """支持 rgb8 / bgr8 / mono8；其它编码抛 ValueError。返回值已 copy()。"""
    enc = msg.encoding
    w, h, step = msg.width, msg.height, msg.step
    buf = bytes(msg.data)

    if enc == 'rgb8':
        img = QImage(buf, w, h, step, QImage.Format_RGB888)
    elif enc == 'bgr8':
        fmt = getattr(QImage, 'Format_BGR888', None)
        if fmt is not None:
            img = QImage(buf, w, h, step, fmt)
        else:
            img = QImage(buf, w, h, step, QImage.Format_RGB888).rgbSwapped()
    elif enc == 'mono8':
        img = QImage(buf, w, h, step, QImage.Format_Grayscale8)
    else:
        raise ValueError(f"unsupported image encoding: {enc}")

    return img.copy()  # 脱离 buf，避免悬垂引用
