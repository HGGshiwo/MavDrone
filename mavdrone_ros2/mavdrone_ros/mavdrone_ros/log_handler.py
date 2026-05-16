from __future__ import annotations

import logging
from logging import LogRecord

from rclpy.node import Node


class Ros2Handler(logging.Handler):
    def __init__(self, node: Node) -> None:
        super().__init__()

        self._node = node

    def emit(self, record: LogRecord) -> None:
        match record.levelno:
            case logging.CRITICAL:
                self._node.get_logger().fatal(str(record.msg))
            case logging.ERROR:
                self._node.get_logger().error(record.msg)
            case logging.WARNING:
                self._node.get_logger().warning(record.msg)
            case logging.INFO:
                self._node.get_logger().info(record.msg)
            case _:
                self._node.get_logger().debug(record.msg)  # All other levels is debug


def redirect_logging_to_ros(node: Node) -> None:
    logger = logging.getLogger()
    for handler in logger.handlers:
        logger.removeHandler(handler)

    logger.addHandler(Ros2Handler(node))
