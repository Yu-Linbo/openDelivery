"""ROS-compatible platform logging, with bounded file retention."""
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path


class RosFormatter(logging.Formatter):
    def format(self, record):
        level = {"WARNING": "WARN", "CRITICAL": "FATAL"}.get(record.levelname, record.levelname)
        message = record.getMessage()
        if record.exc_info:
            message += "\n" + self.formatException(record.exc_info)
        return f"[{level}] [{record.created:.9f}] [{record.name}]: {message}"


def configure(log_dir):
    logger = logging.getLogger("opendelivery")
    if logger.handlers:
        return logger
    logger.setLevel(logging.INFO)
    logger.propagate = False
    formatter = RosFormatter()
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    logger.addHandler(console)
    try:
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        handler = RotatingFileHandler(Path(log_dir) / "platform.log", maxBytes=5 * 1024 * 1024,
                                      backupCount=5, encoding="utf-8")
        handler.namer = lambda name: name + ".log"
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    except OSError:
        logger.warning("platform log file unavailable", exc_info=True)
    return logger
