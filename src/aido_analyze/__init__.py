__version__ = "6.0.11"

from zuper_commons.logs import ZLogger

logger = ZLogger(__name__)
logger.debug(f"aido-analyze version {__version__} path {__file__}")

from .utils_drawing import *
from .utils_video import *
