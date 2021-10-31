__version__ = "6.1.7"

from zuper_commons.logs import ZLogger

logger = ZLogger(__name__)
import os

path = os.path.dirname(os.path.dirname(__file__))

logger.debug(f"aido-analyze version {__version__} path {path}")

from .utils_drawing import *
from .utils_video import *
