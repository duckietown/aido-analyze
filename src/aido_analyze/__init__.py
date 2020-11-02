__version__ = "6.0.7"

from zuper_commons.logs import ZLogger

logger = ZLogger(__name__)
logger.info(f"aido-analyze {__version__}")

from .utils_drawing import *
from .utils_video import *
