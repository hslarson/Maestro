import logging
from Maestro import Maestro

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Create Maestro object
maestro = Maestro('./maestro_settings.txt')

# Do stuff