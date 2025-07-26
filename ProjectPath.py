import sys
import os

PROJECT_PATH = os.path.abspath(os.path.dirname(__file__))
if PROJECT_PATH not in sys.path:
    sys.path.append(PROJECT_PATH)
