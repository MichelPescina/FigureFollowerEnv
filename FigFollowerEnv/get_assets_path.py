import inspect
import os.path
from pathlib import Path

def get_assets_path():
    assets_path = os.path.abspath(inspect.getsourcefile(lambda:0))
    assets_path = Path(assets_path).parent / Path("assets")
    return str(assets_path)
