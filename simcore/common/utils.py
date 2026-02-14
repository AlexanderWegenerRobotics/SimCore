import yaml
from pathlib import Path

def load_yaml(path: str):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data

def get_asset_path(relative_path=None):
    """Returns absolute path to SimCore's assets directory."""
    assets_dir = Path(__file__).parent.parent.parent / "assets"
    if relative_path:
        return str(assets_dir / relative_path)
    return str(assets_dir)