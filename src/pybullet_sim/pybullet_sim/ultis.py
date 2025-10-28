import os
from ament_index_python.packages import get_package_share_directory


# Utility: resolve 'package://' URIs into absolute filesystem paths
def resolve_package_path(path: str) -> str:
    if not path.startswith("package://"):
        return path
    pkg_name = path.split("/")[2]
    rel_path = "/".join(path.split("/")[3:])
    pkg_share = get_package_share_directory(pkg_name)
    return os.path.join(pkg_share, rel_path)
