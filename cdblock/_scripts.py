"""The standalone scripts under scripts/, loaded for the package's own use."""
import importlib.util
import pathlib
import sys

SCRIPTS = pathlib.Path(__file__).resolve().parents[1] / "scripts"
_ABSENT = object()


def load(folder, name, **imports):
    """scripts/folder/name.py, loaded from its file as a module of this package's own. The
    scripts stay standalone, for a reader to run from their folders, so the package loads them
    instead of keeping a copy. Nothing is added to sys.path and the scripts' top-level names stay
    free: imports are the modules the script imports by such a name, in place only while it
    loads."""
    spec = importlib.util.spec_from_file_location(f"{__package__}._{name}", SCRIPTS / folder / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    before = {key: sys.modules.get(key, _ABSENT) for key in imports}
    sys.modules.update(imports)
    try:
        spec.loader.exec_module(module)
    finally:
        for key, value in before.items():
            if value is _ABSENT:
                del sys.modules[key]
            else:
                sys.modules[key] = value
    return module
