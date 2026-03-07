# PyInstaller spec for MushIO V1.0 GUI
#
# Build with:
#   pip install pyinstaller
#   pyinstaller mushio_gui.spec
#
# Output: dist/MushIO_GUI.exe  (single standalone file, ~60-80 MB)
#
# Notes:
#   - Run from the project root directory
#   - The exe bundles Python + all dependencies; no install needed on target PC
#   - opencv-python adds ~40 MB; omit the hiddenimports entry if not using webcam

import sys
from pathlib import Path

ROOT = Path(SPECPATH)  # project root (same dir as this .spec file)

a = Analysis(
    [str(ROOT / 'host' / 'gui.py')],
    pathex=[str(ROOT / 'host')],   # so `import receiver` resolves
    binaries=[],
    datas=[],
    hiddenimports=[
        'matplotlib.backends.backend_tkagg',
        'cv2',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='MushIO_GUI',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,   # no console window (set True to see error output)
    disable_windowed_traceback=False,
    argv_emulation=False,
    icon=None,       # replace with 'icon.ico' if you have one
)
