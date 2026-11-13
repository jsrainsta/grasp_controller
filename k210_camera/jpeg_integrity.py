"""Check decoder diagnostics without redirecting another thread's stderr."""
import json
from pathlib import Path
import subprocess
import sys


def check_jpeg(path):
    path=Path(path).resolve()
    raw=path.read_bytes()
    if len(raw)>30*1024*1024 or not raw.startswith(b"\xff\xd8") or not raw.endswith(b"\xff\xd9"):
        return {"valid":False,"reason":"invalid_jpeg_markers_or_size"}
    # OpenCV/libjpeg warnings bypass Python warnings; isolate the decoder so
    # transport/OCR threads keep their own stderr and no corrupt buffer is reused.
    code=("import cv2,numpy as np,sys,json; "
          "im=cv2.imdecode(np.fromfile(sys.argv[1],np.uint8),cv2.IMREAD_COLOR); "
          "print(json.dumps(None if im is None else list(im.shape)))")
    try:
        result=subprocess.run([sys.executable,"-c",code,str(path)],
                              capture_output=True,timeout=10,check=False)
    except (OSError,subprocess.TimeoutExpired) as error:
        return {"valid":False,"reason":"decoder_probe_failed","diagnostic":str(error)}
    diagnostic=result.stderr.decode("utf-8",errors="replace").strip()
    if result.returncode or diagnostic:
        return {"valid":False,"reason":"decoder_reported_error_or_warning",
                "diagnostic":diagnostic,"returncode":result.returncode}
    try:
        shape=json.loads(result.stdout)
    except (ValueError,UnicodeDecodeError):
        shape=None
    if shape!=[480,640,3]:
        return {"valid":False,"reason":"unexpected_decoded_shape","shape":shape}
    return {"valid":True,"reason":"decoded_without_reported_warning","shape":shape}
