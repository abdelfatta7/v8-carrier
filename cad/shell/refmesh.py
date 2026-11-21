"""Load the reference assembly JSON (robot frame, mm) into trimesh meshes."""
import base64, json, os
import numpy as np

REF = os.path.join(os.path.dirname(__file__), "reference", "model_2026-11-16.json")


def load(path=REF):
    import trimesh
    d = json.load(open(path))
    out = {}
    for m in d["meshes"]:
        v = np.frombuffer(base64.b64decode(m["v"]), dtype="<i2").astype(float).reshape(-1, 3) * 0.05
        dt = "<u2" if m.get("itype") == "u16" else "<u4"
        f = np.frombuffer(base64.b64decode(m["i"]), dtype=dt).astype(np.int64).reshape(-1, 3)
        out[m["name"]] = trimesh.Trimesh(v, f, process=False)
    return out, d
