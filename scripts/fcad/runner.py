"""fcad runner: call FreeCAD headless from the normal project venv.

    from fcad.runner import run
    res = run(job, workdir)          # job: dict (schema in README.md); returns the result dict

It writes the job to <workdir>/job.json, starts FreeCADCmd.exe on fc_build.py with FCAD_JOB pointing at
that file, waits, and returns the parsed <workdir>/result.json. FreeCAD's console output goes to
<workdir>/freecad_log.txt; on failure the error and the tail of that log are raised as FcadError.

FreeCADCmd is found through the environment variable FREECADCMD, else find_freecad_bin() below.
"""
from pathlib import Path
import json
import os
import subprocess

def find_freecad_bin(exe_name):
    """A FreeCAD bin/ executable: on PATH, in Program Files, or in a portable FreeCAD_* folder under a .tools or
    aero-tools directory next to this repository. None if not found."""
    import shutil
    hit = shutil.which(exe_name)
    if hit:
        return Path(hit)
    repo = Path(__file__).resolve().parents[2]
    roots = [Path(os.environ.get("ProgramFiles", "C:/Program Files")), repo.parent / "aero-tools",
             *repo.parent.glob("*/.tools"), *repo.parent.glob(".tools")]
    for root in roots:
        for cand in sorted(root.glob(f"FreeCAD*/bin/{exe_name}"), reverse=True):
            return cand
    return None


DEFAULT_FREECADCMD = find_freecad_bin("FreeCADCmd.exe")
FC_BUILD = Path(__file__).with_name("fc_build.py")


class FcadError(RuntimeError):
    pass


def freecadcmd():
    exe = Path(os.environ.get("FREECADCMD") or DEFAULT_FREECADCMD or "FreeCADCmd.exe")
    if not exe.exists():
        raise FcadError(f"FreeCADCmd not found at {exe}; set FREECADCMD")
    return exe


def _abs(p):
    return str(Path(p).resolve())


def run(job: dict, workdir, timeout=3600) -> dict:
    workdir = Path(workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)
    job = dict(job)
    job["result"] = _abs(workdir / "result.json")
    # absolute output paths, so FreeCAD's working directory does not matter
    out = dict(job.get("outputs", {}))
    if out.get("stl_dir"):
        out["stl_dir"] = _abs(out["stl_dir"])
    out["step"] = [dict(s, path=_abs(s["path"])) for s in out.get("step", [])]
    job["outputs"] = out
    job_path = workdir / "job.json"
    job_path.write_text(json.dumps(job), encoding="utf-8")
    res_path = Path(job["result"])
    if res_path.exists():
        res_path.unlink()
    log_path = workdir / "freecad_log.txt"
    env = dict(os.environ, FCAD_JOB=str(job_path))
    with open(log_path, "w", encoding="utf-8", errors="replace") as log:
        proc = subprocess.run([str(freecadcmd()), str(FC_BUILD)], env=env, stdout=log, stderr=subprocess.STDOUT,
                              timeout=timeout, cwd=str(workdir))
    tail = log_path.read_text(encoding="utf-8", errors="replace")[-4000:]
    if not res_path.exists():
        raise FcadError(f"FreeCAD wrote no result (exit {proc.returncode}). Log tail:\n{tail}")
    res = json.loads(res_path.read_text(encoding="utf-8"))
    if not res.get("ok") or proc.returncode != 0:
        raise FcadError(f"FreeCAD job failed (exit {proc.returncode}):\n{res.get('error', '')}\nLog tail:\n{tail}")
    return res
