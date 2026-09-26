"""
Upload a folder of frames to CVAT as a new task in the Junction organization.

    python cvat_upload.py s02_label/images --task session02_150frames
    python cvat_upload.py <images_dir> --task <name> [--project okra_g1] [--org Junction]

Reads CVAT_HOST and CVAT_TOKEN (personal access token) from ../.env. Creates the project
(one polygon label "okra") if it doesn't exist; refuses to create a second task with the same name.
"""

import argparse
import time
from pathlib import Path

import requests

ENV = Path(__file__).resolve().parent.parent / ".env"


def load_env():
    env = {}
    for line in ENV.read_text().splitlines():
        if "=" in line and not line.lstrip().startswith("#"):
            k, v = line.split("=", 1)
            env[k.strip()] = v.strip()
    return env["CVAT_HOST"].rstrip("/"), env["CVAT_TOKEN"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images")
    ap.add_argument("--task", required=True)
    ap.add_argument("--project", default="okra_g1")
    ap.add_argument("--org", default="Junction")
    args = ap.parse_args()

    host, token = load_env()
    s = requests.Session()
    s.headers["Authorization"] = f"Bearer {token}"
    s.params = {"org": args.org}

    def ok(r):
        if not r.ok:
            raise SystemExit(f"{r.request.method} {r.url.split('?')[0]} -> {r.status_code}: {r.text[:300]}")
        return r.json() if r.content else {}

    projects = ok(s.get(f"{host}/api/projects", params={"name": args.project}))["results"]
    project = next((p for p in projects if p["name"] == args.project), None)
    if project is None:
        project = ok(s.post(f"{host}/api/projects", json={
            "name": args.project,
            "labels": [{"name": "okra", "type": "polygon", "color": "#33cc33"}]}))
        print(f"created project {project['name']} (id {project['id']})")
    else:
        print(f"using project {project['name']} (id {project['id']})")

    existing = ok(s.get(f"{host}/api/tasks", params={"project_id": project["id"], "name": args.task}))["results"]
    if any(t["name"] == args.task for t in existing):
        raise SystemExit(f"task {args.task!r} already exists in {args.project}; pick another --task name")

    files = sorted(p for p in Path(args.images).iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    task = ok(s.post(f"{host}/api/tasks", json={"name": args.task, "project_id": project["id"]}))
    print(f"created task {task['name']} (id {task['id']}), uploading {len(files)} images...")

    handles = [open(p, "rb") for p in files]
    try:
        r = ok(s.post(f"{host}/api/tasks/{task['id']}/data",
                      data={"image_quality": 95, "sorting_method": "lexicographical"},
                      files=[(f"client_files[{i}]", (p.name, h, "image/jpeg")) for i, (p, h) in enumerate(zip(files, handles))]))
    finally:
        for h in handles:
            h.close()

    rq = r.get("rq_id")
    for _ in range(120):
        if rq:
            st = ok(s.get(f"{host}/api/requests/{rq}"))
            state, msg = st.get("status"), st.get("message", "")
        else:
            st = ok(s.get(f"{host}/api/tasks/{task['id']}/status"))
            state, msg = st.get("state"), st.get("message", "")
        if state in ("finished", "Finished"):
            break
        if state in ("failed", "Failed"):
            raise SystemExit(f"upload failed: {msg}")
        time.sleep(3)
    t = ok(s.get(f"{host}/api/tasks/{task['id']}"))
    print(f"done: task id {t['id']}, {t.get('size')} frames, status {t.get('status')}")
    print(f"open: {host}/tasks/{t['id']}")


if __name__ == "__main__":
    main()
