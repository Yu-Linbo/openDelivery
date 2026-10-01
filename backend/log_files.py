"""Read-only, bounded access to the project's diagnostic text logs."""
from pathlib import Path

MAX_READ_BYTES = 2 * 1024 * 1024


def _within(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def roots(project):
    project = Path(project).resolve()
    return {"recording": project / "log_bag"}


def resolve_log(project, raw_path):
    project = Path(project).resolve()
    if ".." in Path(str(raw_path)).parts:
        raise ValueError("path traversal is not allowed")
    path = (project / str(raw_path)).resolve()
    if not any(_within(path, root.resolve()) for root in roots(project).values()):
        raise ValueError("path is outside diagnostic log directories")
    if path.suffix.lower() not in (".log", ".txt"):
        raise ValueError("only .log and .txt files can be viewed")
    if not path.is_file():
        raise FileNotFoundError("log file not found")
    return path


def read_log(project, raw_path):
    path = resolve_log(project, raw_path)
    with path.open("rb") as stream:
        stream.seek(0, 2)
        size = stream.tell()
        offset = max(0, size - MAX_READ_BYTES)
        stream.seek(offset)
        data = stream.read(MAX_READ_BYTES)
    # Never display a fragment of the first line or a split UTF-8 sequence.
    if offset:
        data = data.partition(b"\n")[2]
    return {"path": path.relative_to(Path(project).resolve()).as_posix(),
            "text": data.decode("utf-8", errors="replace"), "bytes": size,
            "truncated": bool(offset), "limit_bytes": MAX_READ_BYTES}
