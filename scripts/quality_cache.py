"""Cache successful clang-tidy analyses against their compilation inputs."""

import hashlib
import importlib.metadata
import json
import os
import re
import shlex
from pathlib import Path
from tempfile import TemporaryDirectory


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


class TidyCache:
    def __init__(self, root, database, extra_args, enabled):
        self.directory = root / ".local/quality-cache"
        self.directory.mkdir(parents=True, exist_ok=True)
        self.enabled = enabled
        self.contents = {}
        self.trees = {}
        self.tree_times = {}
        package = importlib.metadata.distribution("clang-tidy")
        binary = package.locate_file("clang_tidy/data/bin/clang-tidy")
        environment = {
            key: os.environ.get(key)
            for key in (
                "PATH",
                "CPATH",
                "CPLUS_INCLUDE_PATH",
                "C_INCLUDE_PATH",
                "OBJC_INCLUDE_PATH",
                "SDKROOT",
                "GCC_EXEC_PREFIX",
                "COMPILER_PATH",
                "CUDA_PATH",
                "SOURCE_DATE_EPOCH",
            )
        }
        self.context = digest(
            {
                "database": database.read_text(encoding="utf-8"),
                "extra_args": extra_args,
                "environment": environment,
                "tool": [str(binary), package.version, self.file_digest(binary)],
                "runner": [
                    self.file_digest(root / name)
                    for name in ("scripts/quality.py", "scripts/quality_cache.py", "pyproject.toml", "uv.lock")
                ],
            }
        )

    def file_digest(self, path):
        path = Path(path)
        info = path.stat()
        identity = (str(path), info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)
        if identity not in self.contents:
            self.contents[identity] = hashlib.sha256(path.read_bytes()).hexdigest()
        return self.contents[identity]

    def key(self, source):
        configs = []
        for directory in Path(source).parents:
            for name in (".clang-tidy", ".clang-format"):
                path = directory / name
                if path.is_file():
                    configs.append((str(path), self.file_digest(path)))
        return digest([self.context, source, self.file_digest(source), configs])

    def tree_digest(self, directory):
        """Detect new/shadowing headers, including previously missing include directories."""
        if directory not in self.trees:
            root = Path(directory)
            entries = []
            seen = set()
            changed_at = 0
            if root.is_dir():

                def fail(error):
                    raise error

                for name, dirs, files in os.walk(root, followlinks=True, onerror=fail):
                    path = Path(name)
                    info = path.stat()
                    identity = (info.st_dev, info.st_ino)
                    if identity in seen or path.resolve() == self.directory:
                        dirs[:] = []
                        continue
                    seen.add(identity)
                    changed_at = max(changed_at, info.st_mtime_ns, info.st_ctime_ns)
                    dirs[:] = sorted(child for child in dirs if child != ".git")
                    entries.append((name, identity, info.st_mtime_ns, info.st_ctime_ns, dirs[:], sorted(files)))
            self.trees[directory] = digest(entries)
            self.tree_times[directory] = changed_at
        return self.trees[directory]

    def fingerprint(self, dependencies, directories):
        return digest(
            [
                [(name, self.file_digest(name)) for name in dependencies],
                [(name, self.tree_digest(name)) for name in directories],
            ]
        )

    def path(self, source):
        return self.directory / f"{digest(source)}.json"

    def hit(self, source, key):
        if not self.enabled:
            return False
        try:
            record = json.loads(self.path(source).read_text(encoding="utf-8"))
            return record["key"] == key and record["fingerprint"] == self.fingerprint(
                record["dependencies"], record["directories"]
            )
        except OSError, ValueError, KeyError, TypeError:
            return False

    def save(self, source, key, depfile, compiler_output, working_directory, started):
        if not self.enabled:
            return
        try:
            text = depfile.read_text(encoding="utf-8").replace("\\\n", "")
            dependencies = sorted(
                {
                    str((Path(working_directory) / name).absolute())
                    for name in shlex.split(text.split(":", 1)[1].replace("$$", "$"))
                }
            )
            if source not in dependencies:
                return
            dependencies = set(dependencies)
            directories = set()
            searching = False
            for line in compiler_output.splitlines():
                included = re.fullmatch(r"\.+ (.+)", line)
                if included:
                    dependencies.add(str((Path(working_directory) / included[1]).absolute()))
                if line.startswith("#include") and line.endswith("search starts here:"):
                    searching = True
                elif line == "End of search list.":
                    searching = False
                elif searching and line.startswith(" "):
                    directories.add(str((Path(working_directory) / line.strip()).absolute()))
                elif line.startswith('ignoring nonexistent directory "'):
                    directories.add(line.split('"')[1])
                elif line.startswith("Found candidate GCC installation: "):
                    directories.add(str(Path(line.split(": ", 1)[1]).parent.parent))
            dependencies = sorted(dependencies)
            directories.update(str(Path(name).parent) for name in dependencies)
            for name in dependencies:
                info = Path(name).stat()
                # Avoid caching files edited during analysis or time-dependent preprocessing.
                if max(info.st_mtime_ns, info.st_ctime_ns) >= started:
                    return
                if re.search(rb"\b__(?:DATE|TIME|TIMESTAMP)__\b", Path(name).read_bytes()):
                    return
            directories = sorted(directories)
            record = {
                "key": key,
                "dependencies": dependencies,
                "directories": directories,
                "fingerprint": self.fingerprint(dependencies, directories),
            }
            if any(self.tree_times[name] >= started for name in directories):
                return
            with TemporaryDirectory(dir=self.directory) as temporary:
                path = Path(temporary) / "entry.json"
                path.write_text(json.dumps(record), encoding="utf-8")
                path.replace(self.path(source))
        except OSError, ValueError, IndexError:
            # A missing/unusable dependency report leaves this source uncached.
            return
