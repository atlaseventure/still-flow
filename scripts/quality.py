#!/usr/bin/env python3
"""Run repository quality checks and emit JSON Lines diagnostics."""

import argparse
import importlib.metadata
import json
import re
import shlex
import subprocess
import sys
import time
import tomllib
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from tempfile import TemporaryDirectory

from quality_cache import TidyCache

ROOT = Path(__file__).resolve().parents[1]
CPP = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".cu", ".cuh"}
SOURCES = {".c", ".cc", ".cpp", ".cxx", ".cu"}


def emit(tool, status, **fields):
    print(json.dumps({"tool": tool, "status": status, **fields}, ensure_ascii=True), flush=True)


def run(command):
    return subprocess.run(command, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", check=False)


def executable(name):
    return str(Path(sys.executable).parent / name)


def inventory():
    result = run(["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"])
    if result.returncode:
        raise RuntimeError(result.stderr.strip())
    return sorted({name for name in result.stdout.split("\0") if name and (ROOT / name).is_file()})


def cmake_file(name):
    return Path(name).name == "CMakeLists.txt" or name.endswith((".cmake", ".cmake.in"))


def verify_tools():
    configuration = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    for requirement in configuration["dependency-groups"]["quality"]:
        name, version = requirement.split("==")
        try:
            installed = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            installed = "missing"
        if installed != version:
            raise RuntimeError(
                f"Expected {name}=={version}, found {installed}; "
                "run with: uv run --locked --group quality python scripts/quality.py <command>"
            )


def format_files(files, check):
    changed = []
    for name in files:
        suffix = Path(name).suffix
        if suffix in CPP:
            command = [executable("clang-format"), "--style=file", "--fallback-style=none", "--Werror", name]
        elif suffix == ".py":
            command = [executable("ruff"), "format", "--quiet", "--stdin-filename", name, "-"]
        elif cmake_file(name):
            command = [executable("cmake-format"), name]
        else:
            continue
        path = ROOT / name
        original = path.read_bytes()
        result = subprocess.run(command, cwd=ROOT, input=original, capture_output=True, check=False)
        if result.returncode:
            emit("format", "error", file=name, stderr=result.stderr.decode("utf-8", errors="replace"))
            return 2
        if result.stdout != original:
            changed.append(name)
            if not check:
                path.write_bytes(result.stdout)
    emit("format", "failed" if check and changed else "passed", changed=changed, check=check)
    return int(check and bool(changed))


def lint_scripts(files):
    python_files = [name for name in files if name.endswith(".py")]
    if python_files:
        result = run([executable("ruff"), "check", "--output-format=json", *python_files])
        emit(
            "ruff",
            "passed" if result.returncode == 0 else "failed",
            diagnostics=json.loads(result.stdout) if result.stdout.strip() else [],
            stderr=result.stderr,
        )
        if result.returncode:
            return result.returncode
    cmake_files = [name for name in files if cmake_file(name)]
    if cmake_files:
        result = run([executable("cmake-lint"), "--suppress-decorations", *cmake_files])
        diagnostics = []
        for line in result.stdout.splitlines():
            match = re.fullmatch(r"(.+):(\d+),(\d+): \[([A-Z]\d+)\] (.*)", line)
            if match:
                file, row, column, code, message = match.groups()
                diagnostics.append(
                    {"file": file, "line": int(row), "column": int(column) + 1, "code": code, "message": message}
                )
            elif line.strip():
                diagnostics.append({"message": line})
        emit(
            "cmake-lint",
            "passed" if result.returncode == 0 else "failed",
            diagnostics=diagnostics,
            stderr=result.stderr,
        )
        if result.returncode:
            return result.returncode
    return 0


CURAND_SHIM = "curand_mtgp32_kernel.h"


def cuda_header_shims(cuda_path):
    """Return include arguments for headers Clang's CUDA wrapper needs but the toolkit omits.

    Clang's runtime wrapper unconditionally includes cuRAND's MTGP32 kernel header, which minimal
    toolkit installations without cuRAND lack. NVCC never includes it, so analysis uses an empty stand-in.
    """
    roots = [cuda_path / "include", *sorted(cuda_path.glob("targets/*/include"))]
    if any((root / CURAND_SHIM).is_file() for root in roots):
        return []
    directory = ROOT / ".local" / "cuda-shims"
    header = directory / CURAND_SHIM
    content = "// Empty stand-in for Clang CUDA analysis; the toolkit has no cuRAND headers.\n#pragma once\n"
    if not header.is_file() or header.read_text(encoding="utf-8") != content:
        directory.mkdir(parents=True, exist_ok=True)
        header.write_text(content, encoding="utf-8")
    return ["-isystem", str(directory)]


def cuda_arguments(entry):
    """Translate NVCC driver flags while preserving the configured compilation context."""
    args = entry.get("arguments") or shlex.split(entry["command"])
    if Path(args[0]).name != "nvcc":
        return args
    compiler = Path(args[0])
    if not compiler.is_absolute():
        compiler = Path(entry["directory"]) / compiler
    cuda_path = compiler.resolve().parent.parent
    translated = ["clang++", f"--cuda-path={cuda_path}", *cuda_header_shims(cuda_path)]
    iterator = iter(args[1:])
    for arg in iterator:
        if arg == "-forward-unknown-to-host-compiler":
            continue
        if arg.startswith(("--generate-code", "-gencode")):
            value = arg.split("=", 1)[1] if "=" in arg else next(iterator)
            arch = re.search(r"sm_(\d+[a-z]?)", value)
            if arch:
                translated.append(f"--cuda-gpu-arch=sm_{arch[1]}")
        elif arg == "-x":
            language = next(iterator)
            translated.extend([arg, "cuda" if language == "cu" else language])
        elif arg.startswith("-Xcompiler="):
            translated.extend(arg.split("=", 1)[1].split(","))
        elif arg in {"-Xcompiler", "--compiler-options"}:
            translated.extend(next(iterator).split(","))
        else:
            translated.append(arg.replace('\\"', '"'))
    return translated


def lint_cpp(files, build_dir, require_cuda, extra_args, jobs, no_cache):
    import yaml

    database = build_dir / "compile_commands.json"
    if not database.is_file():
        raise RuntimeError(f"Missing {database}; configure and build the project first")
    entries = json.loads(database.read_text(encoding="utf-8"))
    sources = {str((ROOT / name).resolve()): name for name in files if Path(name).suffix in SOURCES}
    configuration = {}
    cache_file = build_dir / "CMakeCache.txt"
    if cache_file.is_file():
        for line in cache_file.read_text(encoding="utf-8").splitlines():
            match = re.fullmatch(r"([^:#/][^:]*):[^=]+=(.*)", line)
            if match:
                configuration[match[1]] = match[2]

    def enabled(option):
        return configuration.get(option, "ON").upper() not in {"OFF", "FALSE", "NO", "0", ""}

    def inactive(name):
        if configuration.get("STILLFLOW_ANDROID_RUNNER") == "ON":
            return not name.startswith("tools/android/")
        if name.startswith("tools/android/"):
            return True
        if name.startswith("api/android/"):
            return configuration.get("STILLFLOW_BUILD_ANDROID_JNI") != "ON"
        if name.startswith("lib/backends/gles/"):
            return configuration.get("STILLFLOW_TARGET_PLATFORM") != "Android"
        if name.startswith(("lib/adapters/", "lib/backends/cuda/")) or name == "api/c/src/desktop.cpp":
            return not enabled("STILLFLOW_BUILD_ADAPTERS")
        if name.startswith("cli/") or name == "tests/video.cpp":
            return not enabled("STILLFLOW_BUILD_CLI") or (name.startswith("tests/") and not enabled("BUILD_TESTING"))
        if name == "tests/backends.cpp":
            return not enabled("BUILD_TESTING") or not enabled("STILLFLOW_BUILD_ADAPTERS")
        if name.startswith("api/c/") or name == "tests/c_api.cpp":
            return not enabled("STILLFLOW_BUILD_C_API") or (name.startswith("tests/") and not enabled("BUILD_TESTING"))
        if name.startswith("tests/"):
            return not enabled("BUILD_TESTING")
        if name.startswith("examples/"):
            return not enabled("STILLFLOW_BUILD_EXAMPLES") or not enabled("STILLFLOW_BUILD_C_API")
        if name in {"lib/vision/src/opencv.cpp", "lib/vision/src/motion.cpp"}:
            return not enabled("STILLFLOW_WITH_OPENCV")
        return False

    selected = []
    covered = set()
    for entry in entries:
        source = str((Path(entry["directory"]) / entry["file"]).resolve())
        if source not in sources:
            continue
        if source.endswith(".cu"):
            entry = {**entry, "arguments": cuda_arguments(entry)}
            entry.pop("command", None)
        selected.append(entry)
        covered.add(source)
    missing = sorted(set(sources) - covered)
    skipped = [
        sources[name]
        for name in missing
        if not (require_cuda and name.endswith(".cu")) and (inactive(sources[name]) or name.endswith(".cu"))
    ]
    missing = [sources[name] for name in missing if sources[name] not in skipped]
    if missing:
        raise RuntimeError(f"Sources missing from compilation database: {', '.join(missing)}")
    if not selected:
        raise RuntimeError("Compilation database contains no project sources")
    emit("clang-tidy-coverage", "passed", files=sorted(sources[name] for name in covered), skipped=skipped)
    cache = TidyCache(ROOT, database, extra_args, not no_cache)
    keys = {source: cache.key(source) for source in covered}
    cached = {source for source in covered if cache.hit(source, keys[source])}
    emit("clang-tidy-plan", "passed", jobs=jobs, cached=len(cached), pending=len(covered - cached))
    for source in sorted(cached):
        emit("clang-tidy", "passed", file=sources[source], diagnostics=[], cached=True)
    with TemporaryDirectory(prefix="work-", dir=cache.directory) as directory:
        temporary = Path(directory)
        (temporary / "compile_commands.json").write_text(json.dumps(selected), encoding="utf-8")

        def analyze(index, source):
            fixes = temporary / f"{index}.yaml"
            dependencies = temporary / f"{index}.d"
            started = time.time_ns()
            result = run(
                [
                    executable("clang-tidy"),
                    f"-p={temporary}",
                    f"--export-fixes={fixes}",
                    "--quiet",
                    "--use-color=false",
                    "--extra-arg=-v",
                    "--extra-arg=-H",
                    "--extra-arg=-Xclang",
                    "--extra-arg=-dependency-file",
                    "--extra-arg=-Xclang",
                    f"--extra-arg={dependencies}",
                    "--extra-arg=-Xclang",
                    "--extra-arg=-sys-header-deps",
                    "--extra-arg=-Wp,-MT,quality",
                    *[f"--extra-arg={arg}" for arg in extra_args],
                    source,
                ]
            )
            diagnostics = []
            if fixes.exists():
                report = yaml.safe_load(fixes.read_text(encoding="utf-8")) or {}
                diagnostics = report.get("Diagnostics", [])
            return result, diagnostics, dependencies, started

        with ThreadPoolExecutor(max_workers=jobs) as executor:
            futures = {
                executor.submit(analyze, index, source): source for index, source in enumerate(sorted(covered - cached))
            }
            for future in as_completed(futures):
                source = futures[future]
                result, diagnostics, dependencies, started = future.result()
                emit(
                    "clang-tidy",
                    "passed" if result.returncode == 0 else "failed",
                    file=sources[source],
                    diagnostics=diagnostics,
                    cached=False,
                    **(
                        {"stderr": result.stderr, "stdout": result.stdout}
                        if result.returncode and not diagnostics
                        else {}
                    ),
                )
                if result.returncode:
                    for pending in futures:
                        pending.cancel()
                    return 1
                if not diagnostics:
                    working_directory = next(
                        entry["directory"]
                        for entry in selected
                        if str((Path(entry["directory"]) / entry["file"]).resolve()) == source
                    )
                    cache.save(source, keys[source], dependencies, result.stderr, working_directory, started)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["format", "lint", "check"])
    parser.add_argument("--check", action="store_true", help="Check formatting without writing files")
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build")
    parser.add_argument("--require-cuda", action="store_true", help="Fail if CUDA is absent from the build")
    parser.add_argument("--extra-arg", action="append", default=[], help="Additional Clang compiler argument")
    parser.add_argument("--jobs", "-j", type=int, default=12, help="Concurrent clang-tidy processes (default: 12)")
    parser.add_argument(
        "--no-cache", action="store_true", help="Run analysis without reading or writing cached results"
    )
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    try:
        verify_tools()
        files = inventory()
        if args.command in {"format", "check"}:
            status = format_files(files, args.check or args.command == "check")
            if status or args.command == "format":
                return status
        return lint_scripts(files) or lint_cpp(
            files, args.build_dir.resolve(), args.require_cuda, args.extra_arg, args.jobs, args.no_cache
        )
    except (OSError, ValueError, RuntimeError, importlib.metadata.PackageNotFoundError) as error:
        emit("quality", "error", message=str(error))
        return 2


if __name__ == "__main__":
    sys.exit(main())
