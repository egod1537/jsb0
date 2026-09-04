#!/usr/bin/env python3
"""Reject source includes that cross the documented JSB0 module boundaries."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


SOURCE_SUFFIXES = {".cpp", ".h", ".hpp", ".inl"}
INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')

RULES: dict[str, tuple[re.Pattern[str], ...]] = {
    "sim": tuple(
        re.compile(pattern)
        for pattern in (
            r"^(gui|flightui|app|runner|messaging|integration)/",
            r"^(imgui|implot|GLFW)(/|\.h)",
        )
    ),
    "common": tuple(
        re.compile(pattern)
        for pattern in (
            r"^(sim|gui|flightui|app|runner|messaging|contract|integration)/",
            r"^(imgui|implot|GLFW)(/|\.h)",
        )
    ),
    "contract": tuple(
        re.compile(pattern)
        for pattern in (
            r"^(sim|gui|flightui|app|runner|messaging|integration)/",
            r"^(imgui|implot|GLFW)(/|\.h)",
        )
    ),
    "flightui": tuple(
        re.compile(pattern)
        for pattern in (
            r"^(gui|messaging|contract|app|runner|integration)/",
            r"^sim/(runtime|gnc|jsbsim)/",
            r"^sim/Simulation(?:\.hpp|\.h)$",
        )
    ),
    "messaging": tuple(
        re.compile(pattern)
        for pattern in (
            r"^(gui|flightui|app|runner|integration)/",
            r"^(imgui|implot|GLFW)(/|\.h)",
        )
    ),
    "gui": tuple(
        re.compile(pattern)
        for pattern in (
            r"^messaging/GuiSimBridge(?:\.hpp|\.h)$",
            r"^sim/runtime/SimRuntime(?:\.hpp|\.h)$",
            r"^sim/Simulation(?:\.hpp|\.h)$",
            r"^sim/(jsbsim|gnc/autopilot)/",
        )
    ),
}

THREAD_OWNERSHIP_RULES: tuple[
    tuple[str, re.Pattern[str], tuple[re.Pattern[str], ...]], ...
] = (
    (
        "GUI-side messaging",
        re.compile(r"^src/messaging/SimMessageClient(?:\.cpp|\.h|\.hpp|\.inl)$"),
        tuple(
            re.compile(pattern)
            for pattern in (
                r"^messaging/GuiSimBridge(?:\.hpp|\.h)$",
                r"^sim/runtime/SimRuntime(?:\.hpp|\.h)$",
                r"^sim/Simulation(?:\.hpp|\.h)$",
                r"^sim/(jsbsim|gnc/autopilot)/",
            )
        ),
    ),
    (
        "simulation-side messaging",
        re.compile(r"^src/messaging/GuiSimBridge(?:\.cpp|\.h|\.hpp|\.inl)$"),
        tuple(
            re.compile(pattern)
            for pattern in (
                r"^messaging/SimMessageClient(?:\.hpp|\.h)$",
                r"^(gui|flightui)/",
                r"^(imgui|implot|GLFW)(/|\.h)",
            )
        ),
    ),
)

APPLICATION_MAIN_LOOP_FORBIDDEN: tuple[re.Pattern[str], ...] = tuple(
    re.compile(pattern)
    for pattern in (
        r"RunScheduled(?:Simulation|Gui)Tick",
        r"nextSimulationTick",
        r"simulationInterval",
        r"SimRuntime\s*::\s*Tick",
        r"simRuntime_?\s*(?:->|\.)\s*Tick\s*\(",
    )
)

GNC_RULES: dict[str, tuple[re.Pattern[str], ...]] = {
    "navigation": tuple(
        re.compile(pattern)
        for pattern in (
            r"^sim/gnc/(guidance|energy|control|autopilot)/",
        )
    ),
    "guidance": tuple(
        re.compile(pattern)
        for pattern in (
            r"^sim/gnc/autopilot/",
        )
    ),
    "energy": tuple(
        re.compile(pattern)
        for pattern in (
            r"^sim/gnc/(navigation|guidance|autopilot)/",
        )
    ),
    "tecs": tuple(
        re.compile(pattern)
        for pattern in (
            r"^sim/gnc/(navigation|guidance|autopilot)/",
        )
    ),
    "control": tuple(
        re.compile(pattern)
        for pattern in (
            r"^sim/gnc/(navigation|guidance|energy|autopilot)/",
        )
    ),
    "autopilot/px4": (re.compile(r"^sim/gnc/autopilot/experimental/"),),
    "autopilot/experimental": (
        re.compile(r"^sim/gnc/autopilot/px4/"),
        re.compile(r"^sim/gnc/autopilot/PX4Autopilot(?:\.hpp|\.h)$"),
    ),
}


def check(root: Path) -> list[str]:
    violations: list[str] = []
    source_root = root / "src"
    deprecated_hold_root = source_root / "sim" / "gnc" / "hold"
    if deprecated_hold_root.exists():
        violations.append("deprecated GNC directory: src/sim/gnc/hold")

    for module, forbidden in RULES.items():
        module_root = source_root / module
        if not module_root.is_dir():
            violations.append(f"missing module directory: src/{module}")
            continue
        for path in sorted(module_root.rglob("*")):
            if not path.is_file() or path.suffix not in SOURCE_SUFFIXES:
                continue
            for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), start=1
            ):
                match = INCLUDE.match(line)
                if not match:
                    continue
                include = match.group(1)
                if any(pattern.search(include) for pattern in forbidden):
                    relative = path.relative_to(root).as_posix()
                    violations.append(f"{relative}:{line_number}: forbidden include {include}")

    gnc_root = source_root / "sim" / "gnc"
    for layer, forbidden in GNC_RULES.items():
        layer_root = gnc_root / layer
        if not layer_root.is_dir():
            continue
        for path in sorted(layer_root.rglob("*")):
            if not path.is_file() or path.suffix not in SOURCE_SUFFIXES:
                continue
            for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), start=1
            ):
                match = INCLUDE.match(line)
                if not match:
                    continue
                include = match.group(1)
                if any(pattern.search(include) for pattern in forbidden):
                    relative = path.relative_to(root).as_posix()
                    violations.append(
                        f"{relative}:{line_number}: forbidden GNC layer include {include}"
                    )

    for ownership, source_pattern, forbidden in THREAD_OWNERSHIP_RULES:
        for path in sorted(source_root.rglob("*")):
            if not path.is_file() or path.suffix not in SOURCE_SUFFIXES:
                continue
            relative = path.relative_to(root).as_posix()
            if not source_pattern.search(relative):
                continue
            for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), start=1
            ):
                match = INCLUDE.match(line)
                if not match:
                    continue
                include = match.group(1)
                if any(pattern.search(include) for pattern in forbidden):
                    violations.append(
                        f"{relative}:{line_number}: forbidden {ownership} include {include}"
                    )

    application_path = source_root / "app" / "Application.cpp"
    if application_path.is_file():
        for line_number, line in enumerate(
            application_path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            if any(
                pattern.search(line) for pattern in APPLICATION_MAIN_LOOP_FORBIDDEN
            ):
                violations.append(
                    "src/app/Application.cpp:"
                    f"{line_number}: simulation/GUI scheduling leaked into Application"
                )
    return violations


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    violations = check(args.root.resolve())
    if violations:
        print("\n".join(violations))
        return 1
    print("architecture include boundaries: passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
