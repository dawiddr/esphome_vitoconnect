"""Catch the ESPHome component-ID pointer regression without a device toolchain.

Run: python tests/check_id_reference_regression.py [repository]
Requires PyYAML and g++. This uses the pointer-aware host adapter, not a full
ESPHome firmware build. Each of the four production lambdas is independently
mutated back to its broken alias declarations as an expected-failure control.
Production files are never modified; mutations are made in temporary copies.
"""
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


def invoke(root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(root / 'tests/host_safety_checks.py'), str(root)],
        text=True, capture_output=True, timeout=90,
    )



def pointer_access_failure(result: subprocess.CompletedProcess[str]) -> bool:
    """Accept the intended GCC or Clang member-access error, never any error."""
    if result.returncode == 0:
        return False
    return bool(re.search(
        r"error: [^\n]*(?:request for member[^\n]*which is of pointer type|"
        r"member reference[^\n]*is a pointer; did you mean to use)", result.stderr,
    ))


def copy_host_inputs(root: Path, target: Path) -> None:
    """Copy only the public source files used by the host compiler."""
    names = [Path("heating_optolink.yaml"), Path("tests/host_safety_checks.py"),
             Path("tests/host_stubs.py")]
    names += [path.relative_to(root) for path in (root / "components/vitoconnect").rglob("*")
              if path.suffix in {".h", ".cpp"} and path.is_file()
              and not any(part.startswith(".") or part == "__pycache__"
                          for part in path.relative_to(root).parts)]
    for name in names:
        source = root / name
        if not source.is_file() or any((root / part).is_symlink()
                                      for part in (name, *name.parents) if part != Path(".")):
            raise RuntimeError(f"Missing or linked host input: {name}")
        destination = target / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)

def main() -> None:
    root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
    original = (root / 'heating_optolink.yaml').read_text()
    pair = re.compile(
        r'auto &n = \*id\(heating_setpoint_normal_direct\);\n\s*'
        r'auto &r = \*id\(heating_setpoint_reduced_direct\);'
    )
    matches = list(pair.finditer(original))
    if len(matches) != 4:
        raise AssertionError(f'Expected four corrected alias pairs, found {len(matches)}')
    labels = ('baseline restore', 'arm guard', 'baseline capture', 'status publication')
    for label, match in zip(labels, matches):
        with tempfile.TemporaryDirectory(prefix='vito-id-negative-') as tmp:
            copy = Path(tmp) / 'site'
            copy_host_inputs(root, copy)
            broken_pair = match.group().replace('= *id(', '= id(')
            mutated = original[:match.start()] + broken_pair + original[match.end():]
            (copy / 'heating_optolink.yaml').write_text(mutated)
            result = invoke(copy)
            if not pointer_access_failure(result):
                raise AssertionError(f'{label}: expected a pointer-access compile error\n{result.stdout}\n{result.stderr}')
        print(f'PASS: restoring the broken {label} aliases causes a compile failure', flush=True)
    result = invoke(root)
    if result.returncode != 0:
        raise AssertionError(f'Corrected YAML failed:\n{result.stdout}\n{result.stderr}')
    print(result.stdout, end='')
    print('PASS: all four expected-failure controls and all corrected host scenarios')


if __name__ == '__main__':
    main()
