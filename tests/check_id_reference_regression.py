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
            shutil.copytree(root, copy, ignore=shutil.ignore_patterns('__pycache__', '.esphome', '.git'))
            broken_pair = match.group().replace('= *id(', '= id(')
            mutated = original[:match.start()] + broken_pair + original[match.end():]
            (copy / 'heating_optolink.yaml').write_text(mutated)
            result = invoke(copy)
            if result.returncode == 0 or 'which is of pointer type' not in result.stderr:
                raise AssertionError(f'{label}: expected a pointer-access compile error\n{result.stdout}\n{result.stderr}')
        print(f'PASS: restoring the broken {label} aliases causes a compile failure', flush=True)
    result = invoke(root)
    if result.returncode != 0:
        raise AssertionError(f'Corrected YAML failed:\n{result.stdout}\n{result.stderr}')
    print(result.stdout, end='')
    print('PASS: all four expected-failure controls and all corrected host scenarios')


if __name__ == '__main__':
    main()
