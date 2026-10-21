"""Run bounded standard-library regressions for validation and safe input copies."""
from __future__ import annotations

import contextlib
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import firmware_checks
import validate
import validate_documents as documents
import check_id_reference_regression as id_check


def completed(code: int = 0, stdout: str = "", stderr: str = ""):
    return subprocess.CompletedProcess([], code, stdout, stderr)


class ValidationChecks(unittest.TestCase):
    def test_clean_environment_removes_source_and_test_overrides(self):
        overrides = {key: "override" for key in (
            "PYTEST_ADDOPTS", "PYTEST_PLUGINS", "PYTEST_DISABLE_PLUGIN_AUTOLOAD",
            "PYTHONPATH", "PYTHONHOME", "PYTHONOPTIMIZE", "PYTHONSTARTUP",
            "GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE", "GIT_CONFIG_COUNT",
        )}
        with patch.dict(os.environ, overrides):
            cleaned = validate.clean_environment()
        self.assertFalse(overrides.keys() & cleaned.keys())
        self.assertEqual(cleaned["PYTHONNOUSERSITE"], "1")

    def test_active_optimization_is_rejected_before_other_checks(self):
        result = subprocess.run([sys.executable, "-O", str(ROOT / "scripts/validate.py"), "--documents-only"],
                                capture_output=True, text=True, env=validate.clean_environment())
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("without Python optimization", result.stderr)

    def test_documents_only_does_not_check_environment_or_run_tests(self):
        with patch.object(validate, "describe_source"), patch.object(validate, "document_checks") as docs, \
             patch.object(validate, "local_checks", side_effect=AssertionError("Local tests ran")):
            validate.main(["--documents-only"])
            docs.assert_called_once()

    def test_invalid_firmware_selections_fail_before_any_checks(self):
        for arguments in (["--firmware", "build", "--configuration", "esp32-p300"],
                          ["--configuration", "esp32-p300"], ["--firmware", "schema"],
                          ["--firmware", "generate", "--configuration", "site-p300"],
                          ["--documents-only", "--firmware", "schema", "--configuration", "esp32-p300"],
                          ["--allow-toolchain-setup"]):
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()), \
                 patch.object(validate, "describe_source", side_effect=AssertionError("Source checked")):
                with self.assertRaises(SystemExit) as error:
                    validate.main(arguments)
                self.assertEqual(error.exception.code, 2)

    def test_bad_host_results_fail(self):
        for result in (completed(1, 'PASS: fake\nSTATUS_FIXTURE={"v":1}'), completed(),
                       completed(stdout="PASS: no fixture"), completed(stdout='STATUS_FIXTURE={"v":1}'),
                       completed(stdout="PASS: bad\nSTATUS_FIXTURE={"),
                       completed(stdout="PASS: empty\nSTATUS_FIXTURE={}"),
                       completed(stdout='PASS: duplicate\nSTATUS_FIXTURE={"v":1}\nSTATUS_FIXTURE={"v":1}')):
            with self.subTest(result=result), self.assertRaises(RuntimeError):
                validate.check_host_result(result)

    def test_executed_host_results_are_counted(self):
        self.assertEqual(validate.check_host_result(completed(stdout=
            'PASS: one\nPASS: two\nSTATUS_FIXTURE={"v":1}\n')), 2)

    def test_wrong_dependency_pin_fails_before_uv_execution(self):
        with patch.object(validate.metadata, "version", return_value="0.0"), \
             patch.object(validate.subprocess, "run", side_effect=AssertionError("Unexpected uv")):
            with self.assertRaisesRegex(RuntimeError, "Expected esphome"):
                validate.check_environment(ROOT)

    def test_unsynchronized_environment_cannot_pass_or_repair(self):
        versions = {"esphome": "2026.9.1", "pyyaml": "6.0.3"}
        commands = []
        def run(command, **kwargs):
            commands.append(command)
            if "sync" in command:
                self.assertIn("--check", command)
                self.assertIn("--locked", command)
                self.assertIn("--offline", command)
                self.assertEqual(kwargs["env"]["UV_PROJECT_ENVIRONMENT"], sys.prefix)
                self.assertNotIn("UV_NO_DEV", kwargs["env"])
                raise subprocess.CalledProcessError(1, command)
            return completed(stdout="test compiler version\n")
        with patch.object(validate.metadata, "version", side_effect=lambda name: versions[name]), \
             patch.dict(os.environ, {"UV_NO_DEV": "1", "UV_PROJECT_ENVIRONMENT": "/wrong"}), \
             patch.object(validate.subprocess, "run", side_effect=run):
            with self.assertRaises(subprocess.CalledProcessError):
                validate.check_environment(ROOT)
        self.assertTrue(any("sync" in command for command in commands))
        self.assertFalse(any("pip" in command for command in commands))

    def test_real_git_errors_are_not_exports(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with patch.object(validate.shutil, "which", return_value="/git"), \
                 patch.object(validate.subprocess, "run", return_value=completed(128, stderr="dubious ownership")):
                with self.assertRaisesRegex(RuntimeError, "Cannot inspect Git"):
                    validate.is_git_checkout(root)
            with patch.object(validate.shutil, "which", return_value="/git"), \
                 patch.object(validate.subprocess, "run", return_value=completed(128, stderr="not a git repository")):
                self.assertFalse(validate.is_git_checkout(root))

    def test_git_marker_requires_git_and_correct_root(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / ".git").write_text("gitdir: missing")
            with patch.object(validate.shutil, "which", return_value=None):
                with self.assertRaisesRegex(RuntimeError, "Git is required"):
                    validate.is_git_checkout(root)
            with patch.object(validate.shutil, "which", return_value="/git"), \
                 patch.object(validate.subprocess, "run", return_value=completed(stdout=str(root.parent))):
                with self.assertRaisesRegex(RuntimeError, "root differs"):
                    validate.is_git_checkout(root)

    def test_git_checks_strip_source_overrides(self):
        calls = []
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            def run(command, **kwargs):
                calls.append(command)
                self.assertFalse(any(key.startswith("GIT_") for key in kwargs["env"]))
                return completed(stdout=str(root) if command[-1] == "--show-toplevel" else "abc")
            with patch.dict(os.environ, {"GIT_DIR": "/wrong", "GIT_INDEX_FILE": "/wrong"}), \
                 patch.object(validate.shutil, "which", return_value="/git"), \
                 patch.object(validate.subprocess, "run", side_effect=run):
                validate.describe_source(root)
            self.assertEqual(len(calls), 3)

    def test_whitespace_failure_propagates(self):
        with patch.object(validate, "check_documents", return_value=(1, 1)), \
             patch.object(validate, "is_git_checkout", return_value=True), \
             patch.object(validate.subprocess, "run", side_effect=subprocess.CalledProcessError(2, "git")):
            with self.assertRaises(subprocess.CalledProcessError):
                validate.document_checks(ROOT)

    def test_private_caches_and_generated_docs_are_excluded(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "README.md").write_text("[valid](public.txt)\n")
            (root / "public.txt").touch()
            for name in (".private", ".esphome", ".pio", ".venv", ".venv-other", "reports", "validation"):
                (root / name).mkdir()
                (root / name / "README.md").write_text("[broken](absent.txt)")
            self.assertEqual(documents.check_documents(root, ("README.md",)), (1, 1))
            (root / "README.md").write_text("[private](.private/README.md)")
            with self.assertRaisesRegex(RuntimeError, "excluded local link"):
                documents.check_documents(root, ("README.md",))

    def test_symlink_and_outside_links_fail(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "public.txt").touch()
            (root / "linked.txt").symlink_to(root / "public.txt")
            for link in ("linked.txt", "../outside.txt", "absent.txt"):
                (root / "README.md").write_text(f"[bad]({link})")
                with self.subTest(link=link), self.assertRaises(RuntimeError):
                    documents.check_documents(root, ("README.md",))

    def test_markdown_fences_code_balanced_and_reference_links(self):
        source = '''[balanced](a(b).md) [space](<a b.md>) [reference][id]
[id]: reference.md "Title"
`[ignored](missing.md)`
~~~~python
[ignored](missing.md)
~~~~~
![image](image.png)
'''
        self.assertEqual(documents.markdown_links(source), ["a(b).md", "a b.md", "reference.md", "image.png"])

    def test_pointer_diagnostics_require_intended_failure(self):
        gcc = "checks.cpp:12: error: request for member 'getState' in 'n', which is of pointer type 'Number*'"
        clang = "checks.cpp:12: error: member reference type 'Number *' is a pointer; did you mean to use '->'?"
        for diagnostic in (gcc, clang):
            self.assertTrue(id_check.pointer_access_failure(completed(1, stderr=diagnostic)))
            self.assertFalse(id_check.pointer_access_failure(completed(0, stderr=diagnostic)))
        for diagnostic in ("g++: command not found", "error: missing header", "which is of pointer type"):
            self.assertFalse(id_check.pointer_access_failure(completed(1, stderr=diagnostic)))

    def test_host_copy_does_not_copy_private_or_generated_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            root, target = Path(temporary) / "source", Path(temporary) / "copy"
            files = {"heating_optolink.yaml", "tests/host_safety_checks.py", "tests/host_stubs.py",
                     "components/vitoconnect/public.h", "components/vitoconnect/number/public.cpp"}
            private = {"secrets.yaml", ".env", ".venv/private.py", ".private/site.json",
                       ".esphome/build/private.h", "components/.DS_Store", "README.md",
                       "components/vitoconnect/.private/secret.h", "components/vitoconnect/.env.h"}
            for name in files | private:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("public or sentinel")
            id_check.copy_host_inputs(root, target)
            self.assertEqual({path.relative_to(target).as_posix() for path in target.rglob("*") if path.is_file()}, files)
            (root / "components/vitoconnect/public.h").unlink()
            (root / "components/vitoconnect/public.h").symlink_to(root / "secrets.yaml")
            with self.assertRaisesRegex(RuntimeError, "linked host input"):
                id_check.copy_host_inputs(root, target)

    def test_firmware_configurations_use_synthetic_secrets_and_local_source(self):
        for name in firmware_checks.CONFIGURATIONS:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temporary:
                target = Path(temporary)
                path = firmware_checks.prepare_configuration(ROOT, target, name)
                content = path.read_text()
                self.assertIn("source: components", content)
                self.assertNotIn("github://", content)
                self.assertIn("synthetic", (target / "secrets.yaml").read_text() if name in {"site-p300", "esp32-gwg"}
                              else "synthetic")
                self.assertFalse((target / ".venv").exists())
                self.assertFalse((target / ".git").exists())
                self.assertFalse((target / ".private").exists())

    def test_unknown_firmware_secret_and_symlink_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root, target = Path(temporary) / "source", Path(temporary) / "target"
            root.mkdir(); target.mkdir()
            source = root / "example-esp32.yaml"
            source.write_text("external_components:\n  - source: components\nwifi:\n  ssid: !secret private_unknown\n")
            with self.assertRaisesRegex(RuntimeError, "Unknown secret"):
                firmware_checks.prepare_configuration(root, target, "esp32-p300")
            source.unlink(); source.symlink_to(ROOT / "example-esp32.yaml")
            with self.assertRaisesRegex(RuntimeError, "linked firmware input"):
                firmware_checks.prepare_configuration(root, target, "esp32-p300")

    def test_target_package_metadata_uses_supported_read_only_query(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "platformio.ini").write_text("[env:test]\n")
            with patch.object(firmware_checks.subprocess, "run", return_value=completed(stdout="framework @ 3.0\n")) as run:
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    firmware_checks.target_package_metadata(root)
                self.assertIn("framework @ 3.0", output.getvalue())
                self.assertEqual(run.call_args.args[0][2:5], ["platformio", "pkg", "list"])
                self.assertIn(str(root), run.call_args.args[0])
            with patch.object(firmware_checks.subprocess, "run", return_value=completed()):
                with self.assertRaisesRegex(RuntimeError, "metadata was empty"):
                    firmware_checks.target_package_metadata(root)

    def test_successful_build_preserves_real_tool_output_before_cleanup(self):
        def run(command, **kwargs):
            directory = Path(command[-1]).parent
            (directory / "firmware.bin").write_bytes(b"synthetic binary")
            return completed(stdout="compiler version: test-tool-version\n", stderr="framework version: test-framework\n")
        output = io.StringIO()
        with patch.object(firmware_checks.subprocess, "run", side_effect=run), contextlib.redirect_stdout(output):
            firmware_checks.run_firmware(ROOT, "esp32-p300", "build")
        self.assertIn("compiler version: test-tool-version", output.getvalue())
        self.assertIn("framework version: test-framework", output.getvalue())
        self.assertIn("Artifact firmware.bin: SHA256", output.getvalue())

    def test_failed_firmware_command_cannot_pass(self):
        with patch.object(firmware_checks.subprocess, "run", return_value=completed(1, stderr="schema error")):
            with self.assertRaisesRegex(RuntimeError, "schema failed"):
                firmware_checks.run_firmware(ROOT, "esp32-p300", "schema")


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ValidationChecks)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful() or result.skipped or not result.testsRun:
        raise SystemExit(1)
    print(f"PASS: {result.testsRun} validation-tool regressions; none skipped", flush=True)
