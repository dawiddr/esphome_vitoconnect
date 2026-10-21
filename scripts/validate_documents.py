"""Check current guides and local Markdown file links without network access."""
from __future__ import annotations

import os
from pathlib import Path
import re
import sys
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
REQUIRED = (
    "AGENTS.md", "README.md", "tests/README.md", "docs/ACTUATOR.md", "docs/RELEASING.md",
)
EXCLUDED = {
    ".git", ".venv", ".venv-ha", "venv", "env", ".tox", ".nox", ".private",
    "__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache", "htmlcov",
    "dist", "build", "node_modules", "site-packages", ".esphome", ".pio",
    "validation", "reports", ".cache",
}


def without_fences(source: str) -> str:
    """Remove backtick and tilde fenced blocks, including longer closing fences."""
    output = []
    fence = None
    for line in source.splitlines():
        match = re.match(r"^ {0,3}(`{3,}|~{3,})(.*)$", line)
        if fence:
            if match and match[1][0] == fence[0] and len(match[1]) >= len(fence) and not match[2].strip():
                fence = None
        elif match and (match[1][0] != "`" or "`" not in match[2]):
            fence = match[1]
        else:
            output.append(line)
    return "\n".join(output)


def destination(source: str) -> str | None:
    """Read an angle-enclosed or balanced bare Markdown link destination."""
    source = source.lstrip()
    if source.startswith("<"):
        end = source.find(">")
        return source[1:end] if end >= 0 else None
    end = 0
    depth = 0
    while end < len(source):
        char = source[end]
        if char == "\\" and end + 1 < len(source):
            end += 2
            continue
        if char.isspace() or (char == ")" and depth == 0):
            break
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
        end += 1
    return re.sub(r"\\([\\()])", r"\1", source[:end]) or None


def code_span(source: str, start: int) -> tuple[int, bool]:
    """Return the end of a backtick span, or its unmatched opening run."""
    ticks = re.match(r"`+", source[start:])[0]
    opening_end = start + len(ticks)
    closing = re.search(r"(?<!`)" + ticks + r"(?!`)", source[opening_end:])
    if closing is None:
        return opening_end, False
    return opening_end + closing.end(), True


def closing_bracket(source: str, start: int, opening: str = "[", closing: str = "]") -> int | None:
    """Find a balanced closing delimiter, respecting Markdown escapes."""
    depth = 1
    index = start + 1
    while index < len(source):
        if source[index] == "\\":
            index += 2
            continue
        if opening == "[" and source[index] == "`":
            index, _ = code_span(source, index)
            continue
        if source[index] == opening:
            depth += 1
        elif source[index] == closing:
            depth -= 1
            if depth == 0:
                return index
        index += 1
    return None


def markdown_links(source: str) -> list[str]:
    """Read inline/image and reference links outside fenced code and code spans."""
    source = without_fences(source)
    code_ranges = []
    index = 0
    while index < len(source):
        if source[index] == "\\":
            index += 2
        elif source[index] == "`":
            end, matched = code_span(source, index)
            if matched:
                code_ranges.append((index, end))
            index = end
        else:
            index += 1

    def normalize(value: str) -> str:
        return " ".join(value.split()).casefold()

    references = {}
    lines = []
    pending = source.splitlines(keepends=True)
    offsets = []
    offset = 0
    for line in pending:
        offsets.append(offset)
        offset += len(line)
    index = 0
    while index < len(pending):
        line = pending[index]
        match = re.match(r"^ {0,3}\[([^\]]+)\]:[ \t]*(.*)$", line)
        if match and any(start <= offsets[index] + line.index("[") < end
                         for start, end in code_ranges):
            match = None
        if match:
            value = match[2]
            if not value and index + 1 < len(pending):
                index += 1
                value = pending[index].lstrip()
            target = destination(value)
            if target is not None:
                references.setdefault(normalize(match[1]), target)
        else:
            lines.append(line)
        index += 1

    def read_links(text: str) -> list[str]:
        links = []
        index = 0
        while index < len(text):
            if text[index] == "\\":
                index += 2
                continue
            if text[index] == "`":
                index, _ = code_span(text, index)
                continue
            if text[index] != "[":
                index += 1
                continue
            end = closing_bracket(text, index)
            if end is None:
                index += 1
                continue
            label = text[index + 1:end]
            links.extend(read_links(label))
            index = end + 1
            target = None
            if text[index:index + 1] == "(":
                target = destination(text[index + 1:])
                end = closing_bracket(text, index, "(", ")")
                if end is not None:
                    index = end + 1
            elif text[index:index + 1] == "[":
                end = closing_bracket(text, index)
                if end is not None:
                    target = references.get(normalize(text[index + 1:end] or label))
                    index = end + 1
            else:
                target = references.get(normalize(label))
            if target is not None:
                links.append(target)
        return links

    return read_links("".join(lines))


def excluded_path(relative: Path) -> bool:
    return bool(EXCLUDED.intersection(relative.parts) or any(
        part.startswith((".venv-", ".venv.")) for part in relative.parts
    ))


def has_symlink(root: Path, path: Path) -> bool:
    relative = path.relative_to(root)
    return any((root / part).is_symlink() for part in (relative, *relative.parents)
               if part != Path("."))


def check_documents(root: Path, required: tuple[str, ...] = REQUIRED) -> tuple[int, int]:
    root = root.resolve()
    for name in required:
        path = root / name
        if not path.is_file() or has_symlink(root, path):
            raise RuntimeError(f"Missing document: {name}")
    documents = []
    for directory, dirs, files in os.walk(root, followlinks=False):
        dirs[:] = [name for name in dirs if not excluded_path((Path(directory) / name).relative_to(root))
                   and not (Path(directory) / name).is_symlink()]
        documents.extend(Path(directory) / name for name in files
                         if name.lower().endswith(".md") and not (Path(directory) / name).is_symlink())
    count = 0
    for document in sorted(documents):
        for link in markdown_links(document.read_text(encoding="utf-8")):
            parsed = urlsplit(link)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            path = document.parent / unquote(parsed.path)
            target = path.resolve()
            if not target.is_relative_to(root):
                raise RuntimeError(f"Link outside project: {document.relative_to(root)}: {link}")
            relative = target.relative_to(root)
            if (not target.is_file() or excluded_path(relative)
                    or has_symlink(root, path)):
                raise RuntimeError(f"Broken or excluded local link: {document.relative_to(root)}: {link}")
            count += 1
    return len(documents), count


def main() -> None:
    documents, links = check_documents(ROOT)
    print(f"PASS: {len(REQUIRED)} required project documents; {documents} Markdown files checked")
    print(f"PASS: {links} local Markdown file links; anchors and remote URLs are not checked")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"FAIL: document validation: {exc}", file=sys.stderr)
        sys.exit(1)
