"""Generate Autos.md entries for each GIF in the Gifs folder."""

from __future__ import annotations

from pathlib import Path
import re
import shutil
from urllib.parse import quote
from typing import Iterable, Tuple

from PIL import Image, ImageSequence


BASE_DIR = Path(__file__).resolve().parent
GIFS_DIR = BASE_DIR / "Gifs"
AUTOS_MD = BASE_DIR / "Autos.md"


def gif_duration_seconds(gif_path: Path) -> float:
	"""Return total duration of a GIF in seconds."""
	with Image.open(gif_path) as gif:
		total_ms = sum(frame.info.get("duration", 0) for frame in ImageSequence.Iterator(gif))
	return total_ms / 1000.0


def normalize_gif_name(gif_path: Path) -> Tuple[str, Path]:
	"""Return display title and a gif path to use in markdown.

	If a gif name starts with a number, make a copy without the number and
	use that copy for the image link, while keeping the header with the number.
	"""
	title = gif_path.stem
	match = re.match(r"^(\d+)[\s._-]+(.+)$", title)
	if not match:
		return title, gif_path

	number, rest = match.groups()
	cleaned_name = f"{rest}{gif_path.suffix}"
	target_path = gif_path.with_name(cleaned_name)
	if not target_path.exists():
		shutil.copy2(gif_path, target_path)
	return f"{number}. {rest}", target_path


def format_entry(gif_path: Path) -> str:
	title, image_path = normalize_gif_name(gif_path)
	duration = gif_duration_seconds(gif_path)
	image_name = quote(image_path.name)
	return "\n".join(
		[
			f"## {title}",
			f"Drive path: ~{duration:.1f} seconds",
			f"![](Gifs/{image_name})",
			"",
		]
	)


def load_delaying_section(existing_md: str) -> str:
	marker = "## Delaying"
	if marker in existing_md:
		return existing_md[existing_md.index(marker) :].rstrip() + "\n"
	return (
		"## Delaying\n\n"
		"Each one can be delayed (measured in seconds by editing ) "
		'"Smartdashboard/Auto Delay (Seconds)"\n'
	)


def generate_markdown(gif_paths: Iterable[Path], delaying_section: str) -> str:
	entries = [format_entry(path) for path in sorted(gif_paths, key=lambda p: p.name.lower())]
	return "\n".join(entries).rstrip() + "\n\n" + delaying_section


def main() -> None:
	if not GIFS_DIR.exists():
		raise SystemExit(f"Gifs folder not found: {GIFS_DIR}")

	gif_paths = [path for path in GIFS_DIR.iterdir() if path.suffix.lower() == ".gif"]
	numbered_stems = set()
	for path in gif_paths:
		match = re.match(r"^(\d+)[\s._-]+(.+)$", path.stem)
		if match:
			_, rest = match.groups()
			numbered_stems.add(rest)

	gifs = [
		path
		for path in gif_paths
		if re.match(r"^(\d+)[\s._-]+(.+)$", path.stem) or path.stem not in numbered_stems
	]
	existing_md = AUTOS_MD.read_text(encoding="utf-8") if AUTOS_MD.exists() else ""
	delaying_section = load_delaying_section(existing_md)
	autos_md = generate_markdown(gifs, delaying_section)
	AUTOS_MD.write_text(autos_md, encoding="utf-8")
	print(f"Wrote {AUTOS_MD}")


if __name__ == "__main__":
	main()
