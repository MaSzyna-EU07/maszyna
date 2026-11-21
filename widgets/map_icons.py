#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from editor.icons import editor_icons


CELL = editor_icons.CELL
SUPER = editor_icons.SUPER
ICONS = (
    editor_icons.translate,
    editor_icons.rotate,
    editor_icons.vehicle,
    editor_icons.camera,
    editor_icons.zoom_in,
    editor_icons.zoom_out,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--png", required=True, type=Path)
    parser.add_argument("--include", required=True, type=Path)
    args = parser.parse_args()

    atlas = Image.new("RGBA", (8 * CELL, CELL), (255, 255, 255, 0))
    for index, draw_icon in enumerate(ICONS):
        mask = Image.new("L", (CELL * SUPER, CELL * SUPER), 0)
        draw_icon(ImageDraw.Draw(mask))
        holes = Image.new("L", mask.size, 0)
        editor_icons.cutouts(draw_icon.__name__, ImageDraw.Draw(holes))
        mask.paste(0, (0, 0), holes)
        mask = mask.resize((CELL, CELL), Image.LANCZOS)
        icon_image = Image.new("RGBA", mask.size, (255, 255, 255, 0))
        icon_image.putalpha(mask)
        atlas.paste(icon_image, (index * CELL, 0))

    args.png.parent.mkdir(parents=True, exist_ok=True)
    args.include.parent.mkdir(parents=True, exist_ok=True)
    atlas.save(args.png, optimize=True)
    args.include.write_text(
        "".join(f"0x{byte:02x}," for byte in args.png.read_bytes()) + "\n",
        encoding="ascii",
    )


if __name__ == "__main__":
    main()
