from pathlib import Path
import sys

from PIL import Image


def largest_component(points):
    points = set(points)
    best = []
    while points:
        start = points.pop()
        stack = [start]
        comp = [start]
        while stack:
            x, y = stack.pop()
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                p = (nx, ny)
                if p in points:
                    points.remove(p)
                    stack.append(p)
                    comp.append(p)
        if len(comp) > len(best):
            best = comp
    return best


def main():
    path = Path(sys.argv[1])
    img = Image.open(path).convert("RGB")
    width, height = img.size

    dark = []
    red = []
    for y in range(height):
        for x in range(width):
            r, g, b = img.getpixel((x, y))
            if x > 80 and r < 70 and g < 75 and b < 80:
                dark.append((x, y))
            if x > 80 and r > 120 and r > g + 25 and r > b + 15:
                red.append((x, y))

    frame = largest_component(dark)
    if not frame:
        print("frame=0")
        return 2
    fx = [p[0] for p in frame]
    fy = [p[1] for p in frame]
    fcx = (min(fx) + max(fx)) / 2.0
    fcy = (min(fy) + max(fy)) / 2.0

    laser = largest_component(red)
    if not laser:
        print(
            "frame=1,fcx=%.1f,fcy=%.1f,fw=%d,fh=%d,laser=0"
            % (fcx, fcy, max(fx) - min(fx) + 1, max(fy) - min(fy) + 1)
        )
        return 3
    lx = [p[0] for p in laser]
    ly = [p[1] for p in laser]
    lcx = sum(lx) / len(lx)
    lcy = sum(ly) / len(ly)
    err_x = lcx - fcx
    err_y = lcy - fcy
    print(
        "frame=1,fcx=%.1f,fcy=%.1f,fw=%d,fh=%d,laser=1,lcx=%.1f,lcy=%.1f,lw=%d,lh=%d,err_x=%.1f,err_y=%.1f,err2=%.1f"
        % (
            fcx,
            fcy,
            max(fx) - min(fx) + 1,
            max(fy) - min(fy) + 1,
            lcx,
            lcy,
            max(lx) - min(lx) + 1,
            max(ly) - min(ly) + 1,
            err_x,
            err_y,
            err_x * err_x + err_y * err_y,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
