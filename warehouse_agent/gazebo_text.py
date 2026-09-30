"""Small geometry font for Ogre2, which does not render Gazebo TEXT markers.

Each lit cell becomes two triangles. Labels face the demo camera and remain
purely visual: there is no sensor or collision geometry.
"""
import math
from functools import lru_cache

from .capture_scene import CAMERA_POSITION

_ROWS = {
    "A": "01110/10001/10001/11111/10001/10001/10001",
    "B": "11110/10001/10001/11110/10001/10001/11110",
    "C": "01111/10000/10000/10000/10000/10000/01111",
    "D": "11110/10001/10001/10001/10001/10001/11110",
    "E": "11111/10000/10000/11110/10000/10000/11111",
    "F": "11111/10000/10000/11110/10000/10000/10000",
    "G": "01111/10000/10000/10111/10001/10001/01111",
    "H": "10001/10001/10001/11111/10001/10001/10001",
    "I": "11111/00100/00100/00100/00100/00100/11111",
    "J": "00111/00010/00010/00010/10010/10010/01100",
    "K": "10001/10010/10100/11000/10100/10010/10001",
    "L": "10000/10000/10000/10000/10000/10000/11111",
    "M": "10001/11011/10101/10101/10001/10001/10001",
    "N": "10001/11001/11001/10101/10011/10011/10001",
    "O": "01110/10001/10001/10001/10001/10001/01110",
    "P": "11110/10001/10001/11110/10000/10000/10000",
    "Q": "01110/10001/10001/10001/10101/10010/01101",
    "R": "11110/10001/10001/11110/10100/10010/10001",
    "S": "01111/10000/10000/01110/00001/00001/11110",
    "T": "11111/00100/00100/00100/00100/00100/00100",
    "U": "10001/10001/10001/10001/10001/10001/01110",
    "V": "10001/10001/10001/10001/10001/01010/00100",
    "W": "10001/10001/10001/10101/10101/10101/01010",
    "X": "10001/10001/01010/00100/01010/10001/10001",
    "Y": "10001/10001/01010/00100/00100/00100/00100",
    "Z": "11111/00001/00010/00100/01000/10000/11111",
    "0": "01110/10001/10011/10101/11001/10001/01110",
    "1": "00100/01100/00100/00100/00100/00100/01110",
    "2": "01110/10001/00001/00010/00100/01000/11111",
    "3": "11110/00001/00001/01110/00001/00001/11110",
    "4": "00010/00110/01010/10010/11111/00010/00010",
    "5": "11111/10000/10000/11110/00001/00001/11110",
    "6": "01110/10000/10000/11110/10001/10001/01110",
    "7": "11111/00001/00010/00100/01000/01000/01000",
    "8": "01110/10001/10001/01110/10001/10001/01110",
    "9": "01110/10001/10001/01111/00001/00001/01110",
    ",": "00000/00000/00000/00000/00100/00100/01000",
    "·": "00000/00000/00000/00100/00000/00000/00000",
}


@lru_cache(maxsize=256)
def cells(text):
    result = []
    lines = text.upper().splitlines()
    for line_index, line in enumerate(lines):
        width = max(0, len(line)*6-1)
        for index, char in enumerate(line):
            for row, bits in enumerate(_ROWS.get(char, "").split("/")):
                for column, bit in enumerate(bits):
                    if bit == "1":
                        result.append((index*6+column-width/2, (len(lines)-1)*4-line_index*9+3-row))
    return tuple(result)


def triangles(text, position, height):
    # Same initial viewpoint as capture_scene.py; keep labels legible in the demo.
    dx, dy, dz = (a-b for a, b in zip(CAMERA_POSITION, position, strict=True))
    horizontal, length = math.hypot(dx, dy), math.sqrt(dx*dx+dy*dy+dz*dz)
    right = (-dy/horizontal, dx/horizontal, 0)
    up = (-dz*dx/(length*horizontal), -dz*dy/(length*horizontal), horizontal/length)
    unit = height/7
    points = []
    for x, y in cells(text):
        for sx, sy in ((0, 0), (1, 0), (1, 1), (0, 0), (1, 1), (0, 1)):
            points.append(tuple(unit*((x+sx)*right[i]+(y+sy)*up[i]) for i in range(3)))
    return points
