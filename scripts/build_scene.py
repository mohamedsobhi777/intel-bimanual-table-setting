"""Build the portable MJCF with a hollow, segmented cup; robot assets stay pristine."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def build():
    source = ROOT / "scenes/dinner_table.xml"
    tree = ET.parse(source)
    cup = tree.find(".//body[@name='cup']")
    for i in range(16):
        angle = 2 * math.pi * i / 16
        ET.SubElement(cup, "geom", {
            "name": f"cup_wall_{i}", "type": "box",
            "pos": f"{0.024 * math.cos(angle):.8f} {0.024 * math.sin(angle):.8f} 0.034",
            "euler": f"0 0 {angle:.8f}", "size": "0.002 0.005 0.03",
            "mass": "0.003", "rgba": "0.12 0.43 0.72 1",
        })
    for name, start, end in [
        ("top", "0.023 0 0.054", "0.043 0 0.054"),
        ("outer", "0.043 0 0.054", "0.043 0 0.014"),
        ("bottom", "0.043 0 0.014", "0.023 0 0.014"),
    ]:
        ET.SubElement(cup, "geom", {"name": f"cup_handle_{name}", "type": "capsule",
            "fromto": f"{start} {end}", "size": "0.004", "mass": "0.004",
            "rgba": "0.12 0.43 0.72 1"})
    target = ROOT / "scenes/scene.xml"
    ET.indent(tree, space="  ")
    tree.write(target, encoding="unicode")
    return target


if __name__ == "__main__":
    print(build())
