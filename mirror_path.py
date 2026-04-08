import os
import json
import copy

FIELD_HEIGHT = 8.052  # meters, FRC 2025 Reefscape

paths_to_flip = [
    "Lehigh-Sweep-Part1.path",
    "Leihigh-Sweep-Part2.path",
    "Lehigh-Doulbe-Sweep-Part3.path",
    "Lehigh-Double-Sweep-Part4.path"
]

autos_to_flip = [
    "12. Depot Double Hairpin.auto"
]

PATHS_DIR = os.path.join("src", "main", "deploy", "pathplanner", "paths")
AUTOS_DIR = os.path.join("src", "main", "deploy", "pathplanner", "autos")


def mirror_point(point):
    """Mirror a {x, y} point across the horizontal centerline."""
    if point is None:
        return None
    return {"x": point["x"], "y": FIELD_HEIGHT - point["y"]}


def mirror_rotation(angle_deg):
    """Mirror a rotation angle across the horizontal centerline."""
    if angle_deg is None:
        return None
    return -angle_deg


def mirror_path(data):
    mirrored = copy.deepcopy(data)

    # Mirror waypoints
    for wp in mirrored.get("waypoints", []):
        wp["anchor"] = mirror_point(wp["anchor"])
        wp["prevControl"] = mirror_point(wp["prevControl"])
        wp["nextControl"] = mirror_point(wp["nextControl"])
        wp["linkedName"] = None  # linked poses won't exist on mirrored side

    # Mirror rotation targets
    for rt in mirrored.get("rotationTargets", []):
        rt["rotationDegrees"] = mirror_rotation(rt.get("rotationDegrees"))

    # Mirror goal end state rotation
    if "goalEndState" in mirrored and mirrored["goalEndState"] is not None:
        mirrored["goalEndState"]["rotation"] = mirror_rotation(
            mirrored["goalEndState"].get("rotation")
        )

    # Mirror ideal starting state rotation
    if "idealStartingState" in mirrored and mirrored["idealStartingState"] is not None:
        mirrored["idealStartingState"]["rotation"] = mirror_rotation(
            mirrored["idealStartingState"].get("rotation")
        )

    return mirrored

def mirror_command(command):
    """Recursively mirror path names in a command tree."""
    if command is None:
        return
    cmd_type = command.get("type")
    cmd_data = command.get("data", {})
    if cmd_type == "path":
        cmd_data["pathName"] = cmd_data["pathName"] + "-Mirrored"
    # Recurse into nested commands
    for child in cmd_data.get("commands", []):
        mirror_command(child)


def mirror_auto(data):
    mirrored = copy.deepcopy(data)
    mirror_command(mirrored.get("command"))
    return mirrored

for filename in paths_to_flip:
    input_path = os.path.join(PATHS_DIR, filename)
    name, ext = os.path.splitext(filename)
    output_filename = name + "-Mirrored" + ext
    output_path = os.path.join(PATHS_DIR, output_filename)

    with open(input_path, "r") as f:
        data = json.load(f)

    mirrored = mirror_path(data)

    with open(output_path, "w") as f:
        json.dump(mirrored, f, indent=2)

    print(f"Mirrored: {filename} -> {output_filename}")

for filename in autos_to_flip:
    input_path = os.path.join(AUTOS_DIR, filename)
    name, ext = os.path.splitext(filename)
    output_filename = name + "-Mirrored" + ext
    output_path = os.path.join(AUTOS_DIR, output_filename)

    with open(input_path, "r") as f:
        data = json.load(f)

    mirrored = mirror_auto(data)

    with open(output_path, "w") as f:
        json.dump(mirrored, f, indent=2)

    print(f"Mirrored: {filename} -> {output_filename}")