
"""Configure Gazebo system plugins in an SDF model."""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


JOINT_STATE_PLUGIN = {
    "filename": "gz-sim-joint-state-publisher-system",
    "name": "gz::sim::systems::JointStatePublisher",
}


def add_joint_state_publisher(
    sdf_path: str | Path,
    output_path: str | Path,
    topic: str | None = None,
) -> Path:
    """Add a Gazebo joint-state publisher to a converted SDF model."""
    source = Path(sdf_path)
    destination = Path(output_path)

    tree = ET.parse(source)
    root = tree.getroot()

    models = root.findall(".//model")

    if root.tag == "sdf":
        models = root.findall("model") + models

    # Avoid counting the same element twice.
    models = list(dict.fromkeys(models))

    if len(models) != 1:
        raise ValueError(
            "Expected exactly one SDF model, "
            f"found {len(models)}."
        )

    model = models[0]

    joints = model.findall("joint")
    if not joints:
        raise ValueError("Model contains no joints.")

    for plugin in model.findall("plugin"):
        if plugin.get("name") == JOINT_STATE_PLUGIN["name"]:
            raise ValueError(
                "JointStatePublisher plugin already exists."
            )

    plugin = ET.SubElement(
        model,
        "plugin",
        JOINT_STATE_PLUGIN,
    )

    if topic is not None:
        if not topic.strip():
            raise ValueError("Topic must not be empty.")

        ET.SubElement(plugin, "topic").text = topic

    destination.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    tree.write(
        destination,
        encoding="utf-8",
        xml_declaration=True,
    )

    return destination


def main() -> None:
    """Generate a model SDF with joint-state feedback."""
    parser = argparse.ArgumentParser(
        description="Configure Gazebo joint-state feedback."
    )

    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--topic", default=None)

    args = parser.parse_args()

    output = add_joint_state_publisher(
        sdf_path=args.input,
        output_path=args.output,
        topic=args.topic,
    )

    print(f"Generated: {output}")


if __name__ == "__main__":
    main()
