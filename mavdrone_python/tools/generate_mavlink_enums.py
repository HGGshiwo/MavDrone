from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from textwrap import fill
from urllib.request import urlopen
import xml.etree.ElementTree as ET

MESSAGE_DEFINITIONS_URLS = [
    "https://raw.githubusercontent.com/mavlink/mavlink/refs/heads/master/message_definitions/v1.0/minimal.xml",
    "https://raw.githubusercontent.com/mavlink/mavlink/refs/heads/master/message_definitions/v1.0/standard.xml",
    "https://raw.githubusercontent.com/mavlink/mavlink/refs/heads/master/message_definitions/v1.0/common.xml",
]


@dataclass
class EnumEntry:
    value: int
    name: str
    description: str | None


@dataclass
class EnumTemplate:
    name: str
    description: str | None
    bitmask: bool
    entries: list[EnumEntry]


def download_content(url: str) -> str:
    with urlopen(url) as response:
        return response.read().decode("utf-8")


def filter_description(description: str | None) -> str | None:
    if description is None:
        return None
    return " ".join(description.replace("\n", "").split())


def parse_message_file(content: str) -> list[EnumTemplate]:
    root = ET.fromstring(content)

    enums = []
    for xml_enum in root.findall(".//enum"):
        entries = []
        for xml_entry in xml_enum.findall(".//entry"):
            description = filter_description(xml_entry.findtext("description"))
            entry = EnumEntry(int(xml_entry.get("value")), xml_entry.get("name"), description)
            entries.append(entry)

        description = filter_description(xml_enum.findtext("description"))
        bitmask = xml_enum.get("bitmask", "false").lower() == "true"

        enum = EnumTemplate(xml_enum.get("name"), description, bitmask, entries)
        enums.append(enum)

    return enums


def generate_python_content(enums: list[EnumTemplate]) -> str:
    header = f"""# Auto-generated file, do not edit.
# Generated at {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}.
from __future__ import annotations

from enum import IntEnum, IntFlag


"""

    file_content = []
    for enum in enums:
        class_header = f"class {enum.name}({'IntFlag' if enum.bitmask else 'IntEnum'}):\n"
        class_description = (
            "" if enum.description is None else f'    """{fill(enum.description, width=100, subsequent_indent="    ")}"""\n\n'
        )
        class_entries = ""
        for entry in enum.entries:
            class_entries += f"    {entry.name} = {entry.value}\n"
            class_entries += (
                "" if entry.description is None else f'    """{fill(entry.description, width=100, subsequent_indent="    ")}"""\n'
            )
            # class_entries += "\n" if entry.description is None else f"  # {entry.description}\n"

        file_content.append(class_header + class_description + class_entries)

    return header + "\n\n".join(file_content)


def main() -> None:
    enums = []
    for url in MESSAGE_DEFINITIONS_URLS:
        message_definitions = download_content(url)
        enums.extend(parse_message_file(message_definitions))

    enum_file = Path(__file__).parents[1] / "mavdrone" / "enums.py"
    enum_file.write_text(generate_python_content(enums))


if __name__ == "__main__":
    main()
