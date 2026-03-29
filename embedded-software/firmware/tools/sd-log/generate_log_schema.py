#!/usr/bin/env python3
"""
Generate a Python representation of the binary log schema from the shared C header.
"""

from __future__ import annotations

import argparse
import re
import struct
from pathlib import Path
from textwrap import dedent

ROOT = Path(__file__).resolve().parents[2]
HEADER_PATH = ROOT / "libs" / "log_records" / "include" / "log_records" / "log_records.h"
OUTPUT_PATH = Path(__file__).resolve().parent / "log_schema.py"

TYPE_FORMATS = {
    "bool": "B",
    "uint8_t": "B",
    "int8_t": "b",
    "uint16_t": "H",
    "int16_t": "h",
    "uint32_t": "I",
    "int32_t": "i",
    "uint64_t": "Q",
    "int64_t": "q",
    "float": "f",
    "double": "d",
}


def extract_macro_block(text: str, macro_name: str) -> str:
    pattern = re.compile(
        rf"#define\s+{macro_name}\(APP\)\s+\\\s*((?:\s*APP\([^)]*\)\s*\\?\s*)+)",
        re.MULTILINE,
    )
    match = pattern.search(text)
    if not match:
        raise SystemExit(f"Unable to find macro {macro_name}")
    return match.group(1)


def extract_field_macro(text: str, macro_name: str) -> str:
    pattern = re.compile(
        rf"#define\s+{macro_name}\(FIELD\)\s+\\\s*((?:\s*FIELD\([^)]*\)\s*\\?\s*)+)",
        re.MULTILINE,
    )
    match = pattern.search(text)
    if not match:
        raise SystemExit(f"Unable to find macro {macro_name}")
    return match.group(1)


def parse_records(text: str):
    body = extract_macro_block(text, "LOG_RECORD_LIST")
    entries = re.findall(r"APP\(([^,]+),\s*([^,]+),\s*([^,]+),\s*([^)]+)\)", body)
    records = []
    for raw_id, raw_name, raw_fields, _enable in entries:
        record_id = int(raw_id.strip(), 0)
        name = raw_name.strip()
        fields_macro = raw_fields.strip()
        records.append((record_id, name, fields_macro))
    return records


def parse_fields(text: str, macro_name: str):
    body = extract_field_macro(text, macro_name)
    fields = re.findall(r"FIELD\(([^,]+),\s*([^)]+)\)", body)
    results = []
    for c_type, field_name in fields:
        c_type = c_type.strip()
        field_name = field_name.strip()
        if c_type not in TYPE_FORMATS:
            raise SystemExit(f"Unsupported C type '{c_type}' in {macro_name}")
        results.append((c_type, field_name))
    return results


def generate_python(records, schema_version):
    header = dedent(
        """\
        \"\"\"Auto-generated from libs/log_records/include/log_records/log_records.h
        Do not edit manually. Run tools/sd-log/generate_log_schema.py instead.
        \"\"\"

        from __future__ import annotations

        import struct

        LOG_SCHEMA_VERSION = {schema_version}

        TYPE_FORMATS = {type_formats}

        RECORDS = {{
        """
    ).format(schema_version=schema_version, type_formats=TYPE_FORMATS)

    lines = [header]
    for record in records:
        name = record["name"]
        fmt = record["struct_format"]
        lines.append(f'    "{name}": {{')
        lines.append(f"        \"id\": {record['id']},")
        lines.append(f"        \"enum\": \"LOG_RECORD_TYPE_{name}\",")
        lines.append("        \"fields\": [")
        for c_type, field_name in record["fields"]:
            lines.append(f"            (\"{c_type}\", \"{field_name}\"),")
        lines.append("        ],")
        lines.append(f"        \"format\": \"{fmt}\",")
        lines.append("        \"struct\": struct.Struct(\"" + fmt + "\"),")
        lines.append("    },")
    lines.append("}\n")
    lines.append(
        "MAX_RECORD_SIZE = max(rec['struct'].size for rec in RECORDS.values())\n"
    )
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Generate log schema Python module.")
    parser.add_argument(
        "--header",
        default=HEADER_PATH,
        type=Path,
        help="Path to log_records.h (default: %(default)s)",
    )
    parser.add_argument(
        "--output",
        default=OUTPUT_PATH,
        type=Path,
        help="Destination Python file (default: %(default)s)",
    )
    args = parser.parse_args()

    text = args.header.read_text()
    schema_version_match = re.search(
        r"#define\s+LOG_SCHEMA_VERSION\s+(\d+)", text
    )
    if not schema_version_match:
        raise SystemExit("LOG_SCHEMA_VERSION not found.")
    schema_version = int(schema_version_match.group(1))

    parsed_records = parse_records(text)
    records = []
    for record_id, name, fields_macro in parsed_records:
        fields = parse_fields(text, fields_macro)
        fmt = "<" + "".join(TYPE_FORMATS[c_type] for c_type, _ in fields)
        struct.calcsize(fmt)  # validate
        records.append(
            {
                "id": record_id,
                "name": name,
                "fields": fields,
                "struct_format": fmt,
            }
        )

    args.output.write_text(generate_python(records, schema_version))
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
