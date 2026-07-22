#!/usr/bin/env python3
"""Convert FLIGHT_RECORDER SD-card CSVs into the /uav/... schema used by
plotjuggler_mfc.xml (same BRANCH_MAP as pj_json_relay.py's normalize_obj)."""
import csv
import sys

BRANCH_MAP = {
    "STAB_MFC": "MFC_STAB",
    "GUIDANCE_MFC": "MFC_GUIDANCE",
    "GUIDANCE_MFC_ACC2ATT": "MFC_ACC2ATT",
    "ACC2ATT": "MFC_ACC2ATT",
}


def convert_header(col):
    if ":" not in col:
        return col
    branch, field = col.split(":", 1)
    branch = BRANCH_MAP.get(branch, branch)
    return f"/uav/{branch}/{field}"


def convert(in_path, out_path):
    with open(in_path, newline="") as f_in:
        reader = csv.reader(f_in, delimiter="\t")
        header = next(reader)
        new_header = [convert_header(c) for c in header]
        with open(out_path, "w", newline="") as f_out:
            writer = csv.writer(f_out)
            writer.writerow(new_header)
            for row in reader:
                writer.writerow(row)


if __name__ == "__main__":
    for in_path, out_path in zip(sys.argv[1::2], sys.argv[2::2]):
        convert(in_path, out_path)
        print(f"wrote {out_path}")