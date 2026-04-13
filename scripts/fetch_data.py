#!/usr/bin/env python3
"""Download the parts of the Camargo et al. (2021) dataset that this project needs.

The dataset is CC BY 4.0. Cite: J. Camargo, A. Ramanathan, W. Flanagan, A. Young,
"A comprehensive, open-source dataset of lower limb biomechanics in multiple
conditions of stairs, ramps, and level-ground ambulation and transitions",
Journal of Biomechanics 119 (2021) 110320, doi:10.1016/j.jbiomech.2021.110320.

Files come from the public Dropbox folder linked on the EPIC Lab page and are saved
under data/raw/camargo/ with the dataset's own layout:
<subject>/<date>/<mode>/<sensor>/<trial>.mat

Dropbox has no documented API for anonymous access to a shared folder, so the
listing uses the same request the Dropbox web page makes. If that stops working,
the script prints the manual steps instead (also available with --manual).

Standard library only. Examples:
    python scripts/fetch_data.py --dry-run
    python scripts/fetch_data.py --subjects AB06 AB07
    python scripts/fetch_data.py --subjects all --modes stair ramp
"""

from __future__ import annotations

import argparse
import http.cookiejar
import json
import re
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path

ROOT_URL = (
    "https://www.dropbox.com/scl/fo/i2ee8z2nxjwkf3mdocw63/AEuNRES_bD2n_Yt0xYzjHQ4"
    "?rlkey=eiqcgo6gyvf4bnfgduq419yyh&dl=0"
)
RLKEY = "eiqcgo6gyvf4bnfgduq419yyh"
LIST_ENDPOINT = "https://www.dropbox.com/list_shared_link_folder_entries"
USER_AGENT = (
    "Mozilla/5.0 (Macintosh; Intel Mac OS X 14_0) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/126.0 Safari/537.36"
)

ALL_MODES = ("treadmill", "levelground", "ramp", "stair")
DEFAULT_SENSORS = ("conditions", "ik", "id", "gcRight", "gcLeft")
TOP_LEVEL_FILES = ("README.txt", "SubjectInfo.mat")

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "data" / "raw" / "camargo"

MANUAL_STEPS = """\
Manual download (needs only a web browser, no login):

1. Open the EPIC Lab dataset page:
   https://www.epic.gatech.edu/opensource-biomechanics-camargo-et-al/
   and follow "(direct link)" to the public Dropbox folder.
2. Download README.txt and SubjectInfo.mat from the top level.
3. For each subject you want (AB06 to AB30, 22 subjects), open
   <subject>/<date>/ and, inside each of treadmill, levelground, ramp and stair,
   download the folders conditions, ik, id, gcRight and gcLeft.
   A whole subject folder is about 1 GB; these five folders are a small part of it.
4. Keep the dataset's layout and put everything under
   data/raw/camargo/, for example
   data/raw/camargo/AB06/10_09_18/stair/id/stair_1_l_01_01.mat

The same data are on Mendeley Data in three parts (CC BY 4.0):
   https://doi.org/10.17632/fcgm3chfff.2
   https://doi.org/10.17632/k9kvm5tn3f.2
   https://doi.org/10.17632/jj3r5f9pnf.2
"""


class DropboxFolder:
    """Anonymous reader for the public Dropbox folder of the dataset."""

    def __init__(self) -> None:
        self.jar = http.cookiejar.CookieJar()
        self.opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(self.jar))
        self.opener.addheaders = [("User-Agent", USER_AGENT)]
        self.opener.open(ROOT_URL, timeout=60).read()
        tokens = [c.value for c in self.jar if c.name == "t"]
        if not tokens:
            raise RuntimeError("Dropbox did not set the session cookie needed for listing")
        self.token = tokens[0]

    def list(self, href: str) -> list[dict]:
        match = re.search(r"/scl/fo/([^/]+)/([^/?]+)(/[^?]*)?\?", href)
        if match is None:
            raise RuntimeError(f"unexpected Dropbox link: {href}")
        link_key, secure_hash, sub_path = match.group(1), match.group(2), match.group(3) or ""
        form = {
            "is_xhr": "true",
            "t": self.token,
            "link_key": link_key,
            "link_type": "c",
            "secure_hash": secure_hash,
            "sub_path": urllib.parse.unquote(sub_path),
            "rlkey": RLKEY,
        }
        request = urllib.request.Request(
            LIST_ENDPOINT,
            data=urllib.parse.urlencode(form).encode(),
            headers={
                "Origin": "https://www.dropbox.com",
                "Referer": ROOT_URL,
                "Content-Type": "application/x-www-form-urlencoded; charset=UTF-8",
            },
        )
        for attempt in range(6):
            try:
                payload = json.loads(self.opener.open(request, timeout=60).read())
                return payload.get("entries", [])
            except (OSError, ValueError) as error:
                if attempt == 5:
                    raise
                wait = 15 * 2**attempt
                print(f"  listing failed, retrying in {wait} s ({error})", file=sys.stderr)
                time.sleep(wait)
        return []

    def download(self, href: str, target: Path, attempts: int = 6) -> None:
        url = href.replace("dl=0", "dl=1")
        target.parent.mkdir(parents=True, exist_ok=True)
        partial = target.with_suffix(target.suffix + ".part")
        for attempt in range(attempts):
            try:
                with self.opener.open(url, timeout=120) as response, open(partial, "wb") as out:
                    while chunk := response.read(1 << 20):
                        out.write(chunk)
                if not looks_valid(partial):
                    # Dropbox answers with a web page instead of the file when it throttles requests.
                    partial.unlink()
                    raise OSError("Dropbox returned a web page instead of the file (rate limited?)")
                partial.replace(target)
                return
            except OSError as error:
                if attempt == attempts - 1:
                    raise
                wait = 15 * 2**attempt
                print(f"  retrying {target.name} in {wait} s ({error})", file=sys.stderr)
                time.sleep(wait)


def looks_valid(path: Path) -> bool:
    """MATLAB files must start with the MATLAB header; nothing may be an HTML page."""
    with open(path, "rb") as handle:
        head = handle.read(64)
    if path.name.endswith(".mat.part") or path.suffix == ".mat":
        return head.startswith(b"MATLAB")
    return not head.lstrip().lower().startswith((b"<!doctype html", b"<html"))


def plan_downloads(
    folder: DropboxFolder, subjects: list[str], modes: list[str], sensors: list[str]
) -> list[tuple[str, str, int | None]]:
    """Return (relative path, href, size in bytes) for every file to fetch."""
    top = folder.list(ROOT_URL)
    by_name = {entry["filename"]: entry for entry in top}
    available = sorted(n for n, e in by_name.items() if e["is_dir"] and re.fullmatch(r"AB\d+", n))
    if subjects == ["all"]:
        subjects = available
    missing = [s for s in subjects if s not in available]
    if missing:
        raise SystemExit(f"unknown subjects {missing}; available: {available}")

    files = [(name, by_name[name]["href"], by_name[name].get("bytes")) for name in TOP_LEVEL_FILES]
    for subject in subjects:
        dates = [e for e in folder.list(by_name[subject]["href"]) if e["is_dir"] and e["filename"] != "osimxml"]
        for date in dates:
            mode_dirs = {e["filename"]: e for e in folder.list(date["href"]) if e["is_dir"]}
            for mode in modes:
                if mode not in mode_dirs:
                    print(f"  {subject}/{date['filename']}: no {mode} folder", file=sys.stderr)
                    continue
                sensor_dirs = {e["filename"]: e for e in folder.list(mode_dirs[mode]["href"]) if e["is_dir"]}
                for sensor in sensors:
                    if sensor not in sensor_dirs:
                        print(f"  {subject}/{date['filename']}/{mode}: no {sensor} folder", file=sys.stderr)
                        continue
                    for entry in folder.list(sensor_dirs[sensor]["href"]):
                        if entry["is_dir"]:
                            continue
                        rel = f"{subject}/{date['filename']}/{mode}/{sensor}/{entry['filename']}"
                        files.append((rel, entry["href"], entry.get("bytes")))
    return files


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--subjects", nargs="+", default=["AB06"], help="subject IDs such as AB06, or 'all'")
    parser.add_argument("--modes", nargs="+", default=list(ALL_MODES), choices=ALL_MODES)
    parser.add_argument("--sensors", nargs="+", default=list(DEFAULT_SENSORS))
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="target folder (default data/raw/camargo)")
    parser.add_argument("--dry-run", action="store_true", help="list the files and total size, download nothing")
    parser.add_argument("--manual", action="store_true", help="print the manual download steps and exit")
    args = parser.parse_args()

    if args.manual:
        print(MANUAL_STEPS)
        return 0

    try:
        folder = DropboxFolder()
        files = plan_downloads(folder, args.subjects, args.modes, args.sensors)
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        print(f"Could not list the Dropbox folder ({error}).\n", file=sys.stderr)
        print(MANUAL_STEPS, file=sys.stderr)
        return 1

    total = sum(size or 0 for _, _, size in files)
    print(f"{len(files)} files, {total / 1e6:.1f} MB")
    if args.dry_run:
        for rel, _, size in files:
            print(f"  {rel}  {size if size is not None else '?'} bytes")
        return 0

    for index, (rel, href, size) in enumerate(files, start=1):
        target = args.out / rel
        if target.exists() and (size is None or target.stat().st_size == size) and looks_valid(target):
            continue
        print(f"[{index}/{len(files)}] {rel}")
        try:
            folder.download(href, target)
        except OSError as error:
            print(f"Download failed for {rel} ({error}).\n", file=sys.stderr)
            print(MANUAL_STEPS, file=sys.stderr)
            return 1
    print(f"Done. Files are in {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
