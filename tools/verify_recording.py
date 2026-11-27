"""Check that a recorded GARUDA OPS flight has not been altered.

    python tools/verify_recording.py session_20261127_001234
    python tools/verify_recording.py                 # checks every recording

Recomputes the SHA-256 fingerprint of the video and the dataset and compares
them with the manifest written when recording stopped.  Any edit, cut or
re-encode of the files changes the fingerprint.

Note: this proves the files match the manifest.  For use as evidence, also
store the manifest (or just its video_sha256 line) somewhere the operator
cannot change, e.g. e-mail it to the station or print it, at the end of the flight.
"""
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def check(manifest_path):
    m = json.loads(manifest_path.read_text())
    print(f"\nSession   : {m['session']}")
    print(f"Recorded  : {m['start_time']}  ->  {m['end_time']}  ({m['duration_sec']} s, {m['source_frames']} frames)")
    ok = True
    for kind, folder in (('video', 'media'), ('dataset', 'data')):
        name, expected = m.get(f'{kind}_file'), m.get(f'{kind}_sha256')
        if not name or not expected:
            print(f'{kind:<9} : (none recorded)')
            continue
        f = ROOT / folder / name
        if not f.is_file():
            print(f'{kind:<9} : MISSING  {name}')
            ok = False
            continue
        actual = sha256_file(f)
        match = actual == expected
        ok &= match
        print(f"{kind:<9} : {'OK      ' if match else 'MISMATCH'} {name}")
        if not match:
            print(f'            expected {expected}\n            actual   {actual}')
    print('RESULT    :', 'VERIFIED - files are unchanged since recording' if ok
          else 'TAMPERED OR MISSING - files do not match the manifest')
    return ok


if __name__ == '__main__':
    data = ROOT / 'data'
    if len(sys.argv) > 1:
        name = sys.argv[1].replace('.manifest.json', '')
        paths = [data / f'{name}.manifest.json']
    else:
        paths = sorted(data.glob('session_*.manifest.json'))
    if not paths or not paths[0].is_file():
        sys.exit('No manifest found. Record a flight first (Record flight -> Stop recording).')
    results = [check(p) for p in paths]
    sys.exit(0 if all(results) else 1)
