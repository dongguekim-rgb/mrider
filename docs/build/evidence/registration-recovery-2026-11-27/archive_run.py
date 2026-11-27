"""Archive raw simulator reports without overwriting an earlier result.

Usage: archive_run.py RAW_DIR NEW_ARCHIVE_DIR SOURCE_SNAPSHOT_DIR
JSON/log payloads are compressed; XML, images and existing gzip files remain
unchanged. source-sha256.json identifies the runtime used by the experiment.
"""
import gzip
import shutil
import sys
from pathlib import Path

raw, output, snapshot = map(Path, sys.argv[1:])
output.mkdir(parents=True, exist_ok=False)
for source in raw.rglob('*'):
    if not source.is_file():
        continue
    target = output / source.relative_to(raw)
    target.parent.mkdir(parents=True, exist_ok=True)
    if source.suffix in ('.json', '.log') and source.name not in ('summary.json', 'start.json'):
        with source.open('rb') as src, gzip.open(str(target) + '.gz', 'wb') as dest:
            shutil.copyfileobj(src, dest)
    else:
        shutil.copy2(source, target)
for name in ('source-sha256.json', 'source.tar.gz'):
    shutil.copy2(snapshot / name, output / name)
print(output)
