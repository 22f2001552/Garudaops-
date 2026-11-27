"""Send one test crowd-alert e-mail, to check data/alert_config.json works.

    python tools/test_email.py someone@example.com
"""
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from drone.notifier import Notifier, maps_url  # noqa: E402

if len(sys.argv) < 2:
    sys.exit('Usage: python tools/test_email.py recipient@example.com')
lat, lon = 17.4435, 78.3772
alert = dict(id='ALERT-TEST', test=True, created='(test e-mail)', decided='(test)', delivered='(test)',
             persons=18, limit=12, crowd_level='HIGH', zone='SECTOR ALPHA', alt=5.0,
             latitude=lat, longitude=lon, maps_url=maps_url(lat, lon), snapshot_sha256=None, recording=None)
station = dict(name='Test Police Station', email=sys.argv[1])
done = threading.Event()


def finished(channel, to, status, file):
    print(f'\nE-mail to {to}: {status}\nCopy saved: data/alerts/outbox/{file}')
    done.set()


print('Sending...')
Notifier(ROOT, lambda lvl, m: print(lvl, m)).send(alert, station, None, finished)
done.wait(40)
