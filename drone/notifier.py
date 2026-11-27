"""Sends delivered crowd alerts to the police station's e-mail.

Configure once in data/alert_config.json (NOT committed to git — it holds a password):

    {
      "email": {
        "enabled": true,
        "smtp_host": "smtp.gmail.com",
        "smtp_port": 587,
        "username": "yourname@gmail.com",
        "app_password": "abcd efgh ijkl mnop",
        "from_name": "GARUDA OPS Drone"
      }
    }

For Gmail, "app_password" is a 16-letter App Password (Google Account → Security →
2-Step Verification → App passwords), not your normal password.

Every alert is also written to data/alerts/outbox/<ALERT>.eml (a copy of exactly
what was sent), even when e-mail is not configured or sending fails.
"""
import json
import smtplib
import ssl
import threading
import time
from email.message import EmailMessage
from email.utils import formatdate, make_msgid
from pathlib import Path


def maps_url(lat, lon):
    return f'https://www.google.com/maps/search/?api=1&query={lat:.6f},{lon:.6f}'


class Notifier:
    def __init__(self, root, log):
        self.root = Path(root)
        self.log = log                      # callable(level, message)
        self.outbox = self.root / 'data' / 'alerts' / 'outbox'

    def config(self):
        f = self.root / 'data' / 'alert_config.json'
        try:
            return json.loads(f.read_text())
        except FileNotFoundError:
            return {}
        except Exception as e:
            self.log('WARN', f'alert_config.json unreadable: {e}')
            return {}

    def email_configured(self):
        now = time.time()
        if now - getattr(self, '_cfg_t', 0) > 3:            # re-read at most every 3 s
            c = self.config().get('email') or {}
            self._cfg_ok = bool(c.get('enabled') and c.get('username') and c.get('app_password')
                                and 'yourname@' not in c.get('username', ''))
            self._cfg_t = now
        return self._cfg_ok

    # ------------------------------------------------------------------ public
    def send(self, alert, station, snapshot, on_done):
        """Build and send the e-mail in a background thread (SMTP can take seconds;
        the flight-control loop must never wait for it)."""
        threading.Thread(target=self._run, args=(alert, station, snapshot, on_done), daemon=True).start()

    def build(self, alert, station, snapshot, sender='garuda-ops@localhost'):
        lat, lon = alert['latitude'], alert['longitude']
        link = maps_url(lat, lon)
        test = ' [TEST]' if alert.get('test') else ''
        msg = EmailMessage()
        msg['Subject'] = (f"CROWD ALERT{test} {alert['id']}: {alert['persons']} persons at "
                          f"{alert['zone']} ({lat:.5f}, {lon:.5f})")
        msg['From'] = sender
        msg['To'] = station.get('email') or 'not-configured@localhost'
        msg['Date'] = formatdate(localtime=True)
        msg['Message-ID'] = make_msgid('garuda')
        msg['X-Priority'] = '1'
        text = f"""GARUDA OPS - DRONE CROWD ALERT{test}

To: {station['name']}
Alert ID: {alert['id']}
Detected at: {alert['created']}
Operator approved at: {alert.get('decided') or '-'}
Delivered at: {alert.get('delivered') or '-'}

Persons detected near drone: {alert['persons']} (limit {alert['limit']}, level {alert['crowd_level']})
Area: {alert['zone']}
GPS: {lat:.6f}, {lon:.6f}
Open in Google Maps: {link}
Drone altitude: {alert['alt']} m

Snapshot SHA-256: {alert.get('snapshot_sha256') or '-'}
Recording session: {alert.get('recording') or 'not recording'}

Sent automatically by the GARUDA OPS drone dashboard after operator approval.
This is a simulation / research prototype.
"""
        msg.set_content(text)
        img_cid = make_msgid('snap')[1:-1]
        html = f"""<div style="font-family:Arial,sans-serif;max-width:640px">
<div style="background:#a32626;color:#fff;padding:14px 18px;border-radius:8px 8px 0 0;font-size:18px;font-weight:bold">
&#9888; DRONE CROWD ALERT{test} &middot; {alert['id']}</div>
<div style="border:1px solid #ddd;border-top:0;padding:16px 18px;border-radius:0 0 8px 8px">
<p style="font-size:16px;margin:0 0 12px"><b>{alert['persons']} persons</b> detected near the drone in
<b>{alert['zone']}</b> (limit {alert['limit']}).</p>
<p style="margin:0 0 14px"><a href="{link}" style="background:#1a5fd1;color:#fff;padding:10px 16px;border-radius:6px;
text-decoration:none;font-weight:bold">&#128205; Open location in Google Maps</a></p>
<table style="border-collapse:collapse;font-size:13px">
<tr><td style="padding:4px 12px 4px 0;color:#666">To</td><td><b>{station['name']}</b></td></tr>
<tr><td style="padding:4px 12px 4px 0;color:#666">GPS</td><td><b>{lat:.6f}, {lon:.6f}</b></td></tr>
<tr><td style="padding:4px 12px 4px 0;color:#666">Detected</td><td>{alert['created']}</td></tr>
<tr><td style="padding:4px 12px 4px 0;color:#666">Approved</td><td>{alert.get('decided') or '-'}</td></tr>
<tr><td style="padding:4px 12px 4px 0;color:#666">Delivered</td><td>{alert.get('delivered') or '-'}</td></tr>
<tr><td style="padding:4px 12px 4px 0;color:#666">Drone altitude</td><td>{alert['alt']} m</td></tr>
<tr><td style="padding:4px 12px 4px 0;color:#666">Snapshot SHA-256</td><td style="font-family:monospace;font-size:11px">{alert.get('snapshot_sha256') or '-'}</td></tr>
</table>
{f'<p><img src="cid:{img_cid}" style="width:100%;border-radius:6px;margin-top:12px"></p>' if snapshot else ''}
<p style="color:#888;font-size:11px">Sent by the GARUDA OPS drone dashboard after operator approval. Simulation / research prototype.</p>
</div></div>"""
        msg.add_alternative(html, subtype='html')
        if snapshot:
            msg.get_payload()[1].add_related(snapshot, 'image', 'jpeg', cid=f'<{img_cid}>',
                                             filename=f"{alert['id']}_snapshot.jpg")
        return msg

    # ------------------------------------------------------------------ worker
    def _run(self, alert, station, snapshot, on_done):
        cfg = (self.config().get('email') or {})
        sender = cfg.get('username') or 'garuda-ops@localhost'
        if cfg.get('from_name'):
            sender = f"{cfg['from_name']} <{sender}>"
        msg = self.build(alert, station, snapshot, sender)
        self.outbox.mkdir(parents=True, exist_ok=True)
        eml = self.outbox / f"{alert['id']}_{time.strftime('%Y%m%d_%H%M%S')}.eml"
        eml.write_bytes(bytes(msg))
        to = station.get('email', '').strip()
        if not cfg.get('enabled') or not cfg.get('username') or not cfg.get('app_password'):
            on_done('email', to or '-', 'SAVED TO OUTBOX (e-mail not configured)', eml.name)
            return
        if not to:
            on_done('email', '-', f"NOT SENT: no e-mail set for {station['name']}", eml.name)
            return
        try:
            with smtplib.SMTP(cfg.get('smtp_host', 'smtp.gmail.com'), int(cfg.get('smtp_port', 587)), timeout=20) as s:
                s.starttls(context=ssl.create_default_context())
                s.login(cfg['username'], cfg['app_password'].replace(' ', ''))
                s.send_message(msg)
            on_done('email', to, 'SENT', eml.name)
        except Exception as e:
            on_done('email', to, f'FAILED: {type(e).__name__}: {e}'[:160], eml.name)
