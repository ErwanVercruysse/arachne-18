"""Logins for the twin: guests may watch and play with the simulation, logged-in users may also move the real robot.

Accounts live OUTSIDE the repo (the repo is public): ~/.config/arachne/users.json holds PBKDF2 hashes only.

    python3 tools/auth.py set root        # set or change a password (asks twice)
    python3 tools/auth.py list
    python3 tools/auth.py hubkey          # make the robot <-> home-server key (copy the file to both)

Sessions are signed cookies (HMAC with ~/.config/arachne/secret), valid 30 days. Requests from the robot itself
(127.0.0.1 without a Cloudflare header) are trusted, so stand.py, walk.py… keep working without a login.
"""
import getpass, hashlib, hmac, json, os, secrets, sys, time
from pathlib import Path

DIR = Path(os.environ.get('ARACHNE_CONFIG', Path.home()/'.config/arachne'))
USERS, SECRET = DIR/'users.json', DIR/'secret'
COOKIE, DAYS = 'arachne_session', 30
_fails = {}          # ip -> [timestamps of failed logins]


def _users():
    try: return json.loads(USERS.read_text())
    except (OSError, ValueError): return {}


def _secret():
    if not SECRET.exists():
        DIR.mkdir(parents=True, exist_ok=True); SECRET.write_text(secrets.token_hex(32)); SECRET.chmod(0o600)
    return SECRET.read_text().strip().encode()


def hash_pw(pw, salt=None, it=200_000):
    salt = salt or secrets.token_hex(16)
    return f'pbkdf2_sha256${it}${salt}${hashlib.pbkdf2_hmac("sha256", pw.encode(), salt.encode(), it).hex()}'


def check_pw(user, pw):
    h = _users().get(user)
    if not h: hash_pw(pw); return False                 # same work either way
    _, it, salt, _ = h.split('$')
    return hmac.compare_digest(hash_pw(pw, salt, int(it)), h)


def set_pw(user, pw):
    u = _users(); u[user] = hash_pw(pw)
    DIR.mkdir(parents=True, exist_ok=True); USERS.write_text(json.dumps(u, indent=1)); USERS.chmod(0o600)


def make_token(user):
    exp = int(time.time()) + DAYS*86400
    msg = f'{user}.{exp}'
    return msg + '.' + hmac.new(_secret(), msg.encode(), hashlib.sha256).hexdigest()


def user_of(token):
    try:
        user, exp, sig = token.rsplit('.', 2)
        ok = hmac.compare_digest(sig, hmac.new(_secret(), f'{user}.{exp}'.encode(), hashlib.sha256).hexdigest())
        return user if ok and int(exp) > time.time() and user in _users() else None
    except (ValueError, AttributeError): return None


def cookie_user(headers):
    for part in (headers.get('Cookie') or '').split(';'):
        k, _, v = part.strip().partition('=')
        if k == COOKIE: return user_of(v)
    return None


def too_many(ip):
    now = time.time(); f = [t for t in _fails.get(ip, []) if now - t < 300]; _fails[ip] = f
    return len(f) >= 8                                   # 8 wrong passwords in 5 minutes -> wait


def failed(ip): _fails.setdefault(ip, []).append(time.time())


def has_accounts(): return bool(_users())


def hub_key():
    """Shared secret between the robot (Pi) and the home server (lvdell): ~/.config/arachne/hub_key on both.
    The Pi announces itself to the server with it, the server signs the commands it passes on to the Pi."""
    try: return (DIR/'hub_key').read_text().strip() or None
    except OSError: return None


def hub_ok(headers):
    k, got = hub_key(), headers.get('X-Arachne-Hub') or ''
    return bool(k) and hmac.compare_digest(got.encode(), k.encode())


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == 'set':
        pw = os.environ.get('ARACHNE_NEW_PW') or getpass.getpass(f'nieuw wachtwoord voor {sys.argv[2]}: ')
        if not os.environ.get('ARACHNE_NEW_PW') and pw != getpass.getpass('nog eens: '): raise SystemExit('niet hetzelfde')
        set_pw(sys.argv[2], pw); print(f'wachtwoord voor {sys.argv[2]} opgeslagen in {USERS}')
    elif len(sys.argv) >= 2 and sys.argv[1] == 'hubkey':
        if not hub_key():
            DIR.mkdir(parents=True, exist_ok=True); (DIR/'hub_key').write_text(secrets.token_hex(32)); (DIR/'hub_key').chmod(0o600)
        print(f'hub-sleutel in {DIR/"hub_key"} (kopieer hetzelfde bestand naar de Pi en de server)')
    elif len(sys.argv) >= 2 and sys.argv[1] == 'list': print('\n'.join(_users()) or '(geen accounts)')
    else: print(__doc__)
