"""Sound on the Mac for what the real robot does: a horn on Y, and a Flemish voice (Ellen) for stand up, lie down,
dance, speed changes, the status on L3…  Listens to the twin's /robot/events (long poll), so it works from any
computer that can reach the robot.

    python3 tools/mac_sounds.py                          # http://spin.local:5870
    python3 tools/mac_sounds.py https://….trycloudflare.com
"""
import json, math, os, struct, subprocess, sys, tempfile, time, urllib.request, wave

URL = (sys.argv[1] if len(sys.argv) > 1 else os.environ.get('ARACHNE_URL', 'http://spin.local:5870')).rstrip('/')
SOUNDS = {'verbonden': '/System/Library/Sounds/Glass.aiff', 'weg': '/System/Library/Sounds/Basso.aiff'}
HORN = os.path.join(tempfile.gettempdir(), 'arachne_toeter.wav')


def make_horn():
    """Two-tone car horn (a major third, slightly square, 0.7 s) written once as a wav."""
    if os.path.exists(HORN): return
    sr, dur = 44100, .7; frames = bytearray()
    for i in range(int(sr*dur)):
        t = i/sr; env = min(1, t/.02, (dur - t)/.08)
        v = sum(math.tanh(3*math.sin(2*math.pi*f*t)) for f in (392, 494))*.22*env
        frames += struct.pack('<h', int(max(-1, min(1, v))*32767))
    with wave.open(HORN, 'wb') as w: w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr); w.writeframes(bytes(frames))


def play(e):
    if e.get('sound') == 'toeter': subprocess.Popen(['afplay', HORN])
    elif e.get('sound') in SOUNDS: subprocess.Popen(['afplay', SOUNDS[e['sound']]])
    if e.get('say'): subprocess.Popen(['say', '-v', 'Ellen', '-r', '190', e['say']])
    print(time.strftime('%H:%M:%S'), e.get('say') or e.get('sound'), flush=True)


def main():
    make_horn(); since = None; told = False
    print(f'luisteren naar {URL}/robot/events …', flush=True)
    while True:
        try:
            q = f'?since={since}' if since is not None else '?since=-1'      # first call: skip old events
            r = json.loads(urllib.request.urlopen(f'{URL}/robot/events{q}', timeout=30).read())
            if since is not None:
                for e in r.get('events', []): play(e)
            since = r.get('n', since); told = False
        except Exception as e:
            if not told: print('geen verbinding:', e, flush=True); told = True
            time.sleep(2)


if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: pass
