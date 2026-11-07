"""Regenerate manifest.json after replacing main.py in this folder.

    python3 make_manifest.py "what changed"

The probe's page (http://10.10.10.10, Update tab) reads manifest.json from
https://egeozgul.com/board and installs every file whose SHA-256 differs from
the probe's copy. main.py is the logger: on the probe it lives in logger.py,
behind the small USB-only launcher that undoes a failed update.
Then commit and push; probes see it within about 10 minutes (site cache).
"""
import hashlib, json, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
# (name on the probe, file in this folder)
FILES = [('logger.py', 'main.py')]

notes = sys.argv[1] if len(sys.argv) > 1 else ''
files = []
for name, path in FILES:
    data = open(os.path.join(HERE, path), 'rb').read()
    try:
        compile(data.decode('utf-8'), path, 'exec')
    except SyntaxError as e:
        sys.exit('%s does not compile: %s' % (path, e))
    files.append({'name': name, 'path': path, 'size': len(data),
                  'sha256': hashlib.sha256(data).hexdigest()})
version = time.strftime('%Y.%m.%d-%H%M') + '-' + files[0]['sha256'][:8]
m = {'version': version, 'notes': notes,
     'released': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), 'files': files}
with open(os.path.join(HERE, 'manifest.json'), 'w') as f:
    json.dump(m, f, indent=1)
    f.write('\n')
print('manifest.json: version', version)
for x in files:
    print('  %-10s <- %-8s %7d bytes  %s' % (x['name'], x['path'], x['size'], x['sha256'][:16]))
