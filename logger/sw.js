/* BoreLogger service worker.
 *
 * Strategy is network-first, cache-as-fallback. Cache-first would be faster,
 * but this app is a single HTML file that changes every time the firmware
 * protocol changes -- serving a stale copy against new firmware produces
 * exactly the silent mismatches that are hardest to debug. Freshness matters
 * more than a few hundred milliseconds of load time here.
 *
 * Bump CACHE_VERSION whenever you want old caches purged.
 */
const CACHE_VERSION = 'borelogger-v1';

const PRECACHE = [
  './',
  './index.html',
  './manifest.json',
  './icons/icon-192.png',
  './icons/icon-512.png'
];

self.addEventListener('install', event => {
  event.waitUntil(
    caches.open(CACHE_VERSION)
      // individual failures must not abort the whole install
      .then(c => Promise.allSettled(PRECACHE.map(u => c.add(u))))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener('activate', event => {
  event.waitUntil(
    caches.keys()
      .then(keys => Promise.all(
        keys.filter(k => k !== CACHE_VERSION).map(k => caches.delete(k))))
      .then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', event => {
  const req = event.request;

  // Only GETs from our own origin. Never touch anything else -- and note
  // Bluetooth and geolocation do not go through fetch at all, so nothing
  // here can interfere with them.
  if (req.method !== 'GET' || new URL(req.url).origin !== self.location.origin) {
    return;
  }

  event.respondWith(
    fetch(req)
      .then(res => {
        if (res && res.status === 200 && res.type === 'basic') {
          const copy = res.clone();
          caches.open(CACHE_VERSION).then(c => c.put(req, copy));
        }
        return res;
      })
      .catch(() => caches.match(req).then(hit => hit || caches.match('./index.html')))
  );
});
