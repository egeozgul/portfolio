# main.py — ESP32 BLE sensor logger, 26.3 Hz by default
#
# Streams IMU + barometer + magnetometer + battery over BLE Notify, and
# optionally records the same samples to the on-board SPI flash.
#
# Hardware (as per your existing wiring):
#   I2C1  SDA=5  SCL=6   -> LSM6DS* IMU @0x6B, MS58xx barometer @0x76
#   SPI1  SCK=7 MOSI=9 MISO=8
#         CS=4  -> W25Q128 flash (16 MB)
#         CS=2  -> RM3100 magnetometer
#   LED=3, battery divider on ADC GPIO1
#
# BLE protocol (see the HTML app for the matching decoder)
#   Service      12345678-1234-5678-1234-56789abcdef0
#     Data  (N)  ...def1   frames pushed by the device
#     Ctrl  (W)  ...def2   1-byte commands from the app
#     Stat  (RN) ...def3   status frames
#
#   Frame = [type][payload].  If a frame is larger than the negotiated MTU
#   allows, it is split: 0xF1 [total length] = first chunk, 0xF2 = following.
#
#   Frames from the board (Data, except 0x10 on Stat)
#     0x01 live sample: record + health byte    0x10 status (also readable;
#          flags: 1 recording, 2 dumping, 4 foreign log, 8 streaming,
#          16 verifying, 32 erasing, 64 bridge workaround active)
#     0x22 stored records [count][records]      0x21 dump end [count u32]
#     0x31 erase progress [pct][done][total]    0x32 erase end, same fields
#     0x33 verify progress [pct][n][total]      0x30 verify result [ok][first bad][kB]
#     0x40 raw bytes [addr u32][data]           0x41 raw read end [addr u32]
#     0x50 session index entry                  0x51 index end
#     0x60 config.json chunk [seq][data]        0x61 config end [len u16][sum u16]
#
#   Commands to the board (Ctrl)
#     0x01 start [epoch u32][tz i16][kind][lat i32][lon i32][acc u16][gps][note]
#     0x02 stop       0x03 erase      0x04 dump all      0x05 status
#     0x06 abort the running erase / dump / verify / raw read
#     0x07 sample period [ms u16]     0x08 verify blank
#     0x09 stream [on][ms u16]        0x0B raw read [addr u32][len u16]
#     0x0C session index              0x0D range dump [addr u32][count u32]
#     0x0E config.json
#   Erase, dumps, verify and raw read are refused while recording, as are a
#   second start and a rate change; every refusal is answered with status.

import bluetooth
import gc
import struct
import time
import json
from machine import Pin, I2C, SPI, ADC
from micropython import const

# ── device config ────────────────────────────────────────────────────
# config.json on the board's filesystem identifies the device, and will
# later hold its calibration values. Those are only ever STORED here: the
# board always records raw data, and corrections are applied afterwards.
#
#   { "device_id": 3 }
#
# A missing or unreadable file never stops the board: it falls back to
# defaults, says so at start-up, and records exactly as before.
CONFIG_FILE = 'config.json'

# Calibration values, STORED here and never applied to what is recorded --
# the board always logs raw data; corrections happen afterwards, so they can
# be improved and re-applied to old surveys. Order when they are applied:
#   accelerometer:  (raw - bias_g) / scale, then rotate by align_deg
#   magnetometer:   soft_iron x (raw - hard_iron_uT), then axis_map,
#                   then rotate by align_deg onto the accelerometer frame
CAL_DEFAULTS = {
    'cal_version': 0,              # which calibration this is; 0 = none yet
    'cal_date': '',
    'accel': {
        'bias_g':    [0.0, 0.0, 0.0],
        'scale':     [1.0, 1.0, 1.0],      # a ratio: 1 means no correction
        'align_deg': [0.0, 0.0],           # tilt of the chip in the tube
    },
    'mag': {
        'axis_map':     'x,-y,-z',         # confirmed by pose checks
        'hard_iron_uT': [0.0, 0.0, 0.0],
        'soft_iron':    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        'align_deg':    [0.0, 0.0, 0.0],   # magnetometer onto accelerometer
    },
}

def _num(v):
    # numbers only: true/false in JSON are ints in Python, so exclude bool
    return type(v) in (int, float)

def _vec(v, n, lo, hi):
    if not (isinstance(v, list) and len(v) == n and all(_num(x) for x in v)):
        return None
    if not all(lo <= x <= hi for x in v):
        return None
    return [float(x) for x in v]

def _mat3(v):
    if not (isinstance(v, list) and len(v) == 3):
        return None
    rows = [_vec(r, 3, -10.0, 10.0) for r in v]
    return None if None in rows else rows

def _axis_map(v):
    if not isinstance(v, str):
        return None
    parts = v.replace(' ', '').split(',')
    if len(parts) != 3:
        return None
    axes = [p[-1:] for p in parts]
    if sorted(axes) != ['x', 'y', 'z']:
        return None
    if not all(p in ('x', 'y', 'z', '-x', '-y', '-z') for p in parts):
        return None
    return ','.join(parts)

# (section, key, checker, plain-language limit for the warning)
CAL_FIELDS = (
    ('accel', 'bias_g',       lambda v: _vec(v, 3, -0.2, 0.2),    '3 numbers within +/-0.2 g'),
    ('accel', 'scale',        lambda v: _vec(v, 3, 0.8, 1.2),     '3 numbers between 0.8 and 1.2'),
    ('accel', 'align_deg',    lambda v: _vec(v, 2, -10.0, 10.0),  '2 numbers within +/-10 deg'),
    ('mag',   'axis_map',     _axis_map,                          'like "x,-y,-z"'),
    ('mag',   'hard_iron_uT', lambda v: _vec(v, 3, -500.0, 500.0),'3 numbers within +/-500 uT'),
    ('mag',   'soft_iron',    _mat3,                              'a 3x3 list of numbers'),
    ('mag',   'align_deg',    lambda v: _vec(v, 3, -10.0, 10.0),  '3 numbers within +/-10 deg'),
)

def load_config():
    cfg = {'device_id': None}
    data = {}
    try:
        with open(CONFIG_FILE) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError('not a JSON object')
        status = 'loaded'
    except OSError:
        data = {}
        status = 'no ' + CONFIG_FILE + ', using defaults'
    except ValueError as e:
        data = {}
        status = CONFIG_FILE + ' unreadable (' + str(e) + '), using defaults'
    warn = []

    did = data.get('device_id')
    # type() rather than isinstance(): true/false in JSON would pass as ints
    if not (type(did) is int and 1 <= did <= 65535):
        if did is not None:
            warn.append('device_id ' + repr(did) + ' ignored (must be 1-65535)')
        did = None
    cfg['device_id'] = did

    v = data.get('cal_version', 0)
    if not (type(v) is int and 0 <= v <= 65535):
        warn.append('cal_version ' + repr(v) + ' ignored (must be 0-65535)')
        v = 0
    cfg['cal_version'] = v
    d = data.get('cal_date', '')
    cfg['cal_date'] = d if isinstance(d, str) else ''

    # each value checked on its own: one bad entry falls back to ITS default
    for sec in ('accel', 'mag'):
        cfg[sec] = {}
        given = data.get(sec, {})
        if not isinstance(given, dict):
            if sec in data:
                warn.append(sec + ' is not an object, using defaults')
            given = {}
        for s2, key, check, rule in CAL_FIELDS:
            if s2 != sec:
                continue
            default = CAL_DEFAULTS[sec][key]
            if key not in given:
                cfg[sec][key] = default
                continue
            good = check(given[key])
            if good is None:
                warn.append(sec + '.' + key + ' ignored (must be ' + rule + ')')
                cfg[sec][key] = default
            else:
                cfg[sec][key] = good
    if warn:
        status += '; ' + '; '.join(warn)
    return cfg, status

def cal_summary(cfg):
    a, m = cfg['accel'], cfg['mag']
    if cfg['cal_version'] == 0 and a == CAL_DEFAULTS['accel'] and \
            m['hard_iron_uT'] == CAL_DEFAULTS['mag']['hard_iron_uT'] and \
            m['soft_iron'] == CAL_DEFAULTS['mag']['soft_iron'] and \
            m['align_deg'] == CAL_DEFAULTS['mag']['align_deg']:
        return 'none yet (defaults), mag axes ' + m['axis_map']
    f = lambda v: '[' + ', '.join('%g' % x for x in v) + ']'
    return ('v%d %s | accel bias %s g, scale %s, align %s deg | mag axes %s, '
            'hard iron %s uT, align %s deg' % (
                cfg['cal_version'], cfg['cal_date'] or '(no date)',
                f(a['bias_g']), f(a['scale']), f(a['align_deg']),
                m['axis_map'], f(m['hard_iron_uT']), f(m['align_deg'])))

CONFIG, CONFIG_STATUS = load_config()
DEVICE_ID = CONFIG['device_id']

# ─── CONFIG ────────────────────────────────────────────────────────────
DEFAULT_PERIOD_MS = 38          # 26.3 Hz at power-on. 1000/38 is 26.316,
                                # about 104/4 against the IMU's output rate,
                                # and short enough that the barometer uses
                                # the stepped path rather than blocking
MIN_PERIOD_MS = 10              # 100 Hz ceiling
MAX_PERIOD_MS = 60000           # ~0.017 Hz floor (uint16 ms on the wire)
OVERSAMPLE_MS = 10              # IMU read spacing while waiting for the tick
ADV_INTERVAL_MS = 100           # advertising interval. 100 ms discovers
                                # almost instantly and costs ~1.8 mA of radio;
                                # 1000 ms drops that to ~0.18 mA but makes the
                                # device picker take a second or two
LED_MIN_MS = 8                  # shortest blink worth attempting
LED_MAX_MS = 70                 # longest, so it still reads as a blink

# Barometer variant. The compensation maths differs between MS58xx parts.
#   MS5837-30BA          -> 8192
#   MS5611 / MS5637 / MS5803-01BA -> 32768
# If pressure reads ~4x high or low at sea level (should be ~1013 hPa),
# this is the constant to change.
BARO_P_DIV = const(8192)

# IMU full-scale, must match the CTRL register values written in init_imu()
IMU_TEMP_LSB   = 256.0         # LSB per degree C, zero at 25 C
ACC_LSB_PER_G  = 16384.0        # +/-2 g
GYRO_MDPS_LSB  = 8.75           # +/-245 dps  (note: NOT 131 - that is MPU6050)

# Flash layout
SECTOR   = const(4096)
REC_SIZE = const(32)
# The header has to own a whole 4 kB sector, because that is the smallest
# erasable unit -- rewriting it erases everything sharing the sector. Putting
# it in the LAST sector instead of the first lets records start at address 0,
# so a record index maps straight onto an address: record N is at N * REC_SIZE.
FLASH_END = const(0x1000000)    # 16 MB part
HDR_ADDR  = const(0xFFF000)     # final sector
DATA_START = const(0x000000)    # records begin at zero
LOG_END   = const(0xFFF000)     # records end where the header begins
IDX_SIZE  = const(64)           # one session index entry
NOTE_LEN  = const(32)           # bytes of free text per session
IDX_TOP   = const(0xFFF000)     # index grows downward from just under the header
MAX_SESSIONS = const(4096)
# Records stop below the index's FULL reservation, not just below the entries
# written so far. find_log_end() can only search below this address -- above
# it, index entries and records would be indistinguishable -- so a record
# written higher up would be lost at the next boot, and the write pointer
# put back on top of existing data. Costs 256 kB of a 16 MB log.
REC_LIMIT = const(0xFFF000 - 4096 * 64)     # IDX_TOP - MAX_SESSIONS * IDX_SIZE

# Stamped into the header sector. NOR flash can only clear bits, so writing
# a record on top of stale data yields (old AND new) rather than an error --
# silent, plausible-looking garbage. The header records which layout owns the
# log so a firmware with a different record format refuses to append to it
# instead of destroying it.
LOG_MAGIC = b'ESPLOG\x0B' + bytes([REC_SIZE])

# Runtime sampling state. The barometer OSR is traded down as the rate rises,
# because its conversion wait dominates the loop budget.
sample_ms = DEFAULT_PERIOD_MS
# Placeholders only. set_period() runs at startup and sets these from
# DEFAULT_PERIOD_MS, so they are never used as written.
baro_d1, baro_d2 = 0x48, 0x58
baro_wait_us = 11000
baro_async = False

# Conversion time per oversampling ratio, from the datasheet, keyed by the
# D1 command byte. time.sleep_ms rounds up to the scheduler tick -- measured
# at 12.2 ms whether asked for 4 or 12 -- so the wait is done in microseconds
# instead, which busy-waits and is accurate.
BARO_CONV_US = {0x40: 600, 0x42: 1170, 0x44: 2280, 0x46: 4540, 0x48: 9040}
BARO_UNPRIMED = const(-1)   # stepped cycle has no conversion in flight
baro_state = BARO_UNPRIMED
baro_D1 = 0
last_press = 0
last_temp = 0

# ─── HARDWARE ──────────────────────────────────────────────────────────
# timeout matters as much as the pins here. The default bus timeout is
# around a second, so a sensor that stops acknowledging does not fail --
# it BLOCKS, and one dead device drags a 38 ms loop out to over a second.
# 8 ms is far longer than any legitimate transaction on this bus (the
# longest is a 14-byte burst at 400 kHz, well under 1 ms) and short enough
# that a dead device costs a fraction of one sample period.
# i2c is created further down, AFTER detect_bridge() has finished with
# GPIO5. Building it here would hand the pin to the peripheral and the
# detection would then be measuring the peripheral rather than the net.
spi = SPI(1, baudrate=1000000, polarity=0, phase=0,
          sck=Pin(7), mosi=Pin(9), miso=Pin(8))

# ─── TEMPORARY: SOLDER BRIDGE WORKAROUND ───────────────────────────────
#
#   THIS EXISTS TO WORK AROUND A HARDWARE FAULT. REMOVE IT ONCE THE BOARD
#   IS REPAIRED. Set BRIDGE_WORKAROUND = False to disable it.
#
# On this board GPIO4 (flash chip select) is shorted to GPIO5 (I2C SDA).
# Confirmed by driving each pin and reading the other, in both directions:
#
#     driving CS_MEM low  ->  SDA 30/30, SCL 0/30
#     driving SDA    low  ->  CS_MEM 30/30, SCL 0/30
#     driving SCL    low  ->  CS_MEM 0/30, SDA 0/30
#
# Two consequences, and two fixes:
#
# 1. cs_mem used to be Pin.OUT driving HIGH to deselect the flash. Through
#    the bridge that actively held SDA high with a push-pull driver, and
#    nothing could pull SDA low, so no START condition was possible and
#    every I2C transaction returned ETIMEDOUT -- never ENODEV, because no
#    address was ever sent. OPEN_DRAIN makes "high" mean "release", and the
#    I2C pull-up already on the net holds it up, which also keeps the flash
#    deselected.
#
# 2. When CS_MEM is driven LOW for a real flash access it drags SDA low.
#    SDA falling while SCL is high is a START condition; rising is a STOP.
#    Holding SCL LOW for the duration suppresses both, because the I2C
#    specification only allows SDA to change while SCL is low -- so no
#    slave registers anything.
#
# Borrowing SCL takes the pin from the I2C peripheral, which then has to be
# rebuilt. Measured at 651 us per flash access, 1.7% of the loop at 26.3 Hz,
# and stable over 100 consecutive cycles.
#
# The guard MUST wrap a whole flash operation, never an individual
# mem_cmd(): one sample makes ~14 of those, and guarding each would cost
# 9.1 ms instead of 0.65 ms.
#
# This is safe only because I2C and SPI never overlap -- single threaded,
# no DMA, and the BLE IRQ touches no bus. Any change to that breaks it
# silently, which is the main reason to repair the board rather than live
# with this.

def detect_bridge():
    """Is GPIO4 actually shorted to GPIO5, right now?

    Assuming the fault is still there is dangerous in BOTH directions:

      assumed present, actually fixed -> cs_mem is open drain with nothing
        but the internal pull-up holding it, and if that were ever left off
        the flash chip select would FLOAT. A floating CS on a bus the
        magnetometer keeps clocking means the flash gets selected at random
        and reads garbage.

      assumed absent, actually present -> cs_mem drives high, pins SDA, and
        I2C dies exactly as it did before.

    So measure it. Both pins are set open drain with pull-ups, GPIO4 is
    driven low, and GPIO5 is read. If they are one net, GPIO5 follows.

    This runs BEFORE the I2C object is created, so it cannot steal the pin
    from the peripheral -- the mistake that made three earlier diagnostics
    meaningless. The same test runs again later if the I2C bus dies during
    a run (see bridge_recheck), because this fault comes and goes.
    """
    return bridge_present()


def bridge_present():
    """The pin test itself. A hit needs GPIO5 to follow GPIO4 BOTH ways:
    low while GPIO4 is driven low, and back high once it is released. An
    SDA line stuck low for some other reason (a sensor holding it, a short
    to ground) only ever reads low, and must not be taken for the bridge."""
    a = Pin(4, Pin.OPEN_DRAIN, Pin.PULL_UP, value=1)
    b = Pin(5, Pin.OPEN_DRAIN, Pin.PULL_UP, value=1)
    time.sleep_ms(2)
    hits = 0
    for _ in range(8):
        a.value(0)
        time.sleep_us(50)
        low = b.value() == 0
        a.value(1)
        time.sleep_us(50)
        if low and b.value() == 1:
            hits += 1
    a.value(1)
    b.value(1)
    return hits >= 7          # 8 tries, allow one glitch either way


BRIDGE_WORKAROUND = detect_bridge()

# The internal pull-up is enabled either way, and it is not optional. With
# the bridge, the I2C pull-up dominates and this does nothing. Without it,
# this is the only thing keeping the chip select from floating.
cs_mem = Pin(4, Pin.OPEN_DRAIN, Pin.PULL_UP, value=1) if BRIDGE_WORKAROUND \
    else Pin(4, Pin.OUT, value=1)
cs_mag = Pin(2, Pin.OUT, value=1)
led = Pin(3, Pin.OUT, value=0)

i2c = I2C(1, sda=Pin(5), scl=Pin(6), freq=400000, timeout=8000)

_guard_depth = 0
_guard_fails = 0

def flash_guard_enter():
    """Park SCL low so the bridge cannot signal on the I2C bus."""
    global _guard_depth
    if not BRIDGE_WORKAROUND:
        return
    if _guard_depth == 0:
        Pin(6, Pin.OUT, value=0)
    _guard_depth += 1

def flash_guard_exit():
    """Release SCL and give the pin back to the I2C peripheral."""
    global _guard_depth, i2c, _guard_fails
    if not BRIDGE_WORKAROUND:
        return
    _guard_depth -= 1
    if _guard_depth > 0:
        return
    _guard_depth = 0
    try:
        Pin(6, Pin.IN, Pin.PULL_UP)
        i2c = I2C(1, sda=Pin(5), scl=Pin(6), freq=400000, timeout=8000)
    except Exception as e:
        # losing the bus here would take both I2C sensors down until
        # retry_failed() next runs, so it is worth knowing about
        _guard_fails += 1
        if _guard_fails <= 3:
            print('flash guard: I2C rebuild failed:', e)

def bridge_recheck():
    """Test for the GPIO4-GPIO5 short again, during a run, and switch to the
    workaround if it is there.

    The short is intermittent: on 2026-11-03 it closed deep in the hole on a
    board that had booted without it, chip-select (push-pull high) then
    pinned SDA high, and the IMU and barometer were lost for up to 12 min.
    The boot-time check cannot see that. Called from retry_failed() only
    when BOTH I2C sensors are down and the SPI magnetometer still works --
    the short's signature -- and never inside a flash operation.

    Found: chip-select becomes open drain and every later flash access parks
    SCL, exactly as when the short is found at boot. The switch is kept
    until the next boot: the workaround is safe without the short too.
    Not found: chip-select goes back to push-pull. Either way the I2C bus is
    rebuilt, because the test borrows SDA from the peripheral."""
    global BRIDGE_WORKAROUND, cs_mem, i2c
    if BRIDGE_WORKAROUND or _guard_depth:
        return False
    flash_sync()                 # let a page program finish before touching CS
    found = bridge_present()
    if found:
        BRIDGE_WORKAROUND = True
        cs_mem = Pin(4, Pin.OPEN_DRAIN, Pin.PULL_UP, value=1)
    else:
        cs_mem = Pin(4, Pin.OUT, value=1)
    try:
        i2c = I2C(1, sda=Pin(5), scl=Pin(6), freq=400000, timeout=8000)
    except Exception as e:
        print('bridge recheck: I2C rebuild failed:', e)
    return found

bat_adc = ADC(Pin(1))
bat_adc.atten(ADC.ATTN_11DB)

IMU_ADDR = const(0x6B)
MS_ADDR = const(0x76)
C = []

# The RM3100 tops out around 1 MHz SPI, the W25Q is happy at tens of MHz.
# They share a bus, so the clock is retuned per device.
# spi.init() reconfigures the peripheral, which is far from free. The two
# devices need different clocks -- the RM3100 tops out near 1 MHz, the flash
# is happy at 8 -- but re-applying a clock that is already set is pure waste,
# and this was happening five times per sample.
_spi_hz = 1000000

def _spi_at(hz):
    global _spi_hz
    if _spi_hz != hz:
        spi.init(baudrate=hz)
        _spi_hz = hz

def spi_flash():
    _spi_at(8000000)

def spi_mag():
    _spi_at(1000000)


# ─── FLASH ─────────────────────────────────────────────────────────────
def mem_cmd(cmd_bytes, read_len=0):
    spi_flash()
    cs_mag.value(1)
    cs_mem.value(0)
    spi.write(cmd_bytes)
    data = spi.read(read_len) if read_len else None
    cs_mem.value(1)
    return data

_flash_busy = False
_ST_CMD = b'\x05'
_ST_BUF = bytearray(1)

def mem_status():
    """Read status register 1 into a preallocated buffer (called per record)."""
    spi_flash()
    cs_mag.value(1)
    cs_mem.value(0)
    spi.write(_ST_CMD)
    spi.readinto(_ST_BUF)
    cs_mem.value(1)
    return _ST_BUF[0]

def mem_wait_ready(timeout_ms=30000):
    """Poll the BUSY bit. There is no ready line on this part, so asking is
    the only way to know -- and while it is busy the chip silently ignores
    every command except this one."""
    global _flash_busy
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        if not (mem_status() & 0x01):
            _flash_busy = False
            return True
        time.sleep_ms(1)
    return False

def flash_sync():
    """Wait only if something might still be in progress.

    A page program takes 0.4 ms, but nothing needs the flash again until the
    next sample -- 10 ms later at 100 Hz. Blocking straight after the write
    meant standing over an operation that would have finished by itself, and
    time.sleep_ms(1) rounded 0.4 ms up to 1.1 ms of wall clock. Deferring the
    check to the next access costs one status read instead."""
    if _flash_busy:
        mem_wait_ready()

def mem_write_enable():
    mem_cmd(b'\x06')                    # callers flash_sync() first

def _addr(a):
    return bytes([(a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF])

def mem_read(addr, n):
    flash_sync()
    return mem_cmd(b'\x03' + _addr(addr), n)

_VBUF = bytearray(1024)
_VBLANK = b'\xff' * 1024

def mem_read_into(addr, buf):
    """Read straight into a preallocated buffer. The blank check walks the
    whole 16 MB, and allocating a fresh bytes object per chunk would spend
    most of the scan in the garbage collector."""
    flash_sync()
    spi_flash()
    cs_mag.value(1)
    cs_mem.value(0)
    spi.write(b'\x03' + _addr(addr))
    spi.readinto(buf)
    cs_mem.value(1)

def mem_erase_sector(addr):
    flash_sync()
    mem_write_enable()
    mem_cmd(b'\x20' + _addr(addr))
    mem_wait_ready()          # 45 ms typical: worth sleeping through

def mem_erase_block64(addr):
    flash_sync()
    mem_write_enable()
    mem_cmd(b'\xD8' + _addr(addr))
    mem_wait_ready()          # 150 ms typical

def mem_write(addr, data):
    # W25Q pages are 256 bytes and a program cannot cross a page boundary.
    global _flash_busy
    while data:
        flash_sync()                    # the previous program, if any
        room = 256 - (addr % 256)
        chunk = data[:room]
        data = data[len(chunk):]
        mem_write_enable()
        mem_cmd(b'\x02' + _addr(addr) + chunk)
        _flash_busy = True              # deliberately not waited on here
        addr += len(chunk)

def is_erased(addr, n):
    d = mem_read(addr, n)
    for b in d:
        if b != 0xFF:
            return False
    return True

def find_log_end():
    """First free record slot. Binary search for the first blank sector,
    then a short linear scan inside the sector before it.

    The search assumes blankness is monotonic: written sectors, then empty
    ones. The session index breaks that by living at the TOP of the same
    range, so the true pattern is written / blank / written. The upper
    bound therefore stops below the index rather than at LOG_END --
    otherwise a nearly-full log can walk lo up into the index and return a
    first-free address that is really an index entry.
    """
    lo = DATA_START // SECTOR
    hi = REC_LIMIT // SECTOR
    first_free = hi
    while lo < hi:
        mid = (lo + hi) // 2
        if mem_read(mid * SECTOR, 4) == b'\xff\xff\xff\xff':
            first_free = mid
            hi = mid
        else:
            lo = mid + 1
    if first_free == DATA_START // SECTOR:
        return DATA_START
    base = (first_free - 1) * SECTOR
    # records pack contiguously from DATA_START, so align the scan to the grid
    p = base - ((base - DATA_START) % REC_SIZE)
    while p < base + SECTOR:
        if is_erased(p, REC_SIZE):
            return p
        p += REC_SIZE
    return p

# Records pack contiguously. With the region pre-erased nothing is erased
# mid-log, so a record spanning a sector boundary is harmless and the old
# skip-to-next-sector padding is no longer needed.

log_ptr = DATA_START
log_foreign = False
prepared_end = 0        # flash below this address is known to be erased
sessions = 0            # index entries written so far

def idx_addr(i):
    """Entry i lives IDX_SIZE below entry i-1: the table grows downward."""
    return IDX_TOP - (i + 1) * IDX_SIZE

def data_limit():
    """Records may grow up to the index reservation (see REC_LIMIT)."""
    return REC_LIMIT if REC_LIMIT < prepared_end else prepared_end

def scan_index():
    global sessions
    n = 0
    while n < MAX_SESSIONS:
        e = mem_read(idx_addr(n), 4)
        start = e[0] | (e[1] << 8) | (e[2] << 16) | (e[3] << 24)
        if start == 0xFFFFFFFF or start >= LOG_END:
            break
        n += 1
    sessions = n

# kind: what the session was recorded for. The two reserved bytes of the
# index entry carry it, so nothing about the record format changes.
KIND_MEAS = const(0)
KIND_CAL  = const(1)

# Latitude and longitude are carried as degrees x 1e7 in a signed 32-bit
# field: +/-90 and +/-180 both fit comfortably, and it keeps ~1 cm of
# resolution without floating point anywhere in the firmware.
IDX_FMT = '<IIhHHHiiHH32s'

def write_index_entry(epoch, tz_min, kind, lat_e7, lon_e7, acc_m, gps_ok,
                      note=b''):
    """Recorded when a session starts, at the top of memory rather than
    inline with the samples, so the data region stays pure records."""
    global sessions
    # scan_index() stops at MAX_SESSIONS, and the reservation above
    # REC_LIMIT holds exactly that many: an entry past it would be lost at the
    # next boot and would sit where records are allowed to go
    if sessions >= MAX_SESSIONS:
        print('session index full (%d sessions) - erase memory first' % MAX_SESSIONS)
        return False
    a = idx_addr(sessions)
    e = struct.pack(IDX_FMT, log_ptr, epoch, tz_min, sample_ms, sessions,
                    kind, lat_e7, lon_e7, acc_m, 1 if gps_ok else 0,
                    note[:NOTE_LEN])
    flash_guard_enter()
    try:
        # Same guard the record path has: programming a cell that is not
        # erased ANDs the two values together and quietly corrupts both.
        if not is_erased(a, IDX_SIZE):
            print('index slot 0x%06X is not erased - erase memory first' % a)
            return False
        mem_write(a, e + b'\x00' * (IDX_SIZE - len(e)))
        flash_sync()        # finish the program before SCL is handed back
    finally:
        flash_guard_exit()
    sessions += 1
    return True

def session_info(i):
    e = mem_read(idx_addr(i), IDX_SIZE)
    n = struct.calcsize(IDX_FMT)
    start, epoch, tz, sms, seq, kind, lat, lon, acc, gps, note = \
        struct.unpack(IDX_FMT, e[:n])
    if kind > 1:
        kind = KIND_MEAS
    if i + 1 < sessions:
        e2 = mem_read(idx_addr(i + 1), 4)
        nxt = e2[0] | (e2[1] << 8) | (e2[2] << 16) | (e2[3] << 24)
    else:
        nxt = log_ptr
    count = (nxt - start) // REC_SIZE if nxt > start else 0
    return start, epoch, tz, sms, seq, count, kind, lat, lon, acc, gps, note

def send_index():
    global dumping
    print('send_index: %d session(s)' % sessions)
    dumping = True          # also suppresses live streaming, so sample
                            # fragments cannot interleave with index fragments
    flash_guard_enter()
    try:
        for i in range(sessions):
            start, epoch, tz, sms, seq, count, kind, lat, lon, acc, gps, note = \
                session_info(i)
            send_frame(h_data, struct.pack('<BHIIhHIBiiHB32s', 0x50, seq, start,
                                           epoch, tz, sms, count, kind,
                                           lat, lon, acc, gps, note))
    finally:
        # end marker and flag reset even on a failure, so neither the app nor
        # the live stream is left waiting on a transfer that already died
        flash_guard_exit()
        send_frame(h_data, b'\x51')
        dumping = False
    send_status()

def send_config():
    """Send this board's config.json to the app, byte for byte.

    Lets the app show calibrated data -- and keep the device ID -- without
    the file being copied off the board by hand. Frames:
      0x60 seq chunk...   the file in pieces of up to 200 bytes
      0x61 <length:u16> <checksum:u16>   end: the file's total length and
                          the sum of its bytes, so the app can tell a
                          complete file from a damaged one. A length of 0
                          means the board has no config.json.
    Read fresh from the filesystem each time, so a newly copied file is sent
    even before the board restarts to use it."""
    global dumping
    try:
        with open(CONFIG_FILE, 'rb') as f:
            data = f.read(4097)
    except OSError:
        data = b''
    if len(data) > 4096:            # not a config file this board wrote or reads
        print('send_config: config.json too large to send')
        data = b''
    print('send_config: %d bytes' % len(data))
    dumping = True                  # no live samples interleaved with the pieces
    try:
        i = 0
        seq = 0
        while i < len(data):
            chunk = data[i:i + 200]
            if not send_frame(h_data, bytes([0x60, seq & 0xFF]) + chunk):
                print('send_config: link stalled')
                break
            i += len(chunk)
            seq += 1
        send_frame(h_data, struct.pack('<BHH', 0x61, len(data), sum(data) & 0xFFFF))
    finally:
        dumping = False
    send_status()

def write_header(prep):
    mem_erase_sector(HDR_ADDR)
    mem_write(HDR_ADDR, LOG_MAGIC + bytes([
        (prep >> 24) & 0xFF, (prep >> 16) & 0xFF,
        (prep >> 8) & 0xFF, prep & 0xFF]))

def region_blank():
    """Is the whole log region genuinely erased?

    This decides whether a chip with no header is virgin, and getting it
    wrong sets prepared_end to LOG_END over data that is still there --
    after which the first write AND-corrupts silently. So it has to be a
    real check, not a sample.

    The old version read 16 bytes every 64 kB and looked at only the last
    4 kB of the index, which is 64 of 4096 slots. Junk written to slot 199
    passed. Both regions are now walked in full, in 1 kB blocks compared
    against a constant so the work happens in C.
    """
    a = DATA_START
    while a < LOG_END:
        n = LOG_END - a
        if n > 1024:
            n = 1024
        if n == 1024:
            mem_read_into(a, _VBUF)
            if _VBUF != _VBLANK:
                return False
        elif mem_read(a, n) != _VBLANK[:n]:
            return False
        a += n
    return True

def log_init():
    """Establish, once at boot, how much of the log is safe to write.

    Erasing per sector while logging costs 45 ms typical and up to 400 ms, and
    at high rates that does not fit between samples. Instead the whole region
    is erased ahead of a session and the erased extent recorded here, so the
    write path never erases and never reads back."""
    global log_ptr, log_foreign, prepared_end, sessions
    hdr = mem_read(HDR_ADDR, len(LOG_MAGIC) + 4)
    if hdr[:len(LOG_MAGIC)] == LOG_MAGIC:
        d = hdr[len(LOG_MAGIC):]
        prepared_end = (d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]
        if prepared_end < DATA_START or prepared_end > LOG_END:
            prepared_end = 0
        log_foreign = False
        if prepared_end == 0:
            # only an erase writes this, so the erase was cut short: whatever
            # is left in the log is not a recording, and nothing can be
            # written until a full erase
            log_ptr = DATA_START
            sessions = 0
            print('log: an erase did not finish - erase again before recording')
        else:
            log_ptr = find_log_end()
            scan_index()
    elif is_erased(HDR_ADDR, 64) and region_blank():
        prepared_end = LOG_END          # virgin chip, nothing to erase
        write_header(prepared_end)
        log_foreign = False
        log_ptr = DATA_START
        sessions = 0
    else:
        log_foreign = True              # someone else's data, or another layout
        prepared_end = 0
        log_ptr = DATA_START
        sessions = 0

# [0x02][addr:3][record:32], reused for every record so the hot path does
# not allocate. A record never crosses a 256-byte page: log_ptr is always a
# multiple of REC_SIZE, which divides the page size.
_PROG = bytearray(4 + REC_SIZE)
_PROG[0] = 0x02

def log_write(rec):
    # Hot path: one bounds check and one page program. No erase, no read-back.
    global log_ptr, _flash_busy
    if log_ptr + REC_SIZE > data_limit():
        return False
    a = log_ptr
    _PROG[1] = (a >> 16) & 0xFF
    _PROG[2] = (a >> 8) & 0xFF
    _PROG[3] = a & 0xFF
    _PROG[4:] = rec
    flash_guard_enter()
    try:
        flash_sync()                    # the previous program, if any
        mem_write_enable()
        spi_flash()
        cs_mag.value(1)
        cs_mem.value(0)
        spi.write(_PROG)
        cs_mem.value(1)
        _flash_busy = True              # deliberately not waited on here
    finally:
        flash_guard_exit()
    log_ptr += REC_SIZE
    return True

def log_erase_all():
    """Erase the whole region and record that it is erased.

    256 block erases at 150 ms typical is close to 40 seconds, and the
    datasheet maximum is 2 s per block, so progress is reported as it goes
    and the operation can be aborted. Silence for that long is
    indistinguishable from a crash."""
    global log_ptr, log_foreign, prepared_end, erasing, erase_pct, abort_dump, sessions
    erasing = True
    abort_dump = False
    erase_pct = 0
    prepared_end = 0
    # The old sessions are gone from this moment, whether or not the erase
    # completes: an abort must not leave the app listing sessions whose
    # records have already been wiped.
    sessions = 0
    log_ptr = DATA_START
    total = FLASH_END // 0x10000
    done = 0
    aborted = False
    # One guard for the whole erase. SCL sits low for up to 40 s, which is
    # fine: no I2C runs during an erase, and slaves resynchronise afterwards.
    flash_guard_enter()
    try:
        # Erase from the top down. The header and the session index occupy
        # the top 260 kB (index 0xFBF000-0xFFEFFF across five 64 kB blocks,
        # header in the last sector), so they go first: erased last, an abort
        # or a power cut left the index describing records that were already
        # gone, and the next boot believed it. The header is rewritten at
        # once saying nothing is safe to write, and a board in that state
        # ignores the index and refuses to record until a full erase.
        # Every block is erased unconditionally: probing a few offsets to
        # skip blank-looking blocks once spared the index and header blocks,
        # because neither sat at a probe point. Sampling is not verification.
        a = FLASH_END - 0x10000
        mem_erase_block64(a)
        done = 1
        write_header(0)
        while a > 0:
            a -= 0x10000
            mem_erase_block64(a)
            done += 1
            erase_pct = done * 100 // total
            if done % 2 == 0 or done == total:
                send_frame(h_data, struct.pack('<BBHH', 0x31, erase_pct, done, total))
                drain_commands(dump_ctx=True)
                if abort_dump:
                    aborted = True
                    break
        if not aborted:
            prepared_end = LOG_END
            write_header(prepared_end)
            log_foreign = False
            erase_pct = 100
    finally:
        # the end frame goes out even if the erase failed, with the
        # percentage reached, so the app is not left waiting for it
        erasing = False
        flash_guard_exit()
        send_frame(h_data, struct.pack('<BBHH', 0x32, erase_pct, done, total))
    send_status()
    print('erase complete' if not aborted else 'erase aborted at %d%%' % erase_pct)

def flash_probe():
    """One read, guarded: the health check for the flash chip."""
    flash_guard_enter()
    try:
        mem_read(0, 4)
    finally:
        flash_guard_exit()

def log_count():
    return (log_ptr - DATA_START) // REC_SIZE


# ─── SENSORS ───────────────────────────────────────────────────────────
def init_imu():
    # 104 Hz rather than 416: the analogue chain bandwidth follows the ODR,
    # so a lower ODR is itself an anti-alias filter ahead of the decimation
    # done in the main loop.
    #
    # CTRL3_C = 0x44: BLOCK DATA UPDATE on, register auto-increment on.
    #
    # Without BDU the sensor refreshes its output registers whenever a new
    # sample is ready, even halfway through a burst read -- so a 16-bit
    # value can come back as the low byte of one sample and the high byte
    # of the next. That produces huge single-sample spikes on ANY IMU
    # channel: gyro, accel and die temperature alike, while the
    # magnetometer on SPI stays clean. The die temperature showed it most
    # plainly, jumping 3-6 C between samples 38 ms apart, which no silicon
    # can do. BDU holds the registers until both bytes have been read.
    i2c.writeto_mem(IMU_ADDR, 0x12, b'\x44')   # BDU + IF_INC
    i2c.writeto_mem(IMU_ADDR, 0x10, b'\x40')   # XL 104 Hz, +/-2 g
    i2c.writeto_mem(IMU_ADDR, 0x11, b'\x40')   # G  104 Hz, +/-245 dps
    c3 = i2c.readfrom_mem(IMU_ADDR, 0x12, 1)[0]
    if c3 & 0x44 != 0x44:
        raise OSError('IMU CTRL3_C reads 0x%02X -- BDU did not stick' % c3)

def init_baro():
    """Read the seven PROM coefficients AND prove a conversion works.

    Built in a local list and only published on success. The old version
    appended straight into the global, so a read that failed on the fourth
    word left C with three entries -- and baro_compute indexes C[1]..C[6]
    unconditionally, so the barometer would flap between working and
    raising IndexError instead of failing cleanly.
    """
    global C
    i2c.writeto(MS_ADDR, b'\x1E')
    time.sleep_ms(10)
    tmp = []
    for i in range(7):
        d = i2c.readfrom_mem(MS_ADDR, 0xA0 + i * 2, 2)
        tmp.append((d[0] << 8) | d[1])
    # a PROM that reads all zeros or all ones is not a PROM that read
    if tmp[1] == 0 or tmp[1] == 0xFFFF:
        raise OSError('MS58xx PROM reads 0x%04X -- not a valid coefficient'
                      % tmp[1])

    # Prove a CONVERSION works, not just the PROM.
    #
    # Reading calibration memory and running the ADC are different parts of
    # the chip, and one can work while the other does not. Observed on a
    # real sensor: reset and all seven PROM words read in under a
    # millisecond each, while every conversion command (0x40-0x58) either
    # NAKed or hung the bus for up to 205 ms. init_baro reported "ok", the
    # sample loop then failed on the first read and cleared HEALTH_BARO,
    # and retry_failed re-proved the PROM every 5 seconds -- printing
    # "barometer ok" forever for a sensor that could not produce a reading.
    #
    # One conversion at the fastest OSR costs about 1.2 ms at startup and
    # makes safe_init tell the truth.
    i2c.writeto(MS_ADDR, b'\x40')          # D1, OSR 256: 0.6 ms
    time.sleep_us(1200)
    i2c.writeto(MS_ADDR, b'\x00')          # read ADC
    r = i2c.readfrom(MS_ADDR, 3)
    raw = (r[0] << 16) | (r[1] << 8) | r[2]
    if raw == 0 or raw == 0xFFFFFF:
        raise OSError('MS58xx conversion returned %d -- the ADC is not '
                      'working even though the PROM reads fine' % raw)

    C = tmp

def mag_reg(reg, val):
    spi_mag()
    cs_mem.value(1)
    cs_mag.value(0)
    spi.write(bytes([reg, val]))
    cs_mag.value(1)

def init_mag():
    # The continuous-measurement rate register (TMRC) is deliberately left
    # alone. Writing 0x0B stopped this part producing data entirely, so the
    # address is either wrong or the value is one it will not accept. Check
    # the datasheet for your revision before touching it: at the default the
    # part runs near 37 Hz, which is why samples repeat above that rate.
    #
    # The write is verified by reading it back, because it can silently fail.
    # Observed on a real board: CMM read back 0x00 after init, the part never
    # measured, DRDY never set, and every sample was skipped -- while init
    # itself raised nothing, so the health flag cheerfully reported "ok".
    # The first SPI transaction after the bus is configured is also unreliable
    # on this hardware, which is the likely reason the write is lost.
    mag_read_reg(0x00)                         # throwaway: wake the bus up

    last = None
    for attempt in range(5):
        mag_reg(0x01, 0x00)                    # stop continuous mode first, so
        time.sleep_ms(5)                       # settings do not change mid-run
        for reg in (0x04, 0x06, 0x08):         # cycle counts = 200 -> 75 LSB/uT
            mag_reg(reg, 0x00)
            mag_reg(reg + 1, 0xC8)
        mag_reg(0x01, 0x79)                    # continuous measurement, all axes
        time.sleep_ms(5)
        last = mag_read_reg(0x01)
        # Mask, do not compare. The part sets bit 1 itself once continuous
        # mode is actually running, so a healthy sensor reads back 0x7B for
        # the 0x79 that was written. An earlier version of this check tested
        # for equality and therefore rejected every working magnetometer,
        # marked it down, and filled the log with sentinels.
        if (last & 0x79) == 0x79:
            if attempt:
                print('  (magnetometer CMM took on attempt %d)' % (attempt + 1))
            return
    raise OSError('RM3100 CMM would not stick: read back 0x%02X, '
                  'wanted the 0x79 bits set'
                  % (last if last is not None else 0xFF))

def mag_read_reg(reg):
    spi_mag()
    cs_mem.value(1)
    cs_mag.value(0)
    spi.write(bytes([reg | 0x80]))
    d = spi.read(1)
    cs_mag.value(1)
    return d[0]

def mag_diagnose():
    """REVID should read 0x22 on an RM3100. If it does, SPI is fine and the
    problem is configuration; 0x00 or 0xFF means the part is not responding
    at all and nothing downstream can work."""
    rev = mag_read_reg(0x36)
    cmm = mag_read_reg(0x01)
    stat = mag_read_reg(0x34)
    cc = (mag_read_reg(0x04) << 8) | mag_read_reg(0x05)
    print('RM3100  revid=0x%02X (expect 0x22)  cmm=0x%02X  status=0x%02X  cc=%d'
          % (rev, cmm, stat, cc))
    return rev

def read_mag_blocking(timeout_ms=300):
    """Used once at startup to seed the last-known value. Everything in the
    sample loop uses the non-blocking form."""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        m = read_mag()
        if m is not None:
            return m
        time.sleep_ms(5)
    return None

_IMU_BUF = bytearray(14)

def imu_accumulate(acc):
    """read_imu() for the oversampling loop: the same burst, read into a
    preallocated buffer and added straight into the running sums, so each of
    the several reads per sample allocates nothing."""
    i2c.readfrom_mem_into(IMU_ADDR, 0x20, _IMU_BUF)
    b = _IMU_BUF
    for i in range(7):
        v = b[2 * i] | (b[2 * i + 1] << 8)
        acc[i] += v - 65536 if v > 32767 else v

def read_imu():
    """One burst: temperature, gyro, accel -- all int16 little-endian.

    OUT_TEMP sits at 0x20, immediately before the gyro registers, so
    starting the burst two bytes earlier picks up the die temperature for
    45 us of extra I2C. That temperature is what the gyro bias tracks, and
    the barometer's is a different die entirely.
    """
    d = i2c.readfrom_mem(IMU_ADDR, 0x20, 14)
    return struct.unpack('<hhhhhhh', d)   # temp, gx, gy, gz, ax, ay, az

def baro_compute(D1, D2):
    if len(C) < 7:
        raise OSError('barometer PROM not loaded')
    # 0 is what the ADC reads when no conversion ran (a command lost, or a
    # read too early). Computing from it gives a plausible-looking but wrong
    # pressure, so it is a failed read, the same test init_baro() applies.
    if D1 == 0 or D2 == 0:
        raise OSError('MS58xx ADC read 0: no conversion')
    dT = D2 - C[5] * 256
    TEMP = 2000 + (dT * C[6]) // 8388608
    OFF = C[2] * 65536 + (C[4] * dT) // 128
    SENS = C[1] * 32768 + (C[3] * dT) // 256
    P = ((D1 * SENS // 2097152) - OFF) // BARO_P_DIV
    return P, TEMP

# Command bytes and the ADC buffer are preallocated: these run on every
# sample, and bytes([cmd]) / readfrom() each allocated a new object.
_BARO_CMD = {c: bytes([c]) for c in (0x40, 0x42, 0x44, 0x46, 0x48, 0x4A,
                                     0x50, 0x52, 0x54, 0x56, 0x58, 0x5A)}
_BARO_ADC = bytearray(3)

def baro_fetch():
    i2c.writeto(MS_ADDR, b'\x00')
    i2c.readfrom_into(MS_ADDR, _BARO_ADC)
    r = _BARO_ADC
    return (r[0] << 16) | (r[1] << 8) | r[2]

def read_baro():
    """Blocking read: start a conversion, wait, collect. Twice.

    This used to cost 27 ms because time.sleep_ms rounds up to the scheduler
    tick -- measured at 12.2 ms whether asked for 4 or 12. Waiting in
    microseconds busy-waits instead and takes the conversion time it actually
    needs, so the whole read is about 7 ms at OSR 1024 and pressure can be
    read fresh on every sample up to roughly 40 Hz."""
    i2c.writeto(MS_ADDR, _BARO_CMD[baro_d1])
    time.sleep_us(baro_wait_us)
    D1 = baro_fetch()
    i2c.writeto(MS_ADDR, _BARO_CMD[baro_d2])
    time.sleep_us(baro_wait_us)
    D2 = baro_fetch()
    return baro_compute(D1, D2)

def baro_step():
    """One tick's worth of barometer work, no blocking wait.

    Used when a blocking read would take too much of the sample period --
    above roughly 33 Hz, see set_period. Two states, not three: collect D1
    and start D2, then collect D2, compute, and start the next D1 in the
    same tick. Starting D1 immediately rather than idling a whole tick is
    what makes it two. Pressure updates at half the sample rate, still far
    faster than the atmosphere moves, and each tick costs one register
    access instead of a conversion wait."""
    global baro_state, baro_D1, last_press, last_temp
    if baro_state == BARO_UNPRIMED:
        # no conversion is running (after a rate change that could not prime,
        # or a failure): start one, and report no data until it has produced
        # a real reading rather than computing from an ADC that never ran
        last_press, last_temp = BAD32, BAD16
        i2c.writeto(MS_ADDR, _BARO_CMD[baro_d1])
        baro_state = 0
    elif baro_state == 0:
        baro_D1 = baro_fetch()                 # D1 started on the previous tick
        i2c.writeto(MS_ADDR, _BARO_CMD[baro_d2])
        baro_state = 1
    else:
        last_press, last_temp = baro_compute(baro_D1, baro_fetch())
        i2c.writeto(MS_ADDR, _BARO_CMD[baro_d1])  # start the next one now, rather
        baro_state = 0                          # than idling for a whole tick

# Preallocated so the read path does no allocation at all. _MAG_TX is the
# command byte followed by nine dummy bytes to clock the result out.
_MAG_TX = bytearray(b'\xA4' + b'\x00' * 9)
_MAG_RX = bytearray(10)
_STAT_TX = bytearray(b'\xB4\x00')
_STAT_RX = bytearray(2)

def mag_ready():
    spi_mag()
    cs_mem.value(1)
    cs_mag.value(0)
    spi.write_readinto(_STAT_TX, _STAT_RX)
    cs_mag.value(1)
    return bool(_STAT_RX[1] & 0x80)            # DRDY

def read_mag():
    """Read all three axes in one SPI transaction.

    DRDY is checked before the read so a stale sample is not returned twice.
    It is deliberately NOT checked afterwards: that was tried as a guard
    against torn reads and rejected every single sample, because the sensor
    runs faster than the loop and DRDY sets again almost immediately. The
    torn-read theory it was defending had already been disproved anyway --
    tearing can only produce errors of zero or a multiple of 256 counts,
    while the excursions seen in real data form a continuum.

    The single write_readinto is kept: it is faster than a write followed by
    a read, and allocates nothing.
    """
    if not mag_ready():
        return None                            # caller keeps the last good one
    spi_mag()
    cs_mem.value(1)
    cs_mag.value(0)
    spi.write_readinto(_MAG_TX, _MAG_RX)
    cs_mag.value(1)
    out = []
    for i in (1, 4, 7):
        v = (_MAG_RX[i] << 16) | (_MAG_RX[i + 1] << 8) | _MAG_RX[i + 2]
        out.append(v - 0x1000000 if v > 0x7FFFFF else v)
    return out

def read_battery_mv():
    # mean of 8 reads / 4095 * 3.3 V * 1.05 * 2 (divider) * 1000, in integers
    # so a reading allocates nothing: 3300 * 1.05 * 2 / (8 * 4095) = 6930 / 32760
    t = 0
    for _ in range(8):
        t += bat_adc.read()
    return t * 6930 // 32760


# ─── BLE ───────────────────────────────────────────────────────────────
_IRQ_CONNECT = const(1)
_IRQ_DISCONNECT = const(2)
_IRQ_WRITE = const(3)
_IRQ_MTU = const(21)

SVC_UUID = bluetooth.UUID('12345678-1234-5678-1234-56789abcdef0')
DATA_UUID = bluetooth.UUID('12345678-1234-5678-1234-56789abcdef1')
CTRL_UUID = bluetooth.UUID('12345678-1234-5678-1234-56789abcdef2')
STAT_UUID = bluetooth.UUID('12345678-1234-5678-1234-56789abcdef3')

_NOTIFY = const(0x0010)
_READ = const(0x0002)
_WRITE = const(0x0008)
_WRITE_NR = const(0x0004)

ble = bluetooth.BLE()

def ble_start():
    """Switch the radio on, retrying after a clean switch-off.

    The EIO failures from ble.active(True) that were once blamed on soft
    reboots were a memory problem: the firmware built without PSRAM left
    too little RAM for the Bluetooth stack. On the BORELOGGER_S3-SPIRAM_OCT
    build the radio starts reliably, including after soft reboots with the
    radio left on (tested 2026-11-04). The switch-off-and-retry is kept as
    cheap insurance, not because a soft reboot is known to need it."""
    # Switch off first. On a fresh boot this does nothing; after a soft
    # reboot it clears whatever the previous run left switched on -- which
    # happens whenever a run is stopped during start-up, before the main
    # loop's KeyboardInterrupt handler exists to shut the radio down.
    try:
        ble.active(False)
        time.sleep_ms(100)
    except Exception:
        pass
    for attempt in range(3):
        try:
            ble.active(True)
            return
        except OSError as e:
            print('BLE start failed (%s), retrying' % e)
            try:
                ble.active(False)
            except Exception:
                pass
            time.sleep_ms(500)
    print('BLE will not start -- press the reset button or replug USB')
    ble.active(True)            # one last try: raise the real error if it fails

ble_start()

try:
    ble.config(mtu=247)
except Exception:
    pass

((h_data, h_ctrl, h_stat),) = ble.gatts_register_services(((
    SVC_UUID, (
        (DATA_UUID, _NOTIFY),
        (CTRL_UUID, _WRITE | _WRITE_NR),
        (STAT_UUID, _READ | _NOTIFY),
    ),
),))
ble.gatts_set_buffer(h_data, 64)
ble.gatts_set_buffer(h_ctrl, 64)
ble.gatts_set_buffer(h_stat, 32)

conns = set()
mtu = 23
pending = []          # commands queued from the IRQ, drained by the main loop
PENDING_MAX = const(32)
want_adv = False

# Advertised AND reported name: ESP32-LOG-<id> once config.json gives the
# board an id, so several boards can be told apart in the connection list.
# Longest possible, ESP32-LOG-65535, uses 20 of the advertisement's 31 bytes.
NAME = b'ESP32-LOG' if DEVICE_ID is None else ('ESP32-LOG-%d' % DEVICE_ID).encode()

def build_adv():
    return bytes([2, 0x01, 0x06, len(NAME) + 1, 0x09]) + NAME

def build_scan_resp():
    # A 128-bit UUID goes into the advertisement least-significant byte first.
    le = bytes(reversed(bytes.fromhex('1234567812345678123456789abcdef0')))
    return bytes([17, 0x07]) + le

def advertise():
    # The GAP name is what a phone or computer reports AFTER connecting, and
    # may cache; left unset it is MicroPython's 'MPY ESP32'. Match it to NAME.
    try:
        ble.config(gap_name=NAME)
    except Exception:
        pass
    try:
        ble.gap_advertise(None)
        time.sleep_ms(50)
        ble.gap_advertise(ADV_INTERVAL_MS * 1000, adv_data=build_adv(),
                          resp_data=build_scan_resp())
    except Exception as e:
        print('adv error:', e)

def ble_irq(event, data):
    global mtu, want_adv
    if event == _IRQ_CONNECT:
        conns.add(data[0])
        # a new link starts at the default ATT MTU until it negotiates;
        # keeping the previous client's value would send frames too large
        # for this one, which the stack then truncates
        mtu = 23
    elif event == _IRQ_DISCONNECT:
        conns.discard(data[0])
        want_adv = True
    elif event == _IRQ_WRITE:
        if data[1] == h_ctrl:
            v = ble.gatts_read(h_ctrl)
            # bounded, so a client flooding writes during a long erase cannot
            # grow the queue without limit; 0x06 (abort) is always accepted
            if v and (len(pending) < PENDING_MAX or v[0] == 0x06):
                pending.append(bytes(v))
    elif event == _IRQ_MTU:
        mtu = data[1]

ble.irq(ble_irq)

def _notify(handle, payload, wait=True):
    ok = False
    tries = 6 if wait else 1
    for c in tuple(conns):
        for i in range(tries):
            try:
                ble.gatts_notify(c, handle, payload)
                ok = True
                break
            except OSError:
                if not wait:
                    break
                time.sleep_ms(10)     # stack buffer full, let the link drain
    return ok

def send_frame(handle, frame, wait=True):
    """Split a frame across notifications when it exceeds the MTU.

    The first fragment carries the total length. Without it the receiver has
    no way to know when a frame is complete, and frames of any size other
    than the one it happens to assume are silently discarded."""
    room = mtu - 3
    total = len(frame)
    if total <= room:
        return _notify(handle, frame, wait)
    if total > 255:
        return False
    i = 0
    first = True
    while i < total:
        if first:
            n = room - 2
            pkt = bytes([0xF1, total]) + frame[i:i + n]
        else:
            n = room - 1
            pkt = bytes([0xF2]) + frame[i:i + n]
        if not _notify(handle, pkt, wait):
            return False
        i += n
        first = False
    return True


# ─── STATE ─────────────────────────────────────────────────────────────
recording = False
dumping = False
abort_dump = False
verifying = False
erasing = False
erase_pct = 0
stream_on = True        # BLE live streaming
stream_ms = DEFAULT_PERIOD_MS   # requested period: every sample by default
# While True the stream stays at the sample rate (every sample, up to the
# 50 Hz BLE cap) through rate changes. A Stream command asking for a slower
# rate turns it off; asking for the sample rate or faster turns it back on.
stream_follow = True
stream_div = 1          # send every Nth sample
stream_count = 0
actual_ms = 0           # smoothed real loop period, so a shortfall is visible
prof = [0, 0, 0, 0, 0, 0]
prof_n = 0
PROF_NAMES = ('imu', 'baro', 'mag', 'batt', 'flash', 'ble')
led_until = 0
led_ms = 60
sample_count = 0
errors = 0              # consecutive loop failures; drives sensor re-init
errors_total = 0        # every loop failure since boot, as the status reports
t0 = time.ticks_ms()
t_prev = t0
t_ms_now = 0

def recalc_led():
    """Scale the blink to the sample period. A fixed short pulse is invisible
    at low rates and a fixed long one merges into a solid glow at high ones,
    so aim for roughly a quarter of the period."""
    global led_ms
    v = sample_ms // 4
    if v < LED_MIN_MS:
        v = LED_MIN_MS
    elif v > LED_MAX_MS:
        v = LED_MAX_MS
    led_ms = v

def recalc_stream():
    """Frames can only be sent on sample instants, so the reachable stream
    rates are sample_rate / N. Round to the nearest whole divisor and report
    the period actually achieved -- reporting the requested one instead makes
    the app claim a rate the hardware never delivers."""
    global stream_div, stream_count
    d = (stream_ms + sample_ms // 2) // sample_ms
    if d < 1:
        d = 1
    stream_div = d
    stream_count = 0

def effective_stream_ms():
    return stream_div * sample_ms

def set_stream(on, ms):
    global stream_on, stream_ms, stream_follow
    stream_on = bool(on)
    stream_ms = max(20, min(60000, ms))
    stream_follow = ms <= sample_ms
    recalc_stream()
    eff = effective_stream_ms()
    print('ble streaming %s: asked %.2f Hz, sending %.2f Hz (every %d samples)' %
          ('on' if stream_on else 'off', 1000.0 / stream_ms, 1000.0 / eff, stream_div))
    send_status()

def set_period(ms):
    """Change the sampling period. Clamped, and the barometer OSR follows,
    because two conversion waits are the largest fixed cost in the loop."""
    global sample_ms, next_tick, actual_ms
    global baro_d1, baro_d2, baro_async, baro_state, baro_wait_us, stream_ms
    sample_ms = max(MIN_PERIOD_MS, min(MAX_PERIOD_MS, ms))
    actual_ms = 0            # the old rate's average says nothing about this one
    if sample_ms >= 200:
        baro_d1, baro_d2 = 0x48, 0x58                  # OSR 4096
    elif sample_ms >= 100:
        baro_d1, baro_d2 = 0x46, 0x56                  # OSR 2048
    else:
        baro_d1, baro_d2 = 0x44, 0x54                  # OSR 1024
    # Blocking used to cost 27 ms because sleep_ms rounds to the scheduler
    # tick; sleeping in microseconds instead brings it down to about two
    # conversions plus I2C. That is worth paying while it stays a small
    # part of the period, which holds up to about 33 Hz.
    baro_wait_us = int(BARO_CONV_US[baro_d1] * 1.25)   # 25% margin
    # Blocking now costs about two conversions plus I2C, so it is worth
    # paying whenever that is a small part of the period. Only above ~40 Hz
    # does it stop fitting.
    was_async = baro_async
    baro_async = (2 * baro_wait_us / 1000.0 + 2.0) > sample_ms * 0.25
    if was_async and not baro_async:
        # a stepped conversion may still be running; let it finish and
        # collect it, or the first blocking read meets a busy ADC
        try:
            time.sleep_us(BARO_CONV_US[0x48] + 1000)
            baro_fetch()
        except Exception:
            pass
    baro_state = 0           # not used in blocking mode
    if baro_async:
        baro_state = BARO_UNPRIMED
        # Prime the cycle. A barometer that is down must not stop the rate
        # change half-way: this used to raise out of set_period, leaving the
        # stream divisor, LED length and status for the old rate.
        try:
            i2c.writeto(MS_ADDR, _BARO_CMD[baro_d1])
            baro_state = 0
        except Exception:
            pass
    next_tick = time.ticks_ms()
    if stream_follow:
        stream_ms = max(20, sample_ms)   # keep streaming every sample (50 Hz cap)
    recalc_stream()          # the divisor depends on the sample period
    recalc_led()
    print('rate set to %.3f Hz (%d ms)' % (1000.0 / sample_ms, sample_ms))
    send_status()

def send_status():
    # Always built, even with nobody connected: STAT is a readable
    # characteristic, and a client that reads it straight after connecting
    # must find the current status there rather than an empty value. The
    # frame is built at most every few seconds, so this costs nothing.
    flags = ((1 if recording else 0) | (2 if dumping else 0) |
             (4 if log_foreign else 0) | (8 if stream_on else 0) |
             (16 if verifying else 0) | (32 if erasing else 0) |
             (64 if BRIDGE_WORKAROUND else 0))      # bridge workaround active
    used_kb = (log_ptr - DATA_START) // 1024
    cap_kb = (data_limit() - DATA_START) // 1024 if prepared_end else 0
    frame = struct.pack('<BBIIIHHHHHHB', 0x10, flags, log_count(),
                        used_kb, cap_kb, last_mv, min(errors_total, 65535), sample_ms,
                        # uint16 on the wire, and MicroPython's pack wraps
                        # silently: a 40 s sample with a 60 s stream is 80 s
                        min(effective_stream_ms(), 65535), sessions,
                        min(int(actual_ms), 65535) if actual_ms else sample_ms,
                        health)
    try:
        ble.gatts_write(h_stat, frame)
    except Exception:
        pass
    if conns:
        send_frame(h_stat, frame)

def verify_blank():
    """Walk the entire log region and confirm every byte is 0xFF. Compares
    whole 1 kB blocks against a constant, so the work happens in C rather
    than a Python loop over 16 million bytes."""
    global verifying, abort_dump
    verifying = True
    abort_dump = False      # a stale abort from an earlier dump must not stop this one
    send_status()
    print('verifying blank...')
    flash_guard_enter()
    first_bad = 0
    ok = 0                  # only a walk that ends without an error is 'blank'
    a = DATA_START
    n = 0
    total = (LOG_END - DATA_START + 1023) // 1024
    try:
        clean = 1
        while a < LOG_END:
            chunk = LOG_END - a
            if chunk > 1024:
                chunk = 1024
            if chunk == 1024:
                mem_read_into(a, _VBUF)
                bad = _VBUF != _VBLANK
            else:
                bad = mem_read(a, chunk) != _VBLANK[:chunk]
            if bad:
                clean = 0
                first_bad = a
                break
            a += chunk
            n += 1
            if n % 256 == 0:
                send_frame(h_data, struct.pack('<BBHH', 0x33,
                                               n * 100 // total, n, total))
                drain_commands(dump_ctx=True)
                if abort_dump:
                    break
        # an aborted walk proves nothing about the part it did not reach
        ok = clean if a >= LOG_END else 0
    finally:
        flash_guard_exit()
        send_frame(h_data, struct.pack('<BBII', 0x30, ok, first_bad,
                                       (a - DATA_START) // 1024))
        verifying = False
    send_status()
    if ok:
        print('verify done: blank')
    elif a < LOG_END and not first_bad and abort_dump:
        print('verify aborted at 0x%06X - nothing proven' % a)
    else:
        print('verify done: data at 0x%06X' % first_bad)

def send_mem_range(addr, length):
    """Stream raw flash bytes, whatever they contain. The record dump only
    works on a log this firmware wrote; this works on anything, which is what
    you need when the question is 'what is actually in there'."""
    global dumping, abort_dump
    if addr < 0 or addr >= FLASH_END:
        # still close the transfer, or the app waits for an end that never comes
        print('raw read refused: 0x%X is beyond the chip' % addr)
        send_frame(h_data, struct.pack('<BI', 0x41, addr & 0xFFFFFFFF))
        send_status()
        return
    if addr + length > FLASH_END:
        length = FLASH_END - addr
    dumping = True
    abort_dump = False
    flash_guard_enter()
    a = addr
    try:
        room = mtu - 3
        step = room - 5                      # 1 type byte + 4 address bytes
        if step > 192:
            step = 192
        if step < 8:
            step = 8
        a = addr
        end = addr + length
        while a < end:
            if abort_dump or not conns:
                break
            n = end - a
            if n > step:
                n = step
            if not send_frame(h_data, struct.pack('<BI', 0x40, a) + mem_read(a, n)):
                break
            a += n
            drain_commands(dump_ctx=True)
    finally:
        flash_guard_exit()
        send_frame(h_data, struct.pack('<BI', 0x41, a))
        dumping = False
    send_status()

def do_dump(start=DATA_START, end=None):
    global dumping, abort_dump
    dumping = True
    abort_dump = False
    send_status()
    flash_guard_enter()
    sent = 0
    stalled = False
    try:
        stop = log_ptr if end is None else (end if end < log_ptr else log_ptr)

        # Pack as many records into one notification as the MTU allows.
        #
        # One record per packet used 33 of the 244 payload bytes available at
        # MTU 247 and threw the other 211 away. The link is limited by packets
        # per second, not bytes per second -- 200 records/s is about 6 kB/s on a
        # link that manages 60-100 -- so the waste was the whole bottleneck.
        #
        # Frame: [0x22][count][record * count]
        per = (mtu - 3 - 2) // REC_SIZE
        if per < 1:
            per = 1                     # tiny MTU: send_frame will fragment
        elif per > 32:
            per = 32                    # count is one byte, and 1 kB is plenty

        p = start
        while True:
            if p + REC_SIZE > stop:
                break
            if abort_dump or not conns:
                break
            n = (stop - p) // REC_SIZE
            if n > per:
                n = per
            blob = mem_read(p, n * REC_SIZE)      # one SPI burst, not n of them
            # a blank record means the end of the log; send whatever preceded it
            blank = -1
            for k in range(n):
                if blob[k * REC_SIZE:k * REC_SIZE + 4] == b'\xff\xff\xff\xff':
                    blank = k
                    break
            if blank == 0:
                break
            if blank > 0:
                n = blank
                blob = blob[:n * REC_SIZE]
            # A busy link is normal during a dump; giving up on it is not. Keep
            # retrying the same batch, backing off, rather than truncating the
            # transfer and reporting success.
            ok = False
            for attempt in range(20):
                if send_frame(h_data, bytes([0x22, n]) + blob):
                    ok = True
                    break
                time.sleep_ms(20 + attempt * 10)
                drain_commands(dump_ctx=True)
                if abort_dump or not conns:
                    break
            if not ok:
                stalled = True
                print('dump: link stalled after %d records' % sent)
                break
            sent += n
            p += n * REC_SIZE
            drain_commands(dump_ctx=True)
            if blank > 0:
                break
    finally:
        # the end marker carries the count, so the receiver can tell a
        # complete transfer from a truncated one -- sent even on a failure
        flash_guard_exit()
        send_frame(h_data, struct.pack('<BI', 0x21, sent))
        dumping = False
    if stalled:
        print('dump: incomplete')
    send_status()

# erase, dump, verify blank, range dump, raw read
_NOT_WHILE_RECORDING = (0x03, 0x04, 0x08, 0x0D, 0x0B)
# stop, status, rate, stream, index, config: safe to run after a long operation
_OK_AFTER_LONG_OP = (0x02, 0x05, 0x07, 0x09, 0x0C, 0x0E)

def drain_commands(dump_ctx=False):
    global recording, abort_dump
    if dump_ctx:
        # Inside a long operation only abort is acted on. Harmless requests
        # (stop, status, rate, stream, index, config) stay queued, in order,
        # and run when the operation ends. Anything else -- start, erase,
        # dump, verify, raw read -- was pressed while the board was busy and
        # the app has moved on since, so it is never run late: an erase
        # pressed during a dump would otherwise fire the moment it ended,
        # and a Start pressed during a 45 s erase would begin long after.
        keep = []
        dropped = False
        while pending:
            pkt = pending.pop(0)
            if pkt[0] == 0x06:
                abort_dump = True
            elif pkt[0] in _OK_AFTER_LONG_OP:
                keep.append(pkt)
            else:
                print('command 0x%02X dropped: busy' % pkt[0])
                dropped = True
        pending.extend(keep)
        if dropped and not any(k[0] == 0x05 for k in keep):
            pending.append(b'\x05')    # answer with a status once it is done
        return
    while pending:
        pkt = pending.pop(0)
        cmd = pkt[0]
        if recording and cmd in _NOT_WHILE_RECORDING:
            # these block the loop for seconds and would leave a hole in the
            # recording; say so, rather than leave the app waiting for an answer
            print('command 0x%02X refused: recording' % cmd)
            # A read also gets its normal end frame, reporting nothing sent,
            # so a client waiting for that frame (an app export, a script)
            # finishes at once instead of waiting for its own timeout.
            if cmd in (0x04, 0x0D):
                send_frame(h_data, struct.pack('<BI', 0x21, 0))
            elif cmd == 0x0B and len(pkt) >= 5:
                send_frame(h_data, struct.pack('<BI', 0x41,
                    pkt[1] | (pkt[2] << 8) | (pkt[3] << 16) | (pkt[4] << 24)))
            send_status()
            continue
        if cmd == 0x01:
            if recording:
                # an app retry or a double tap: a second index entry here
                # would split one survey into two sessions
                print('start ignored: already recording')
                send_status()
            elif log_foreign or log_ptr + REC_SIZE > data_limit():
                print('log not prepared - erase before recording')
                send_status()
            else:
                epoch = 0
                tz = 0
                kind = KIND_MEAS
                if len(pkt) >= 7:
                    epoch = (pkt[1] | (pkt[2] << 8) |
                             (pkt[3] << 16) | (pkt[4] << 24))
                    tz = pkt[5] | (pkt[6] << 8)
                    if tz > 32767:
                        tz -= 65536
                if len(pkt) >= 8:
                    kind = KIND_CAL if pkt[7] else KIND_MEAS
                lat = lon = 0
                acc = 0
                gps = 0
                if len(pkt) >= 19:
                    lat = (pkt[8] | (pkt[9] << 8) |
                           (pkt[10] << 16) | (pkt[11] << 24))
                    lon = (pkt[12] | (pkt[13] << 8) |
                           (pkt[14] << 16) | (pkt[15] << 24))
                    if lat > 0x7FFFFFFF:
                        lat -= 0x100000000
                    if lon > 0x7FFFFFFF:
                        lon -= 0x100000000
                    acc = pkt[16] | (pkt[17] << 8)
                    gps = pkt[18]
                note = bytes(pkt[19:19 + NOTE_LEN]) if len(pkt) > 19 else b''
                if not write_index_entry(epoch, tz, kind, lat, lon, acc, gps,
                                         note):
                    print('start refused')      # the reason is printed above
                    send_status()
                    continue
                # One deliberate collection now, before the first record. On
                # the 8 MB PSRAM heap a collection pauses the loop for ~200 ms
                # (5 samples at 26 Hz); starting with an empty heap, and
                # allocating little per sample, keeps one from landing in
                # the middle of a survey.
                gc.collect()
                recording = True
                print('recording %s, epoch %d, tz %d min, gps %s' %
                      ('CALIBRATION' if kind else 'measurement', epoch, tz,
                       ('%.6f,%.6f +/-%dm' % (lat / 1e7, lon / 1e7, acc))
                       if gps else 'unavailable'))
                if note:
                    try:
                        print('  note:', note.rstrip(b'\x00').decode())
                    except Exception:
                        pass
                send_status()
        elif cmd == 0x02:
            recording = False
            print('stopped')
            send_status()
        elif cmd == 0x03:
            print('erasing memory...')
            log_erase_all()
        elif cmd == 0x04:
            do_dump()
        elif cmd == 0x05:
            send_status()
        elif cmd == 0x07 and len(pkt) >= 3:
            if recording:
                # the session's index entry already states the period, so a
                # change now would make it describe the records wrongly
                print('rate change refused: recording')
                send_status()
            else:
                set_period(pkt[1] | (pkt[2] << 8))
        elif cmd == 0x08:
            verify_blank()
        elif cmd == 0x09 and len(pkt) >= 4:
            set_stream(pkt[1] != 0, pkt[2] | (pkt[3] << 8))
        elif cmd == 0x0C:
            # Sending the index means dozens of notifications with retries,
            # which stalls the sample loop for the best part of a second.
            # Never do that while samples are being recorded.
            if recording:
                print('index request deferred: recording')
                send_status()
            else:
                send_index()
        elif cmd == 0x0E:
            # like the index: dozens of notifications would stall sampling
            if recording:
                print('config request deferred: recording')
                send_status()
            else:
                send_config()
        elif cmd == 0x0D and len(pkt) >= 9:
            a = (pkt[1] | (pkt[2] << 8) | (pkt[3] << 16) | (pkt[4] << 24))
            c = (pkt[5] | (pkt[6] << 8) | (pkt[7] << 16) | (pkt[8] << 24))
            if (a - DATA_START) % REC_SIZE:
                # mid-record start: every "record" sent would straddle two
                print('range dump refused: 0x%06X is not a record boundary' % a)
                send_frame(h_data, struct.pack('<BI', 0x21, 0))
                send_status()
            else:
                do_dump(a, a + c * REC_SIZE)
        elif cmd == 0x0B and len(pkt) >= 7:
            a = pkt[1] | (pkt[2] << 8) | (pkt[3] << 16) | (pkt[4] << 24)
            n = pkt[5] | (pkt[6] << 8)
            send_mem_range(a, n)


# ─── SENSOR HEALTH ─────────────────────────────────────────────────────
# Any one sensor, or a whole bus, can fail without taking the logger with
# it. A dead I2C line used to raise out of init_imu() at startup and kill
# the script before BLE ever advertised -- so the board looked completely
# dead when only one pin was at fault.
#
# Each sensor is tracked independently. A failed one is retried
# periodically, so an intermittent fault recovers by itself.

HEALTH_IMU   = const(1)
HEALTH_BARO  = const(2)
HEALTH_MAG   = const(4)
HEALTH_FLASH = const(8)
HEALTH_BATT  = const(16)

health = 0                 # bitmask of what is currently working
RETRY_MS = const(5000)     # how often to retry a failed sensor
last_retry = 0

# Written into the record when a sensor is down, so the log says "no data"
# instead of silently repeating the last good value. Real int16 readings are
# clamped to -32767 (clamp16 for the magnetometer and barometer temperature,
# _real16 for the IMU -- an accelerometer saturated at -2 g would otherwise
# read exactly -32768), so -32768 always means 'down'. Pressure is always
# positive, so -1 is unambiguous there.
BAD16 = const(-32768)
BAD32 = const(-1)

def clamp16(v):
    """The magnetometer is a 24-bit part packed into an int16 field, and the
    barometer's temperature comes out of a compensation formula rather than a
    register. Either can exceed int16 -- a field over 437 uT, or one garbage
    read -- and MicroPython's struct.pack silently wraps an out-of-range
    value (40000 packs as -25536), so an over-range field would be logged as
    a plausible reading of the opposite sign. Clamping real readings to
    -32767 keeps -32768 reserved as the 'sensor down' marker, and that
    marker itself is passed through untouched so it reaches the log."""
    if v == BAD16:
        return v
    if v > 32767:
        return 32767
    if v < -32767:
        return -32767
    return v

def _real16(v):
    """A real int16 reading, kept clear of the BAD16 marker."""
    return -32767 if v == -32768 else v

def _ok(bit):
    return bool(health & bit)

def _mark(bit, good):
    global health
    if good:
        health |= bit
    else:
        health &= ~bit

def safe_init(bit, fn, name):
    """Run one sensor's init without letting it abort startup."""
    try:
        fn()
        _mark(bit, True)
        print('  %-12s ok' % name)
        return True
    except Exception as e:
        _mark(bit, False)
        print('  %-12s FAILED: %s' % (name, e))
        return False

def health_str():
    return '%s%s%s%s%s' % (
        'I' if _ok(HEALTH_IMU) else '-',
        'B' if _ok(HEALTH_BARO) else '-',
        'M' if _ok(HEALTH_MAG) else '-',
        'F' if _ok(HEALTH_FLASH) else '-',
        'V' if _ok(HEALTH_BATT) else '-')

def retry_failed():
    """Bring back anything that is currently down. Cheap when everything
    is working, which is the common case."""
    global last_retry
    if health == (HEALTH_IMU | HEALTH_BARO | HEALTH_MAG |
                  HEALTH_FLASH | HEALTH_BATT):
        return
    if time.ticks_diff(time.ticks_ms(), last_retry) < RETRY_MS:
        return
    last_retry = time.ticks_ms()
    # Both I2C sensors down while the SPI magnetometer works is the signature
    # of the GPIO4-GPIO5 short closing mid-run. Test for it before retrying
    # the sensors, so that if it is there they come straight back.
    if (not BRIDGE_WORKAROUND and not _ok(HEALTH_IMU) and not _ok(HEALTH_BARO)
            and _ok(HEALTH_MAG)):
        if bridge_recheck():
            print('*** BRIDGE DETECTED during the run (t=%.1fs): GPIO4 is shorted '
                  'to GPIO5; switched to the workaround' % (t_ms_now / 1000))
    if not _ok(HEALTH_IMU):
        safe_init(HEALTH_IMU, init_imu, 'IMU')
    if not _ok(HEALTH_BARO):
        safe_init(HEALTH_BARO, init_baro, 'barometer')
    if not _ok(HEALTH_MAG):
        safe_init(HEALTH_MAG, init_mag, 'magnetometer')
    if not _ok(HEALTH_FLASH):
        try:
            flash_probe()
            _mark(HEALTH_FLASH, True)
            print('  flash        recovered')
        except Exception:
            pass
    if not _ok(HEALTH_BATT):
        # the ADC has no init, so the probe IS the read. Without this the
        # battery channel would stay down for good after one bad read,
        # because the sample loop no longer retries it every tick.
        try:
            read_battery_mv()
            _mark(HEALTH_BATT, True)
            print('  battery ADC  recovered')
        except Exception:
            pass


# ─── STARTUP ───────────────────────────────────────────────────────────
print('=' * 52)
print('  ESP32 BLE data logger  |  %.1f Hz' % (1000.0 / sample_ms))
print('=' * 52)

# Advertise BEFORE touching any sensor. If a bus is dead, the board still
# comes up, still talks, and can still hand over whatever is already in
# flash -- which is exactly when you most need to reach it.
advertise()
print('advertising as', NAME.decode())
print('config:', CONFIG_STATUS, '| device id:', DEVICE_ID if DEVICE_ID is not None else 'not set')
print('calibration:', cal_summary(CONFIG), '-- stored only; raw data is recorded')

print('sensors:')
safe_init(HEALTH_IMU, init_imu, 'IMU')
safe_init(HEALTH_BARO, init_baro, 'barometer')
safe_init(HEALTH_MAG, init_mag, 'magnetometer')
time.sleep_ms(300)

try:
    flash_probe()
    _mark(HEALTH_FLASH, True)
    print('  flash        ok')
except Exception as e:
    print('  flash        FAILED: %s' % e)

try:
    last_mv = read_battery_mv()
    _mark(HEALTH_BATT, True)
except Exception as e:
    last_mv = 0
    print('  battery ADC  FAILED: %s' % e)

if _ok(HEALTH_MAG):
    try:
        mag_diagnose()
    except Exception as e:
        print('  mag_diagnose failed:', e)
    try:
        last_mag = read_mag_blocking()
    except Exception:
        last_mag = None
    if last_mag is None:
        print('WARNING: magnetometer produced no sample at startup')
    else:
        print('magnetometer seeded:', last_mag)
else:
    last_mag = None

# Everything that depends on the sample period -- barometer mode and OSR,
# stream divisor, LED length -- is derived here rather than duplicated in the
# module-level defaults, which silently went stale when the default changed.
# The first barometer conversion after start-up came back as garbage once
# (66,570 hPa on the very first record, normal from the second on). One
# throwaway read here absorbs it before anything can be recorded. Done
# BEFORE set_period(), because set_period primes the stepped read cycle
# and a blocking read afterwards would leave that cycle out of step.
if _ok(HEALTH_BARO):
    try:
        read_baro()
    except Exception:
        pass

set_period(DEFAULT_PERIOD_MS)
flash_guard_enter()
try:
    log_init()
finally:
    flash_guard_exit()
if log_foreign:
    print('WARNING: log contains data this firmware did not write.')
    print('         Recording is blocked until the log is erased.')
else:
    print('stored records:', log_count(),
          ' free:', max(0, data_limit() - log_ptr) // REC_SIZE)

print('health:', health_str(), ' (IMU / Baro / Mag / Flash / Vbat)')
if BRIDGE_WORKAROUND:
    print('*** BRIDGE DETECTED: GPIO4 is shorted to GPIO5.')
    print('***   flash CS is open-drain, SCL parks low during flash access.')
    print('***   Costs ~0.65 ms per sample. Repair the board to remove this.')
else:
    print('no GPIO4-GPIO5 bridge detected: normal push-pull chip select')

next_tick = time.ticks_ms()
last_status = time.ticks_ms()
imu_acc = [0, 0, 0, 0, 0, 0, 0]
imu_n = 0
_LIVE = bytearray(34)     # [0x01][record:32][health], reused every sample
_LIVE[0] = 0x01
last_tick_ms = 0
mag_skips = 0
mag_silent = 0        # consecutive samples with no magnetometer data

# ─── TEMPORARY: SPIKE DETECTOR ─────────────────────────────────────────
#
# The app shows single-sample spikes on EVERY channel at once -- mag, die
# temperature, barometer temperature, pressure. Those sensors are on
# different buses and different silicon, so nothing physical can move them
# together. A magnetic transient cannot touch the barometer. That leaves
# one corrupted RECORD rather than noisy sensors, and the corruption is
# either in how it is packed here or in how it crosses the BLE link.
#
# Measured separately: reading the magnetometer alone for 2000 samples
# gave 0.07 uT of scatter and not one spike. So the sensor and the SPI
# read are clean.
#
# This checks every record against the previous one BEFORE it is sent. If
# a spike appears here, the data really is wrong at the source and the
# context printed with it says what else was happening. If nothing ever
# prints while the app still shows spikes, the record left this board
# intact and the corruption is in the BLE transport or the app's decoder.
#
# Set SPIKE_DEBUG = False to turn it off.

SPIKE_DEBUG = False     # bench diagnostic: True prints SPIKE lines, even
                        # while recording, and each print holds up the loop
# counts, chosen well above real sample-to-sample noise. The die temperature
# was 60 (0.23 C) and flagged ordinary noise: the IMU's temperature output
# moves by up to ~80 counts between samples on this board.
_SPIKE_NAMES = ('mx', 'my', 'mz', 'itemp', 'temp', 'press', 'ax', 'ay', 'az')
_SPIKE_LIM = (400, 400, 400, 160, 100, 300, 800, 800, 800)
_spike_prev = [0] * 9
_spike_cur = [0] * 9
_spike_have = False
_spike_n = 0


def check_spike(mx, my, mz, itemp, temp, press, ax, ay, az):
    """Prints only on a jump. Compares into preallocated lists and builds
    text only when there is something to report: it runs on every sample,
    and the old dict-and-string version allocated ~220 bytes each time."""
    global _spike_have, _spike_n, _spike_prev, _spike_cur
    c = _spike_cur
    c[0] = mx; c[1] = my; c[2] = mz; c[3] = itemp; c[4] = temp
    c[5] = press; c[6] = ax; c[7] = ay; c[8] = az
    hits = None
    if _spike_have:
        p = _spike_prev
        for i in range(9):
            x = p[i]
            y = c[i]
            if x == BAD16 or y == BAD16 or x == BAD32 or y == BAD32:
                continue        # sensor-down sentinels are not spikes
            if abs(y - x) > _SPIKE_LIM[i]:
                if hits is None:
                    hits = []
                hits.append('%s %d->%d (d=%+d)' % (_SPIKE_NAMES[i], x, y, y - x))
    _spike_prev, _spike_cur = c, _spike_prev     # swap, no copy
    _spike_have = True
    if hits:
        _spike_n += 1
        if _spike_n <= 40:      # enough to see the pattern, not a flood
            print('SPIKE #%d  conn=%d rec=%d dump=%d magskip=%d imu_n=%d loop=%.1fms'
                  % (_spike_n, len(conns), recording, dumping, mag_skips, imu_n,
                     actual_ms if actual_ms else 0))
            for h in hits:
                print('    ' + h)


# ─── FIELD PAGE ────────────────────────────────────────────────────────
# fieldpage.py serves the probe's page over the USB cable (http://10.10.10.10)
# on the BORELOGGER_S3 firmware. It only ever queues the same Start/Stop
# bytes the BLE app sends, and reads flash only inside service(), on this
# loop, under this file's flash guard. Missing module, stock firmware or any
# failure here never stops logging. Set FIELD_PAGE = False to leave it out.
FIELD_PAGE = True
fieldpage = None
if FIELD_PAGE:
    try:
        import fieldpage as _fp
        if _fp.start(globals()):
            fieldpage = _fp
            print('field page: http://10.10.10.10 over USB')
        else:
            print('field page: not available on this firmware')
    except Exception as e:
        print('field page not started:', e)
_fp_fails = 0


def fieldpage_service():
    global fieldpage, _fp_fails
    try:
        fieldpage.service()
        _fp_fails = 0
    except Exception as e:
        _fp_fails += 1
        if _fp_fails <= 3:
            print('field page error:', e)
        if _fp_fails >= 20:
            print('field page disabled after repeated errors')
            fieldpage = None


# ─── MAIN LOOP ─────────────────────────────────────────────────────────
while True:
    try:
        if want_adv:
            want_adv = False
            advertise()

        drain_commands()
        if fieldpage:
            fieldpage_service()

        # Spend the idle part of the interval oversampling the IMU instead
        # of sleeping. Averaging every reading in the window is a decimation
        # filter: without it, a 4 Hz sample of a 104 Hz sensor folds all the
        # motion above 2 Hz back into the band we care about.
        for i in range(7):
            imu_acc[i] = 0
        imu_n = 0
        win_start = time.ticks_ms()
        win_mid = time.ticks_add(win_start,
                                 time.ticks_diff(next_tick, win_start) // 2)
        win_mag = None
        while time.ticks_diff(next_tick, time.ticks_ms()) > OVERSAMPLE_MS:
            # The window lasts most of the sample period -- up to a minute at
            # the slowest rate -- so commands and re-advertising are handled
            # here too. Otherwise a Stop at 0.05 Hz took 20 s to happen.
            # A Start handled here makes the first record of the session an
            # average that began before the start, and the gc.collect() at
            # Start (~200 ms) falls in the same window. One record, harmless,
            # but visible at the very start of a session.
            if pending or want_adv:
                if want_adv:
                    want_adv = False
                    advertise()
                drain_commands()
            try:
                if not _ok(HEALTH_IMU):
                    raise OSError('imu down')   # skip, do not wait on it
                imu_accumulate(imu_acc)
                imu_n += 1
            except Exception:
                pass
            # The boxcar average is centred on the middle of this window, so
            # the magnetometer is sampled there too. Reading it after the tick
            # instead put the two vectors ~150 ms apart at 4 Hz, which biases
            # heading during any rotation and is invisible to the filter.
            if win_mag is None and time.ticks_diff(time.ticks_ms(), win_mid) >= 0:
                if _ok(HEALTH_MAG):
                    try:
                        win_mag = read_mag()
                    except Exception:
                        _mark(HEALTH_MAG, False)
            if led_until and time.ticks_diff(time.ticks_ms(), led_until) >= 0:
                led.value(0)
                led_until = 0
            time.sleep_ms(OVERSAMPLE_MS)

        if led_until and time.ticks_diff(time.ticks_ms(), led_until) >= 0:
            led.value(0)
            led_until = 0

        now = time.ticks_ms()
        wait = time.ticks_diff(next_tick, now)
        if wait > 0:
            time.sleep_ms(wait)
        elif wait < -sample_ms:
            next_tick = now
        next_tick = time.ticks_add(next_tick, sample_ms)

        _p0 = time.ticks_us()
        # Real readings are clamped to -32767: an accelerometer saturated at
        # -2 g reads -32768, which is the 'sensor down' marker, so a hard
        # knock would otherwise be logged as a dead IMU.
        if imu_n:
            # boxcar average of everything gathered since the last tick
            itemp = _real16(imu_acc[0] // imu_n)
            gx = _real16(imu_acc[1] // imu_n)
            gy = _real16(imu_acc[2] // imu_n)
            gz = _real16(imu_acc[3] // imu_n)
            ax = _real16(imu_acc[4] // imu_n)
            ay = _real16(imu_acc[5] // imu_n)
            az = _real16(imu_acc[6] // imu_n)
        elif _ok(HEALTH_IMU):
            try:
                itemp, gx, gy, gz, ax, ay, az = read_imu()
                itemp, gx, gy, gz = _real16(itemp), _real16(gx), _real16(gy), _real16(gz)
                ax, ay, az = _real16(ax), _real16(ay), _real16(az)
            except Exception:
                _mark(HEALTH_IMU, False)
                itemp = gx = gy = gz = ax = ay = az = BAD16
        else:
            # already known down: do not pay the bus timeout again. The
            # retry timer will probe it on its own schedule.
            itemp = gx = gy = gz = ax = ay = az = BAD16
        if imu_n:
            _mark(HEALTH_IMU, True)
        _p1 = time.ticks_us()
        try:
            if not _ok(HEALTH_BARO):
                raise OSError('baro down')      # skip, do not wait on it
            if baro_async:
                baro_step()
                press, temp = last_press, last_temp
            else:
                press, temp = read_baro()
                last_press, last_temp = press, temp
        except Exception:
            _mark(HEALTH_BARO, False)
            if baro_async:
                baro_state = BARO_UNPRIMED    # the stepped cycle is out of step
            last_press, last_temp = BAD32, BAD16
            press, temp = BAD32, BAD16
        _p2 = time.ticks_us()
        if win_mag is not None:
            m = win_mag
        else:
            m = None
            if _ok(HEALTH_MAG):
                try:
                    m = read_mag()
                except Exception:
                    _mark(HEALTH_MAG, False)
        if m is None:
            mag_skips += 1
            # A silent magnetometer is not the same as a missing one: DRDY
            # simply never sets, nothing raises, and health would keep
            # reporting "up" forever. A long unbroken run of skips means the
            # part has stopped measuring, so hand it to retry_failed().
            if _ok(HEALTH_MAG):
                mag_silent += 1
                if mag_silent >= 200:
                    print('magnetometer silent for 200 samples, reinitialising')
                    _mark(HEALTH_MAG, False)
                    # start counting afresh: a re-init that "succeeds" but
                    # still produces nothing must be caught again, and with
                    # the count left at 200 it never reached 200 a second time
                    mag_silent = 0
            # a downed sensor writes an unreachable value rather than
            # repeating the last good one, so the log cannot be misread
            # (and one that has never produced a sample has nothing to
            # repeat: zeros would read as a real, if odd, field)
            m = last_mag if (_ok(HEALTH_MAG) and last_mag is not None) \
                else (BAD16, BAD16, BAD16)
        else:
            last_mag = m
            mag_silent = 0
        mx, my, mz = m
        _p3 = time.ticks_us()
        try:
            if not _ok(HEALTH_BATT):
                raise OSError('adc down')
            last_mv = read_battery_mv()
        except Exception:
            _mark(HEALTH_BATT, False)
            # a running board never reads 0 V, so this is unambiguous in the
            # log. Repeating the last good value would look like real data.
            last_mv = 0
        _p4 = time.ticks_us()

        # stamp the centre of the averaging window, not the moment the packet
        # happened to be assembled. Accumulated rather than measured against a
        # fixed origin: ticks_ms wraps, and a wrapped difference would go
        # negative and break the record format.
        ref = win_mid if imu_n else time.ticks_ms()
        t_ms_now += time.ticks_diff(ref, t_prev)
        t_prev = ref
        if t_ms_now < 0:
            t_ms_now = 0
        if t_ms_now > 0xFFFFFFEF:
            t_ms_now = 0xFFFFFFEF
        t_ms = t_ms_now
        # 32 bytes: divides evenly into both the 256-byte flash page and the
        # 4096-byte sector. The magnetometer is int16 because +/-32767 counts
        # is +/-437 uT, eight times anything this sensor has seen.
        rec = struct.pack('<IhhhhhhihhhhhH',
                          t_ms, ax, ay, az, gx, gy, gz,
                          press, clamp16(temp),
                          clamp16(mx), clamp16(my), clamp16(mz),
                          itemp, last_mv)

        # checked BEFORE the record goes anywhere, so a spike seen here is
        # real and a spike seen only in the app is a transport problem
        if SPIKE_DEBUG:
            check_spike(mx, my, mz, itemp, temp, press, ax, ay, az)

        if recording:
            try:
                wrote = log_write(rec)
                _mark(HEALTH_FLASH, True)
            except Exception as e:
                _mark(HEALTH_FLASH, False)
                wrote = False
                print('flash write failed:', e)
            if wrote:
                # the LED means one thing: a record reached the memory
                led.value(1)
                led_until = time.ticks_add(time.ticks_ms(), led_ms)
            else:
                recording = False
                print('flash full' if _ok(HEALTH_FLASH) else 'flash unavailable')
                send_status()

        # exponential average of the real period between samples
        if last_tick_ms:
            gap = time.ticks_diff(now, last_tick_ms)
            # outlier cut-off scaled to the period: a fixed 5000 ms threw away
            # every gap at sample periods of 5 s and longer, so the reported
            # rate stayed frozen at whatever it was before the change
            if 0 < gap < sample_ms * 4 + 1000:
                actual_ms = gap if not actual_ms else (actual_ms * 7 + gap) / 8
        last_tick_ms = now

        _p5 = time.ticks_us()
        prof[0] += time.ticks_diff(_p1, _p0)
        prof[1] += time.ticks_diff(_p2, _p1)
        prof[2] += time.ticks_diff(_p3, _p2)
        prof[3] += time.ticks_diff(_p4, _p3)
        prof[4] += time.ticks_diff(_p5, _p4)
        prof_n += 1

        stream_count += 1
        if not (conns and stream_on) or dumping:
            # nothing is going to consume it; without this the counter is
            # incremented every sample forever and never reset
            stream_count = 0
        elif stream_count >= stream_div:
            stream_count = 0
            # best effort: a dropped display frame costs nothing, whereas
            # blocking here would corrupt every timestamp that follows
            # health rides the sample frame, not just the 5 s status frame,
            # so the health strip updates at the same rate as the plots.
            # This appends a byte AFTER the record -- the 32-byte flash
            # format is untouched.
            _LIVE[1:33] = rec
            _LIVE[33] = health
            send_frame(h_data, _LIVE, False)
        prof[5] += time.ticks_diff(time.ticks_us(), _p5)

        sample_count += 1
        errors = 0
        retry_failed()

        if time.ticks_diff(time.ticks_ms(), last_status) >= 5000:
            last_status = time.ticks_ms()
            send_status()
            # Three lines of UART is ~217 characters, which at 115200 baud is
            # 18.8 ms of blocking -- enough to visibly delay a sample at any
            # rate above about 20 Hz. Nobody is reading the console during a
            # recording, so the diagnostics wait until it finishes.
            if recording:
                for i in range(6):
                    prof[i] = 0
                prof_n = 0
            else:
                print('t=%.1fs  set %.1f Hz  actual %.1f Hz  magskip=%d  '
                      'health=%s  %s%s%s' % (
                    t_ms / 1000, 1000.0 / sample_ms,
                    1000.0 / actual_ms if actual_ms else 0,
                    mag_skips, health_str(),
                    'REC' if recording else '--',
                    '  [BRIDGE WORKAROUND]' if BRIDGE_WORKAROUND else '',
                    ('  spikes=%d' % _spike_n) if SPIKE_DEBUG else ''))
            if prof_n and not recording:
                acct = sum(prof[:6]) / prof_n / 1000.0
                print('   baro: %s  osr_cmd=0x%02X  wait=%d ms  state=%d' % (
                    'stepped' if baro_async else 'blocking',
                    baro_d1, baro_wait_us // 1000, baro_state))
                print('   per sample: ' + '  '.join(
                    '%s %.2f' % (PROF_NAMES[i], prof[i] / prof_n / 1000.0)
                    for i in range(6)) +
                    '  | accounted %.2f of %.2f ms' % (acct, actual_ms))
                for i in range(6):
                    prof[i] = 0
                prof_n = 0

    except KeyboardInterrupt:
        led.value(0)
        # Shut the radio down on the way out. Harmless, and it makes a
        # stop-and-rerun cleaner when the VM is not restarted.
        try:
            ble.gap_advertise(None)
            ble.active(False)
        except Exception:
            pass
        # The page server is a thread and outlives this loop. Left running,
        # it keeps serving this run's (now dead) state, and running main.py
        # again without a reset starts a second one beside it. fieldpage.py
        # ends a server thread as soon as its _gen moves on.
        #
        # Wait for it to finish BEFORE returning, and only then close the
        # socket. Closing it under the thread made its accept() fail with
        # ENOTCONN, and fieldpage.py logs that by writing fp_errors.log --
        # a flash filesystem write. A soft reboot that follows straight after
        # (mpremote, Thonny) could kill the thread in the middle of that
        # write and hang the board until a power cycle. The thread's accept()
        # times out every second, so it sees the new _gen within about 1 s;
        # once its heartbeat (_beat) has gone quiet it has left its loop.
        if fieldpage:
            try:
                fieldpage._gen += 1
                t_stop = time.ticks_ms()
                while time.ticks_diff(time.ticks_ms(), t_stop) < 3000:
                    if time.ticks_diff(time.ticks_ms(), fieldpage._beat) > 1500:
                        break                # no heartbeat for 1.5 s: thread gone
                    time.sleep_ms(50)
                if fieldpage._srv:
                    fieldpage._srv.close()
                    fieldpage._srv = None
            except Exception:
                pass
        print('stopped, %d samples' % sample_count)
        break

    except Exception as e:
        # Individual sensor reads are guarded above, so anything landing here
        # is something else -- a bug, or memory pressure. Reinitialising every
        # sensor on a fault that has nothing to do with them used to spam the
        # console and disturb timing, so that is left to retry_failed().
        errors += 1
        errors_total += 1
        led.value(0)
        # throttled on the running total: errors itself restarts every 5, so
        # throttling on it printed 3 lines of every 5 failures, forever
        if errors_total <= 3 or errors_total % 50 == 0:
            print('loop error (%d): %s' % (errors_total, e))
        if errors >= 5:
            health &= ~(HEALTH_IMU | HEALTH_BARO | HEALTH_MAG)
            errors = 0
        time.sleep_ms(100)