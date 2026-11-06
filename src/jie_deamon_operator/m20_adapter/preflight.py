"""Preflight report format and the live-start gate that consumes it.

``scripts/m20_preflight.py`` runs bounded, read-only checks on GOS and writes a
JSON report.  The live bridge refuses to open the AOS control socket unless a
fresh, all-pass report is present when ``require_preflight:=true``.  This turns
``commissioned:=true`` from a blank attestation into a checkable precondition
without pretending the checks are a safety certification.
"""
import json
import os
import platform
import time

#: Bump when the meaning of the checks changes; stale reports are rejected.
REPORT_VERSION = 2


def hostname():
    try:
        return os.uname().nodename
    except AttributeError:  # non-Linux (unit tests, development hosts)
        return platform.node() or 'unknown'


def required_checks(checks):
    return [c for c in checks if c.get('severity', 'required') == 'required']


def build_report(checks, host=None, extra=None, now=None):
    now = time.time() if now is None else now
    return {
        'version': REPORT_VERSION,
        'generated_at': now,
        'generated_at_text': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now)),
        'host': host or hostname(),
        'ok': all(bool(c.get('ok')) for c in required_checks(checks)),
        'checks': checks,
        'extra': extra or {},
    }


def write_report(path, report):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)
    return path


def check_preflight(path, max_age=600.0, now=None):
    """Validate a preflight report.  Returns ``{ok, reason, report}``."""
    now = time.time() if now is None else now
    result = {'ok': False, 'reason': '', 'report': None}
    if not path or not os.path.exists(path):
        result['reason'] = 'preflight report missing: %s' % path
        return result
    try:
        with open(path, encoding='utf-8') as handle:
            report = json.load(handle)
    except (OSError, ValueError) as exc:
        result['reason'] = 'preflight report unreadable: %s' % exc
        return result
    result['report'] = report
    if report.get('version') != REPORT_VERSION:
        result['reason'] = ('preflight report version %r != %r'
                            % (report.get('version'), REPORT_VERSION))
        return result
    age = now - float(report.get('generated_at', 0))
    if age < -60.0 or age > float(max_age):
        result['reason'] = 'preflight report age %.1fs outside [0, %.1fs]' % (age, max_age)
        return result
    failed = [c.get('name') for c in required_checks(report.get('checks', []))
              if not c.get('ok')]
    if not report.get('checks'):
        result['reason'] = 'preflight report has no checks'
        return result
    if failed:
        result['reason'] = 'preflight checks failed: ' + ', '.join(str(f) for f in failed)
        return result
    result['ok'] = True
    return result
