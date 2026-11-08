"""Startup-only scheduling for event-driven localization threads on the RDK.

SCHED_FIFO is a scheduling policy, not a PREEMPT_RT kernel or deadline proof.
Failure keeps the existing normal scheduling; no sensor/motor admission gate.
"""
import os
import resource
import subprocess

PRIORITY = 10


def requested():
    return os.environ.get('RDK_LOCALIZATION_RT', '1').lower() not in ('0', 'false', 'off')


def prepare_realtime():
    """Grant only this process/its children rtprio, before starting workers.

    The RDK account already has noninteractive sudo. No root application,
    kernel setting, login-wide limit or repeated privilege helper is needed.
    """
    error = None
    if requested():
        soft, hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
        if soft != resource.RLIM_INFINITY and soft < PRIORITY:
            try:
                if hard == resource.RLIM_INFINITY or hard >= PRIORITY:
                    resource.setrlimit(resource.RLIMIT_RTPRIO, (PRIORITY, hard))
                else:
                    result = subprocess.run(['sudo', '-n', 'prlimit', '--pid', str(os.getpid()),
                        '--rtprio=' + str(PRIORITY) + ':' + str(PRIORITY)],
                        capture_output=True, text=True, timeout=1.)
                    if result.returncode:
                        error = result.stderr.strip() or 'rtprio permission setup failed'
            except (OSError, subprocess.TimeoutExpired) as exc:
                error = str(exc)
    return dict(requested=requested(), requested_priority=PRIORITY,
        rtprio_limit=list(resource.getrlimit(resource.RLIMIT_RTPRIO)), error=error)


def set_realtime_thread():
    """Apply to the caller only; create fit/log/receiver threads beforehand."""
    error = None
    if requested():
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(PRIORITY))
        except OSError as exc:
            error = str(exc)
    return dict(requested=requested(), requested_priority=PRIORITY,
        actual_policy=os.sched_getscheduler(0), actual_priority=os.sched_getparam(0).sched_priority,
        error=error)
