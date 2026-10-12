"""The intake's field-confirmed text protocol, shared by route and console."""
import time


def _sequence(send, steps, wait, finish_on_error=False):
    replies = []
    all_ok = True
    for command, pause in steps:
        ok, message = send(command)
        replies.append(f'{command}: {message}')
        all_ok = all_ok and ok
        if not ok and not finish_on_error:
            return False, ' / '.join(replies)
        if pause:
            wait(pause)
    return all_ok, ' / '.join(replies)


def start_intake(send, wait=time.sleep):
    return _sequence(send, [('STOP', .3), ('ZERO', .8), ('MOVE 1 90 5', 0.)], wait)


def stop_intake(send, wait=time.sleep):
    # Retain the old place_only preset's 1 s before stopping and re-zeroing.
    return _sequence(send, [('MOVE 1 0 5', 1.), ('STOP', .3), ('ZERO', 0.)],
                     wait, finish_on_error=True)
