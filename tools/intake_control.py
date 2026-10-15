"""Legacy raw manual presets; automatic routes use mechanism_control.

These helpers only write commands. They do not prove acceptance or completion.
Do not use start_intake to re-zero an automatic pickup on every ball.
"""
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
    return _sequence(send, [('dji_stop', .3), ('dji_zero', .8), ('dji_move 1 90 5', 0.)], wait)


def stop_intake(send, wait=time.sleep):
    return _sequence(send, [('dji_move 1 0 5', 0.)], wait)
