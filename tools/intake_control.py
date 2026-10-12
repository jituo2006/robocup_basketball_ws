"""The intake's field-confirmed text protocol, shared by route and console."""
import time


def start_intake(send, wait=time.sleep):
    replies = []
    for command, pause in [('STOP', .3), ('ZERO', .8), ('MOVE 1 90 5', 0.)]:
        ok, message = send(command)
        replies.append(f'{command}: {message}')
        if not ok:
            return False, ' / '.join(replies)
        if pause:
            wait(pause)
    return True, ' / '.join(replies)
