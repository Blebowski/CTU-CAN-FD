import subprocess
import os
import signal

__all__ = ['run']
def get_children_pids(parent_pid):
    cmd = ['ps', '-o', 'pid', '--ppid', str(parent_pid), '--noheaders']
    res = subprocess.run(cmd, stdout=subprocess.PIPE, check=False)
    out = res.stdout.decode('ascii')
    return [int(pid_str) for pid_str in out.split() if int(pid_str) != parent_pid]


def recursive_kill(pid, sig=signal.SIGTERM):
    children = get_children_pids(pid)
    for child in children:
        recursive_kill(child)
        try:
            os.kill(child, sig)
        except ProcessLookupError:
            pass


# ghdl creates a new process group for itself and fails to kill its child when it receives a signal :(
def sighandler(signo, frame):
    signal.signal(signo, signal.SIG_DFL)
    recursive_kill(os.getpid(), signo)
    # restore the handler, because vunit swallows the resulting exception
    #signal.signal(signo, sighandler)


def run(ui):
    signal.signal(signal.SIGTERM, sighandler)
    signal.signal(signal.SIGINT, sighandler)
    ui.main()
