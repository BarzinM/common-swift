import sys
import subprocess
import signal


def preexec_function():
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def get(parameter):
    subprocess.check_call(["rosservice", "call", "/mavros/param/get", parameter], preexec_fn=preexec_function)


def set(parameter, value):
    pixhawk_value = "[0," + str(float(value)) + "]"
    return subprocess.check_call(["rosservice", "call", "/mavros/param/set", parameter, pixhawk_value], preexec_fn=preexec_function)

if __name__ == "__main__":
    parameter = sys.argv[1]
    print parameter
    print "=" * 10
    iterate = True
    while iterate:
        try:
            get(parameter)
            print "-" * 20
        except KeyboardInterrupt:
            print "\nExiting"
            iterate = False
            break
