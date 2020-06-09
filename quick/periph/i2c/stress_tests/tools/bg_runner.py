#!/usr/bin/env python3

import signal
import sys
import subprocess
import time
import shlex

# application timeout in seconds
timeout = 2

def main():
  if len(sys.argv) <= 1:
    return

  start_time = time.monotonic()

  child_processes = []
  for arg in sys.argv[1:]:
    child_processes.append(subprocess.Popen(shlex.split(arg),
                                            stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE
                                            ))

  time.sleep(timeout)

  for cp in child_processes:
    cp.send_signal(signal.SIGINT)
    outs, errs = cp.communicate()
    print("command: {}".format(" ".join(cp.args)))
    print(" status code: {}".format(cp.returncode))
    print(" stdout: {}".format(outs.decode("utf-8")))
    print(" stderr: {}\n".format(errs.decode("utf-8")))

  elapsed_time = time.monotonic() - start_time
  print("Elapsed time: {}".format(elapsed_time))

if __name__ == "__main__":
  main()
