#!/usr/bin/env python

import subprocess

def get_cpu_time(process_name, unit='ticks'):
    """
    Given a process name with an associated PID (process ID)
    this function calculates how much cpu time it has consumed
    at the time of the query by using :
    cat /proc/'my PID'/stat
    based on:
    http://stackoverflow.com/questions/16726779/how-do-i-get-the-total-cpu-usage-of-an-application-from-proc-pid-stat
    input: 
        process_name: the name of the linux process from which you want to query CPU time
        unit: 'ticks' or 'secs', the desired time unit
    output: list
    1 - CPU time in user code of the process in clock ticks
    2 - CPU time in kernel code of the process in clock ticks
    3 - CPU time in user code of the child process in clock ticks
    4 - CPU time in kernel code of the child process in clock ticks
    """
    try:
        pid = int(subprocess.check_output("pidof " + process_name, shell=True))
    except:
        print 'Error: process with name ": ' + process_name + '" does not exist'
        return None
    file_ = open("/proc/%d/stat"%pid)
    process_info = file_.readline().split()
    # use shell command : "man 5 proc" to get more information
    utime = float(process_info[13])  # (14) CPU time spent in user code, measured in clock ticks
    stime = float(process_info[14])  # (15) CPU time spent in kernel code, measured in clock ticks
    cutime = float(process_info[15]) # (16) Children process CPU time spent in user code (in clock ticks)
    cstime = float(process_info[16]) # (17) Children process CPU time spent in kernel code (in clock ticks)
    # see if user requires the time to be given in secs or ticks
    if unit == 'secs':
        # get ticks per second
        ticks_per_second = float(subprocess.check_output("getconf CLK_TCK", shell=True))
        # divide each element of the list by ticks_per_second
        return [x / ticks_per_second for x in [utime, stime, stime, cstime]]
    elif unit == 'ticks':
        return [utime, stime, cutime, cstime]
    else:
        print "Error: while getting cpu time, unit must be ticks or secs, unknown option given"
        return None
