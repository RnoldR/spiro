"""
I am having this error too with Thonny version 4.1.4, see code below.
It happens when I run from my local drive, from the rpico drive
itself or use this as the code.py code. Am I doing something wrong?

Code:
"""
with open('logfile.txt', 'w') as outfile:
    outfile.write('Hi there\n')
"""
Thonny Reponse:
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
OSError: [Errno 30] Read-only filesystem
"""