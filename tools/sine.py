#!/usr/bin/env python

from math import sin, pi, cos
STEPS = 100

print "#ifndef __SINE_H__"
print "#define __SINE_H__\n"

print "#define SINE_STEPS %s" % STEPS
print "int8_t sine[%d] =\n{" % STEPS;

step = (pi * 2) / STEPS; 
i = 0
offset = 0.0 #pi / 2;
numbers = []
for i in xrange(0, STEPS):
    numbers.append("%d" % ((127 * sin(i * step + offset))))

print ",".join(numbers)

print "};\n";

print "#endif"
