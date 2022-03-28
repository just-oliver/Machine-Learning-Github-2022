# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

xvals = np.linspace(-5,5,num=501)
novals = np.zeros(501)

#yvals = 1 / (1 + np.exp(-xvals))
#yvals = (np.exp(xvals) - np.exp(-xvals)) / (np.exp(xvals) + np.exp(-xvals))
#yvals = np.maximum(xvals, novals)
yvals = np.maximum(xvals, xvals * 0.01)
#yvals = np.maximum(xvals, np.exp(xvals) - 1)
#for x in range(250):
#    yvals[500-x] = xvals[500-x]

yspace = np.linspace(np.min(yvals),np.max(yvals),num=501)

plt.plot(xvals, novals, color="k")
plt.plot(novals, yspace, color="k")
plt.plot(xvals, yvals)
plt.xlabel("Input value")
plt.ylabel("Result")
plt.title("Leaky relu function (Gradient for x < 0 is 0.01)")
plt.xlim(xvals[0],xvals[500])
plt.ylim(np.min(yvals),np.max(yvals))
plt.grid(True)

