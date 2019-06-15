import numpy as np
from scipy import signal
from scipy import constants
import math

#adopted from: https://github.com/breeswish/hexi

class RealtimeFilter():
  def __init__(self, b, a):
    assert(len(b) == len(a))
    self.n = len(b) # n = order + 1
    self.b = b
    self.a = a
    self.reset()

  def reset(self):
    self.input = np.zeros(self.n, dtype=np.float)
    self.output = np.zeros(self.n, dtype=np.float)

  def apply(self, v):
    self.input[self.n - 1] = v
    self.output[self.n - 1] = 0
    output = 0
    for i in range(0, self.n):
      output = output + \
        self.b[i] * self.input[self.n - 1 - i] - \
        self.a[i] * self.output[self.n - 1 - i]
    self.output[self.n - 1] = output
    for i in range(0, self.n - 1):
      self.input[i] = self.input[i+1]
      self.output[i] = self.output[i+1]
    return output


