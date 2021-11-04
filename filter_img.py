#!/usr/bin/env python
# # coding=latin-1

import numpy as np

# FIR bei gleichem a fuer alle Zeitpunkte:
# x(n) = y(n)*a + y(n-1)*a + ... + y(0)*a = y(n)*a + x(n-1)
class LowPassFilter(object):
    """IIR Filter: x(n) = y(n)*k + x(n-1)*1
        Args:
            data_shape: shape of data to filter
            k = b_0 bei IIR (entspricht a bei FIR)"""
    def __init__(self, data_shape, k=0.2):
        self.integrator = np.zeros(data_shape)
        self.k = k  # omega*control_period

    def filter(self, data):
        data = np.array(data).astype("float")
        self.integrator = np.abs(self.integrator + (data - self.integrator) * self.k)
        return self.integrator


def main():
    import random
    lowPass = LowPassFilter((2, 1), 0.05)
    data = np.array([200, 200])

    for i in range(5):
        data_in = data + [random.randint(-10, 10), random.randint(-10, 10)]
        data_out = lowPass.filter(data)

    print("Data after 5 iterations")
    print(data_out)

    for i in range(100000):
        data_in = data + [random.randint(-1, 1), random.randint(-1, 1)]
        data_out = lowPass.filter(data)

    print("Data after 100000 iterations")
    print(data_out)


if __name__ == "__main__":
    main()
