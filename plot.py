import matplotlib.pyplot as plt
import numpy as np
plt.style.use('seaborn-v0_8')

x = np.linspace(-5, 5, 1000)
y = np.tanh(x / 0.5)

print(np.arctanh(0.99) * 0.5)

plt.plot(x, y)
plt.plot(x, 0*y+0.95, '--', color='black', lw=1)
plt.plot(x, 0*y-0.95, '--', color='black', lw=1)
# plt.grid(True)
plt.show()
