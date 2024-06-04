import numpy as np

J = np.array([16.571710e-6, 16.655602e-6, 29.261652e-6])
KR = np.array([0.007, 0.007, 0.008])
Komega = np.array([0.00115, 0.00115, 0.002])
KI = np.array([0.03, 0.03, 0.03])

for var, name in [(KR, "KR"), (Komega, "Komega"), (KI, "KI")]:
    var = var / J
    elts = ", ".join(str(x) for x in var)
    print(f"{name} = " + "{" + elts + "},")
