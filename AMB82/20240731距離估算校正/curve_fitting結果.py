import matplotlib.pyplot as plt
import pandas as pd
def order_1(x):
    return -14.04+1.18*x
def order_2(x):
    return 0.1054+0.9397579*x+8.86754e-4*(x**2)
def order_3(x):
    return -15.5162+1.367*x-2.5996e-3*(x**2)+8.6963e-6*(x**3)
def order_4(x):
    return 3.89298+0.64038*x+6.6809e-3*(x**2)-3.994e-5*(x**3)+8.97e-8*(x**4)
def order_5(x):
    return 467.69487-21.4*x+0.39583*(x**2)-3.25866e-3*(x**3)+1.26646e-5*(x**4)-1.871e-8*(x**5)

ideal = [i for i in range(50, 251)]

raw_data = pd.read_csv("./data.csv")
raw_distance = raw_data["distance"]
raw_measure = raw_data["measure"]

res1 = [order_1(i) for i in range(50, 251)]
res2 = [order_2(i) for i in range(50, 251)]
res3 = [order_3(i) for i in range(50, 251)]
res4 = [order_4(i) for i in range(50, 251)]
res5 = [order_5(i) for i in range(50, 251)]

plt.plot(raw_measure, raw_distance, "o", label="original data point", ms=5.0)
plt.plot([i for i in range(50, 251)], res1, label="1 order")
plt.plot([i for i in range(50, 251)], res2, label="2 order")
plt.plot([i for i in range(50, 251)], res3, label="3 order")
plt.plot([i for i in range(50, 251)], res4, label="4 order")
plt.plot([i for i in range(50, 251)], res5, label="5 order")

plt.xlabel("calculated distance")
plt.ylabel("ideal distance")

plt.legend()
plt.grid()
plt.show()