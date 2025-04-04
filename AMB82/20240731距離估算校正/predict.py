import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score
df = pd.read_csv("data.csv")
df
X = df[["measure"]]
y = df[["distance"]]

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
X_train
model = LinearRegression()

from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
degree = 5
poly = PolynomialFeatures(degree)
model = make_pipeline(poly, LinearRegression())
model.fit(X_train, y_train)


y_pred = model.predict(X_test)

mse = mean_squared_error(y_test, y_pred)
r2 = r2_score(y_test, y_pred)

print(f'mse: {mse}')
print(f'r2: {r2}')

# print(model.predict([[input()]]))

poly_features = poly.get_feature_names_out(['measure'])
coefficients = model.named_steps['linearregression'].coef_
intercept = model.named_steps['linearregression'].intercept_
print("數學模型：")
formula = f'{intercept}'
for coef, feature in zip(coefficients, poly_features):
  formula += f' + ({coef} * {feature})'
print(formula)

x = float(input())
print(-14.04+1.18*x)
print(0.1054+0.9397579*x+8.86754e-4*(x**2))
print(-15.5162+1.367*x-2.5996e-3*(x**2)+8.6963e-6*(x**3))
print(3.89298+0.64038*x+6.6809e-3*(x**2)-3.994e-5*(x**3)+8.97e-8*(x**4))
print(467.69487-21.4*x+0.39583*(x**2)-3.25866e-3*(x**3)+1.26646e-5*(x**4)-1.871e-8*(x**5))
