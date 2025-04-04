import pandas as pd
data = pd.read_csv("./data.csv")
measure_1 = data["measure"][0:21].reset_index(drop=True)
measure_2 = data["measure"][21:42].reset_index(drop=True)
measure_3 = data["measure"][42:63].reset_index(drop=True)
measure_4 = data["measure"][63:84].reset_index(drop=True)
measure_5 = data["measure"][84:105].reset_index(drop=True)
measure_6 = data["measure"][105:126].reset_index(drop=True)
measure_7 = data["measure"][126:147].reset_index(drop=True)
measure_8 = data["measure"][147:].reset_index(drop=True)

new_data = pd.DataFrame({
    "distance":data["distance"][0:21],
    "measure_1":measure_1,
    "measure_2":measure_2,
    "measure_3":measure_3,
    "measure_4":measure_4,
    "measure_5":measure_5,
    "measure_6":measure_6,
    "measure_7":measure_7,
    "measure_8":measure_8,
})
print(new_data)
new_data.to_csv("./data_processed.csv")
# print(data)