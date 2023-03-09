import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# parse
series = []

# Data gathered by
# candump can0,040:FFF -td > "candump.txt"
# Format should look like this" (000.010060)  can0  010   [8]  14 00 00 00 00 00 B9 00",

with open("candump.txt") as f:
    lines = f.readlines()

for msg in lines:
    msg_split = msg.split(" ")

    _id = msg_split[5]
    _delta_t = float(msg_split[1][1:-1])
    _counter = int(msg_split[17], 16)
    _position = int(
        "".join(msg_split[12:16]), 16
    )  # ''.join() concats the strings with '' as a (no) divider

    series.append([_id, _delta_t, _counter, _position])

print(len(series), " samples plotted")

df = pd.DataFrame(data=series[1:], columns=["id", "delta_t", "counter", "position"])

# plot

sns.relplot(
    data=df,
    kind="line",
    x=df.index,
    y="delta_t",  # errorbar="sd",
)


plt.xlabel("samples")
plt.ylabel("delta_t [s]")
plt.show()
