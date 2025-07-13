import matplotlib.pyplot as plt

# Scalability plot
dimension = [7, 42, 82]
## fai salvare i tempi di esecuzione e importali
execution_times = [t_7, t_42, t_82]
plt.plot(dimension, execution_times, linestyle='dotted', color='blue')
plt.set_title(f"Model scalability")
plt.set_xlabel("Dimensionality")
plt.set_ylabel("Time (s)")
plt.savefig("Model scalability")