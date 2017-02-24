import matplotlib.pyplot as plt

raw_data = []

with open(r'C:\Users\kkuwatani\Documents\Other\Personal\Sousvide\data\run3_Kp=50, Ki=2.5, Ki limit=2, more water.log') as file:
    for line in file:
        if line[0:4] == 'UART' or line == "":
            pass
        else:
            raw_data.append(line[6:-1])

processed_data = []
set_point = []
time = []

counter = 0

for i in range(0, len(raw_data), 2):
    # try:
    processed_data.append(int(raw_data[i+1][2:]+raw_data[i][2:], 16)*0.0625)
    # except Exception as e:
    #     print(e)
    #     del raw_data[i]

for i in range(len(processed_data)):
    set_point.append(70)
    time.append(counter)
    counter += 10./60


plt.figure(1)
plt.grid(which='both')
plt.locator_params(nbins=10)
plt.plot(time, processed_data)
plt.plot(time, set_point, '--')
plt.xlabel(r'$Time (minutes)$', fontsize=22)
plt.ylabel(r'$Degrees (C)$', fontsize=22)
plt.title(r'Sous Vide Temp Feedback, Kp=50, Ki=2.5, Ki limit=2')

plt.show()