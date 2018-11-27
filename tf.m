clc;
clear;

y = [];
yi = y(end);
idx = max(find(abs(y-yi)>=0.37*yi));
k = 3.1

sys = tf([k],[idx 1])

plot(1)
step(sys)
ylabel('Measured Signal')
xlabel('Time')
