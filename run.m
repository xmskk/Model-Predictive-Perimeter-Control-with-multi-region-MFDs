for i = 1:40
    results(i) = mppc(i);
    tts(i) = sum(results(i).s.x, 'all');
end

figure(1)
plot(results(20).s.x(1, :))
hold on
plot(results(20).s.x(2, :))
hold on
plot(results(20).s.x(3, :))
hold on
plot(results(20).s.x(4, :))
hold off
fig = gcf;
title('Accumulation per group')
xlabel('Time (min)')
ylabel('Vehicles (veh)')
legend('n11', 'n12', 'n21', 'n22')
ylim([0, 10000])
xlim([0, 120])
exportgraphics(fig, 'Figure_1.png', 'Resolution', 100)

figure(2)
plot(results(20).p.q(1, :))
hold on
plot(results(20).p.q(2, :))
hold on
plot(results(20).p.q(3, :))
hold on
plot(results(20).p.q(4, :))
hold off
fig = gcf;
title('Demand flow q')
xlabel('Time (min)')
ylabel('Flow (veh/s)')
legend('q11', 'q12', 'q21', 'q22')
ylim([0, 5])
xlim([0, 120])
exportgraphics(fig, 'Figure_2.png', 'Resolution', 100)

figure(3)
stairs(results(20).s.u(1, :))
hold on
stairs(results(20).s.u(2, :))
hold off
fig = gcf;
title('Evolution of u')
xlabel('Time (min)')
ylabel('rate (u)')
legend('u12', 'u21')
ylim([0, 1])
xlim([0, 120])
exportgraphics(fig, 'Figure_3.png', 'Resolution', 100)

figure(4)
plot(results(20).s.x(1, :) + results(20).s.x(2, :))
hold on
plot(results(20).s.x(3, :) + results(20).s.x(4, :))
hold off
fig = gcf;
title('Accumulation per region')
xlabel('Time (min)')
ylabel('Vehicles (veh)')
legend('n1', 'n2')
ylim([0, 10000])
xlim([0, 120])
exportgraphics(fig, 'Figure_4.png', 'Resolution', 100)

figure(5)
plot(mfd(results(20).s.x(1, :) + results(20).s.x(2, :)))
hold on
plot(mfd(results(20).s.x(3, :) + results(20).s.x(4, :)))
hold off
fig = gcf;
title('Outflow per region')
xlabel('Time (min)')
ylabel('Outflow (veh/s)')
legend('G1', 'G2')
ylim([0, 10])
xlim([0, 120])
exportgraphics(fig, 'Figure_5.png', 'Resolution', 100)

figure(6)
plot(tts')
hold off
fig = gcf;
title('Total time spent')
xlabel('prediction horizon Nc')
ylabel('total time spent (veh*s)')
xlim([1, 40])
exportgraphics(fig, 'Figure_6.png', 'Resolution', 100)

function G = mfd(n)

    % MFD value of accumulation n
    G = (1.4877*(10^(-7))*(n.^3) - 2.9815*(10^(-3))*(n.^2) + 15.0912*n)/3600;
    
end