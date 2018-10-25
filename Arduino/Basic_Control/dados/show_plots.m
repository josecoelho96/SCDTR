data = load('t3.tsv');

Ki = 0.5;
Kp = 1;

plot(data(:, 1)/1000000, data(:, 2));
hold on;
plot(data(:, 1)/1000000, data(:, 3));
grid on;
xlabel('Tempo [s])');
ylabel('Valor [PWM | lux]');
title(sprintf('Evolucao temporal do sistema. Kp =  %.2f, Ki = %.2f', Kp, Ki));
legend('Entrada (PWM)', 'Saída (lux)');
