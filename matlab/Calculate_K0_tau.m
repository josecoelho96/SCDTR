%% Calculate poly for G0
xG0 = [0 28 58 86 112 140]; 
yG0= [0 50 100 150 200 255];
figure;
hold on;
plot(xG0,yG0,'o');
p3 = polyfit(xG0,yG0,1);
xpoly = 0:1:200;
ypoly = (p3(1)*xpoly)+p3(2);
plot(xpoly,ypoly);
title('Variação da tensão do LED com os Luxs pretendidos');
xlabel('Iluminância pretendida [Lux]');
ylabel('PWM');
legend('PWM para os valores de iluminância', 'Polinómio que aproxima os dados');


%% Calculate poly for K0
xK0 = [0 3.01 0.525 0.25 0.155 0.115 -0.115 -0.155 -0.26 -0.52 -3.005];
yK0 = [0 3.070175439 0.5353864981 0.2551020408 0.1580020387 0.1066790353 0.1066790353 0.1580020387 0.2653061224 0.530287579 3.065075479];

figure;
hold on;
plot(xK0,yK0,'o');
p1 = polyfit(xK0,yK0,2);
xpoly = -5:0.001:5;
ypoly = (p1(1)*xpoly.^2)+(p1(2)*xpoly)+p1(3);
plot(xpoly,ypoly);
title('Variação do ganho estático com a variação da tensão');
xlabel('\Delta Volt [V]');
ylabel('Ganho estático');
legend('Ganho das amostras', 'Polinómio que aproxima os dados');

%% Calculate poly for tau

xtau = [0 3.01 0.525 0.25 0.155 0.115 -0.115 -0.155 -0.26 -0.52 -3.005];
ytau = [0 0.18 0.089 0.063 0.017 0.005 0.002 0.006 0.007 0.011 0.013];

figure;
hold on;
plot(xtau,ytau,'o');
p2 = polyfit(xtau,ytau,2);
xpoly = -5:0.001:5;
ypoly = (p2(1)*xpoly.^2)+(p2(2)*xpoly)+p2(3);
plot(xpoly,ypoly);
title('Variação da constante de tempo com a variação da tensão');
xlabel('\Delta Volt [V]');
ylabel('Tau [s]');
legend('Tau das amostras', 'Polinómio que aproxima os dados');