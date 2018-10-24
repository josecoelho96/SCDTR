filename = 't7-pos.tsv';
fileID = fopen(filename, 'r', 'n', 'UTF-8');
LDR = [];
LED = [];
mLdr = -0.652;
bLdr = 1.76;
Vref = 5; % arduino vcc
maxResistanceLdr = 1000000; % 1MOhm
R1ref = 9850; % measured value


line = fgetl(fileID);
while ischar(line)

    if contains(line, "LED: ")
        A = sscanf(line, "LED: %d\t%d");
        LED = [LED; A'];
    else
        A = sscanf(line, "%d\t%d");
        LDR = [LDR; A'];
    end
    line = fgetl(fileID);
end
fclose(fileID);

LED(:, 3) = ((LED(:,2)).*Vref)/255.0;

% LDR content by index
% 1 - timestamp
% 2 - adc value
% 3 - voltage
% 4 - resistance
% 5 - lux value
LDR(:, 3) = (Vref*LDR(:, 2))/1023.0;

for i=1:length(LDR)
    if LDR(i, 3) == 0
        LDR(i, 4) = maxResistanceLdr;
    else
        LDR(i, 4) = R1ref*(Vref/LDR(i, 3) - 1);
    end
end

LDR(:, 5) = 10.^((log10(LDR(:, 4)/1000) - bLdr)/mLdr);

% smoothing out of values
for i = 1:length(LDR)-5
    LDRsmooth(i, :) = [LDR(i, 1) mean(LDR(i:i+5, 3))];
end

LDRmedian = medfilt1(LDR(:,3), 5);
    
figure;
hold on;
grid on;
yyaxis left;
stairs(LED(:,1), LED(:,2), 'b')
ylabel('Input: LED [8-bit PWM: 0 - 255]')
yyaxis right;
plot(LDR(:,1), LDR(:,2), 'r')
title('Variação do LED e reposta do LDR - Raw');
xlabel('Tempo [us]')
legend('PWM LED', 'Medição ADC LDR');

figure;
hold on;
grid on;
yyaxis left;
stairs(LED(:,1), LED(:,3), 'b.-')
ylabel('Input: LED [8-bit PWM: 0 - 255]')
yyaxis right;
plot(LDR(:,1), LDR(:,3), 'r.--')
plot(LDRsmooth(:,1), LDRsmooth(:,2), 'g.-')
plot(LDR(:,1), LDRmedian, 'k.-')
% ylabel('Resposta no LDR [Voltage]')
% title('Variação do LED e reposta do LDR - Processado');
% xlabel('Tempo [us]')
% legend('PWM LED', 'Medição Voltage LDR', 'Medição voltage LDR (averaged)');

% adding helper lines
refline(0, 1.90232);
refline(0, 3.3418);

