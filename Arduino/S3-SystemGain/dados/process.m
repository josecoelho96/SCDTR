filename = 't4.tsv';
fileID = fopen(filename, 'r', 'n', 'UTF-8');

LDR = [];
LED = [];

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

figure;
yyaxis left;
plot(LDR(:,1), LDR(:,2))
yyaxis right;
stairs(LED(:,1), LED(:,2))
figure;
plot(diff(LDR(:,1)))
