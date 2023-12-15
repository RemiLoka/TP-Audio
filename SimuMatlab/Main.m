clear all;

% Audio

[voix, Fs] = audioread('/Users/remi/Desktop/TP-Audio/SimuMatlab/DataAudio/s1.wav');

% Bruit

t = 1:length(voix);
noise = 0.02 * randn(size(t));
noise = noise';

%% Naive Realisation

% Paramètres

seuil = [0.0001,0.001,0.01,0.1,0.3,0.5];
voixSeuil = zeros(length(seuil),length(voix));
valueRemove = zeros(length(seuil),1);

% Fonction

for n = 1:length(seuil)
    [voixSeuil(n,:), valueRemove(n,:)] = NaiveRealisation(voix,seuil(n));
end

% Affichage

N = length(voixSeuil(1,:));
axis_naive = 1:N;

xLabel = (7*N)/8;
yLabel = 0.70;

figure

subplot(321)
plot(axis_naive, voix, 'b', axis_naive, voixSeuil(1,:), 'r')
label = num2str((valueRemove(1,1)/N)*100);
text(xLabel, yLabel, label, 'EdgeColor', 'black');
title('voix filtrée avec un seuil de 0.0001')

subplot(322)
plot(axis_naive, voix, 'b', axis_naive, voixSeuil(2,:), 'r')
label = num2str((valueRemove(2,1)/N)*100);
text(xLabel, yLabel, label, 'EdgeColor', 'black');
title('voix filtrée avec un seuil de 0.001')

subplot(323)
plot(axis_naive, voix, 'b', axis_naive, voixSeuil(3,:), 'r')
label = num2str((valueRemove(3,1)/N)*100);
text(xLabel, yLabel, label, 'EdgeColor', 'black');
title('voix filtrée avec un seuil de 0.01')

subplot(324)
plot(axis_naive, voix, 'b', axis_naive, voixSeuil(4,:), 'r')
label = num2str((valueRemove(4,1)/N)*100);
text(xLabel, yLabel, label, 'EdgeColor', 'black');
title('voix filtrée avec un seuil de 0.1')

subplot(325)
plot(axis_naive, voix, 'b', axis_naive, voixSeuil(5,:), 'r')
label = num2str((valueRemove(5,1)/N)*100);
text(xLabel, yLabel, label, 'EdgeColor', 'black');
title('voix filtrée avec un seuil de 0.3')

subplot(326)
plot(axis_naive, voix, 'b', axis_naive, voixSeuil(6,:), 'r')
label = num2str((valueRemove(6,1)/N)*100);
text(xLabel, yLabel, label, 'EdgeColor', 'black');
title('voix filtrée avec un seuil de 0.5')

sound(voixSeuil(3,:),Fs)

%% Réaliation avancée

% Paramètres

seuil = 0.05;
window_size = 8000;
slope = 500;
hold = 8000;

% Fenêtrage

% voix = voix(1:30000);

% Filtrage

[y, filtre, energie] = AdvanceRealisation(voix + noise, seuil, window_size, slope, hold);

% Affichage

N = length(y);
axis_avance = 1:N;

figure
subplot(211)
plot(axis_avance,voix,'b', axis_avance,y,'r');
title('voix filtrée et voix originelle')
subplot(212)
plot(filtre)
title('filtre associé')

sound(y, Fs)

%% Compression

y_0_2 = Compression(voix, 0, 2);
y_0_4 = Compression(voix, 0, 4);
y_02_2 = Compression(voix, 0.2, 2);
y_06_2 = Compression(voix, 0.5, 2);

figure
subplot(221)
plot(axis_naive, voix, 'b', axis_naive, y_0_2, 'r')
title('thresold = 0 et reduction du gain à de 2')
subplot(222)
plot(axis_naive, voix, 'b', axis_naive, y_0_4, 'r')
title('thresold = 0 et reduction du gain à de 4')
subplot(223)
plot(axis_naive, voix, 'b', axis_naive, y_02_2, 'r')
title('thresold = 0.2 et reduction du gain à de 2')
subplot(224)
plot(axis_naive, voix, 'b', axis_naive, y_06_2, 'r')
title('thresold = 0.5 et reduction du gain à de 2')
