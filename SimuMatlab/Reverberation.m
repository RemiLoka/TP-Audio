clear all;

[voix, Fs] = audioread('/Users/remi/Desktop/TP-Audio/SimuMatlab/DataAudio/s1.wav');

t = 1:length(voix);
noise = 0.02 * randn(size(t));
noise = noise';

%% Naive Realisation

seuil = [0.0001,0.001,0.01,0.1,0.5];
voixSeuil = zeros(length(seuil),length(voix));
valueRemove = zeros(length(seuil),1);

for n = 1:length(seuil)
    [voixSeuil(n,:), valueRemove(n,:)] = NaiveRealisation(voix,seuil(n));
end

% sound(voixSeuil(3,:))

%% Réaliation avancée

seuil = 0.05;
window_size = 8000;
slope = 500;
hold = 8000;

%voix = voix(1:30000);

[y, filtre, energie] = AdvanceRealisation(voix + noise, seuil, window_size, slope, hold);

N = length(y);
sample = 1:N;

figure
subplot(211)
plot(sample,voix,'b', sample,y,'r');
subplot(212)
plot(filtre)

sound(y, Fs)

%% Compression

