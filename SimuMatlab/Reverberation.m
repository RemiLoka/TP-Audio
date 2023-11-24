clear all;

[voix, Fs] = audioread('/Users/remi/Desktop/TP-Audio/SimuMatlab/DataAudio/Voix1.wav');

% Naive Realisation

seuil = [0.0001,0.001,0.01,0.1,0.5];
voixSeuil = zeros(length(seuil),length(voix));
valueRemove = zeros(length(seuil),1);

for n = 1:length(seuil)
    [voixSeuil(n,:), valueRemove(n,:)] = NaiveRealisation(voix,seuil(n));
end

% sound(voixSeuil(3,:))

% Réaliation avancée

[y, filtre] = AdvanceRealisation(voix, 0.01);

N = length(y);
sample = 1:N;

figure
subplot(211)
plot(sample,voix,'b', sample,y,'r');
subplot(212)
plot(filtre)

sound(y, Fs)