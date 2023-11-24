function [y, filtre, moy] = AdvanceRealisation(voix, seuil)

num_samples = length(voix);
energie_voix = voix.*voix;
y = zeros(1,num_samples);
filtre = zeros(1,num_samples);

% Filtre
size_filter = 20;

state = 1;

for n = size_filter+1:num_samples
    window = energie_voix(n-size_filter+1:n);
    moy = mean(window);
    
    if state == 1
        if moy > seuil
            state = 2;
        end
    elseif state == 2
        if filtre(n-1) >= 1
            state = 3;
        else
            filtre(n) = filtre(n-1) + 0.02;
        end
    elseif state == 3
        filtre(n) = 1;
        if moy < seuil
            state = 4;
        end
    else
        if filtre(n-1) <= 0
            state = 1;
        else
            filtre(n) = filtre(n-1) -0.02;
        end
    end
    y(n) = filtre(n)*voix(n);
end