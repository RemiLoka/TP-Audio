function [y, filtre, moy] = AdvanceRealisation(voix, seuil)

num_samples = length(voix);
energie_voix = voix.*voix;
y = zeros(num_samples,1);
filtre = zeros(1,num_samples);

% Filtre
size_filter = 500;
slope = 1/1000;

state = 1;
hold = 0;

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
            filtre(n) = 1;
        else
            filtre(n) = filtre(n-1) + slope;
        end
    elseif state == 3
        filtre(n) = 1;
        if moy < seuil
            state = 4;
        end
    elseif state == 4
        if filtre(n) <= 0.5
            hold = hold + 1;
            filtre(n) = 0.5;
            if hold == 500
                state = 5;
            end
        else
            filtre(n) = filtre(n-1) - slope/4;
        end
    else
        hold = 0;
        if filtre(n-1) <= 0
            state = 1;
        else
            filtre(n) = filtre(n-1) - slope;
        end
    end
    y(n) = filtre(n)*voix(n);
end