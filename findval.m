function  idx = findidx(data, target)

% Loop para achar o indice do valor mais próximo ao target no vetor data.
for k = 0.5:-0.001: 0.001
    idx = find(abs(data-target)<k);
    if length(idx) == 1
        break
    end
end
end