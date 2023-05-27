syms a1 a2 a3 b0 b1
assume([a1 a2 a3],'real')
vetor1 = [1 a1 a2 0];
vetor2 = sym([1 -1]);
vetor3 = [b0 b1];
tamanho_resultado = length(vetor1) + length(vetor2) - 2;
    resultado = sym(zeros(1, tamanho_resultado));

    for i = 1:tamanho_resultado
        for j = 1:min(i, length(vetor2))
            resultado(i) = resultado(i) + vetor1(i-j+1) * vetor2(j);
        end
    end
% 
% vetor1 = resultado;
% vetor2 = [b0 b1];
% 
%     tamanho_resultado = length(vetor1) + length(vetor2) - 1;
%     matriz = sym(zeros(length(vetor1)+1, tamanho_resultado));
% 
%     for i = 1:length(vetor1)
%         for j = 1:length(vetor2)
%             matriz(i, i+j-1) = vetor2(j);
%         end
%     end
% 
%     matriz(:, 2:end) = [vetor1' matriz(:, 2:end)];

    disp(matriz);


 vetor = resultado   
for i = 1:length(vetor)
    coluna = [];
    for j = 1:i
        coluna = [coluna; vetor(j)];
    end
    coluna = [coluna; vetor(i)];
    matriz = [matriz coluna];
end

disp(matriz);