classdef GeneralizedPredictiveController
    properties
        nit
        Ny
        Nu
        lambda
        ts
        Am
        Bm
        ref
        E
        G
        H
        gt
        Kgpc
        du
        u
        F
    end
    
    methods
        function obj = GeneralizedPredictiveController(nit, Ny, Nu, lambda, ts, Am, Bm)
            obj.nit = nit;
            obj.Ny = Ny;
            obj.Nu = Nu;
            obj.lambda = lambda;
            obj.ts = ts;
            obj.Am = Am;
            obj.Bm = Bm;
            na = length(Am);
            obj.u = zeros(1,nit);
            obj.du = zeros(1,nit);
            obj.E = zeros(1, Ny);
            obj.G = zeros(Ny, Nu);
            obj.H = zeros(Ny,1);
            obj.gt = zeros(Ny, Ny);
            obj.Kgpc = zeros(1, Ny);
            obj.F = zeros(Ny, na);
        end
        
        function obj = calculateController(obj)
            ny = obj.Ny;
            nu = obj.Nu;
            delta = [1, -1];
            Atil = conv(obj.Am, delta);
            natil = length(Atil);
            obj.F = zeros(ny, natil - 1);
            nb = length(obj.Bm)-1;
            G_aux = zeros(ny,nu);
            
            % Cálculo dos matrizes do preditor (F, H, G)
            % Primeira equação diophantine

            rr = [1, zeros(1, natil - 1)];
            q = zeros(ny, 1);

            for k = 1:ny
                [q(k), rr] = deconv(rr, Atil);
                obj.F(k, :) = rr(2:end);
                disp(obj.F)
            end

            for j = 1:ny
                if (j==1)
                    aux = [q(j) zeros(1,ny-j)];
                else
                    aux = [q(1:j) zeros(1,ny-j)];
                    
                end
                obj.E(j, :) = aux;
            end

            % Primeira equação diophantine
            B_aux = conv(obj.Bm, obj.E(end, :));
            nb_aux = length(B_aux);
            T_BE = [1, zeros(1, nb_aux - 1)]; % Polinomio C
            rr_BE = B_aux;

            for k = 1:ny
                [q_BE, r_BE] = deconv([rr_BE(2:end), 0], T_BE);
                G_aux(k) = q_BE;
                rr_BE = r_BE;
            end


            for i = 1:nu
                obj.G(i:end,i) = G_aux(1:end-i+1,1);
            end

            for i = 1:ny
                Haux(i,:) = conv(obj.E(i,:), [obj.Bm(2:end), zeros(1, ny)]);
            end


            obj.H =  zeros(ny,1);

            for i = 1:ny
                obj.H(i,:) = Haux(i,i+1:i+1+nb-1-1);
            end

            obj.gt = (obj.G' * obj.G + obj.lambda * eye(obj.Nu)) \ obj.G';
            obj.Kgpc = obj.gt(1, :);

        end
        
       
    end
end
