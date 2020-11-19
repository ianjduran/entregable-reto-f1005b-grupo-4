classdef ZonasDeDerrape
    methods (Static)
        % Devuelve un array con las posiciones en X de todos los puntos que
        % tienen un radio de curvatura igual a 50
        function [puntos] = xRadio50(eqn)
            syms x
            coeficientes = coeffs(eqn);
            a1 = coeficientes(2);
            a2 = coeficientes(3);
            a3 = coeficientes(4);
            % Derivada de la función de la curva
            funcionPrima = @(x) 3*a3*x.^2 + 2*a2*x + a1; 
            % Segunda derivada de la función de la curva
            funcion2Prima = @(x) 6*a3*x + 2*a2;
            funcionRadioDeCurva = @(x) (((1+funcionPrima(x).^2))^(3/2))...
                /((funcion2Prima(x)).^2).^(1/2) == 50;
            puntosConRepetidos = solve(funcionRadioDeCurva, x);
            puntosConRepetidos = double(puntosConRepetidos);
            puntos = unique(puntosConRepetidos);
        end
    end
end