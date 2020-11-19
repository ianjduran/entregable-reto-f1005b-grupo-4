% Se pueden cambiar los puntos aquí:
puntoInicial = [10, 290];
puntoFinal = [280, 120];
puntoMedio1 = [80, 320];
puntoMedio2 = [200, 100];

% -----------------------------CÓDIGO MAIN--------------------------------:
clc; % Para limpiar la consola.

% Obtenemos la función cúbica que pasa por los 4 puntos, además de sus 4 coeficientes.
[funcion, a3, a2, a1, a0] = ...
    obtenerCubica(puntoInicial, puntoFinal, puntoMedio1, puntoMedio2);

% Obtenemos la longitud de la curva, desde el punto inicial hasta el final.
longitudFuncion = ...
    longitudCubica(a3, a2, a1, puntoInicial(1), puntoFinal(1));

% Obtenemos el radio de curvatura más bajo de toda la función, además de los dos puntos donde se manifiesta.
[minRadioCurva, coordXMinRadio, coordXMinRadio2] ...
    = minRadioDeCurva(a3, a2, a1, funcion);

% Por medio de expresiones lógicas verificamos que se cumplan las condiciones establecidas para la pista, y mostramos en pantalla.
fprintf("\n\n<strong>¿Cumple con las condiciones?</strong>\n");
if abs(funcion(puntoInicial(1)) - puntoInicial(2)) < 0.1 && ...
abs(funcion(puntoFinal(1)) - puntoFinal(2)) < 0.1
    fprintf("Inicia y termina en puntos designados: " + ...
        "<strong>SÍ</strong>\n");
else
    fprintf("Inicia y termina en puntos designados: " + ...
        "<strong>NO</strong>\n");
end
if longitudFuncion > 300 && longitudFuncion < 500
    fprintf("Longitud de arco: <strong>SÍ</strong>\n");
else
    fprintf("Longitud de arco: <strong>NO</strong>\n");
end
if minRadioCurva < 50 && coordXMinRadio ~= puntoInicial(1) && ...
coordXMinRadio ~= puntoFinal(1)
    fprintf("Radio de curvatura: <strong>SÍ</strong>\n");
else
    fprintf("Radio de curvatura: <strong>NO</strong>\n");
end

clf; % Limpiamos el espacio donde se graficará la pista y las otras cosas.
hold on; % Permitimos poner varias cosas en el mismo plot.
fplot(funcion, [puntoInicial(1), puntoFinal(1)]); % Ploteamos la función únicamente desde el punto inicial hasta el final.
axis equal; % Para que los dos ejes tengan la misma escala (es decir, que la gráfica no se "estire" en X o en Y).
xlim([puntoInicial(1) - 20, puntoFinal(1) + 20]); % Hacemos un pequeño "zoom out" para que las cosas no se vean en los bordes.
limitesGraficaY = ylim;
ylim([limitesGraficaY(1) - 20, limitesGraficaY(2) + 20]); % "Zoom out" pero en el eje Y.
puntoIncial = plot(puntoInicial(1), puntoInicial(2), 'O', "LineWidth", 6); % Ponemos el punto inicial.
puntoFinal = plot(puntoFinal(1), puntoFinal(2), 'O', "LineWidth", 6); % Ponemos el punto final.

funcion2Prima = @(x) 6*a3*x + 2*a2; % La segunda derivada de la función de la pista. Aquí nos sirve para conocer su concavidad.

% Dependiendo de la concavidad de la función, sabremos si hay que plotear la circunferencia superpuesta debajo o encima de la curva.
% Ploteamos el círculo superpuesto del primer punto.
if(funcion2Prima(coordXMinRadio) > 0)
    plotearDebajo = false;
else
    plotearDebajo = true;
end
plotCirculoSuperpuesto(coordXMinRadio, funcion(coordXMinRadio), ...
    minRadioCurva, plotearDebajo);

% Ploteamos el círculo superpuesto del segundo punto.
if(funcion2Prima(coordXMinRadio2) > 0)
    plotearDebajo = false;
else
    plotearDebajo = true;
end
plotCirculoSuperpuesto(coordXMinRadio2, funcion(coordXMinRadio2), ...
    minRadioCurva, plotearDebajo);

% Nada más para que los nombres de la curva y los puntos salgan en el plot.
legend('Pista', 'Punto Inicial', 'Punto Final');
    

% ---------------------------------FUNCIONES------------------------------:

% Obtenemos la función cúbica que pasa por los 4 puntos, además de los coeficientes de esta, utilizando matrices.
function [cubica, a3, a2, a1, a0] = ...
obtenerCubica(punto1, punto2, punto3, punto4)
    matrizA = [punto1(1)^3, punto1(1)^2, punto1(1), 1;...
                punto2(1)^3, punto2(1)^2, punto2(1), 1;...
                punto3(1)^3, punto3(1)^2, punto3(1), 1;...
                punto4(1)^3, punto4(1)^2, punto4(1), 1;];
    
    matrizB = [punto1(2); punto2(2); punto3(2); punto4(2)];
    
    matrizX = matrizA\matrizB; % https://www.mathworks.com/help/matlab/ref/mldivide.html
    
    a3 = matrizX(1);
    a2 = matrizX(2);
    a1 = matrizX(3);
    a0 = matrizX(4);
    
    fprintf("La función es: " + ...
        "<strong>y = (%.10fx³) + (%.7fx²) + (%fx) + (%f)</strong>\n\n", ...
        a3, a2, a1, a0);
    fprintf("<strong>Coeficientes:</strong>\n");
    fprintf("a3: %.10f\n", a3);
    fprintf("a2: %.7f\n", a2);
    fprintf("a1: %f\n", a1);
    fprintf("a0: %f\n\n", a0);
    cubica = @(x) a3*x.^3 + a2*x.^2 + a1*x + a0;
end

% Obtenemos la longitud de la curva desde xMin hasta xMax utilizando la fórmula: https://calcworkshop.com/wp-content/uploads/arc-length-function.png
function [longitud] = longitudCubica(a3, a2, a1, xMin, xMax)
    funcionPrima = @(x) 3*a3*x.^2 + 2*a2*x + a1; % Derivada de la función de la curva
    funcionAIntegrar = @(x) (1 + funcionPrima(x).^2).^(1/2);
    longitud = integral(funcionAIntegrar, xMin, xMax);
    fprintf("La longitud de arco de la función, desde el punto " + ...
        "incial al final es: <strong>%f</strong>\n", longitud);
end

% Obtenemos el mínimo radio de curva de toda la función y devolvemos las coords en X de los dos puntos donde está este radio de curva
function [minRadio, coordXMinRadio, coordXMinRadio2] = ...
minRadioDeCurva(a3, a2, a1, funcion)
    funcionPrima = @(x) 3*a3*x.^2 + 2*a2*x + a1; % Derivada de la función de la curva
    funcion2Prima = @(x) 6*a3*x + 2*a2; % Segunda derivada de la función de la curva
    funcionRadioDeCurva = @(x) (((1+funcionPrima(x).^2))^(3/2))...
        /((funcion2Prima(x)).^2).^(1/2); % Mediante esta función podemos hallar el radio de curva en un punto X de nuestra función. Esta es la función original: https://d2vlcm61l7u1fs.cloudfront.net/media%2F1ef%2F1efd28e6-1fae-4756-99f5-d49052cb3a62%2FphpLe2rED.png
    funcionRadioDeCurvaInvertida = @(x) ...
        (((1+funcionPrima(-x).^2))^(3/2))/((funcion2Prima(-x)).^2).^(1/2); % Es la reflexión sobre el eje Y de funcionRadioDeCurva. Más abajo veremos para qué la quiero.
    coordXMinRadio = fminsearch(funcionRadioDeCurva, 0); % Hallamos el mínimo de la función del radio de curva (bueno, de hecho la coordenada en X donde está este mínimo)
    coordXMinRadio2 = -fminsearch(funcionRadioDeCurvaInvertida, -1000); % Hallamos el segundo mínimo de la función del radio de curva. Utilizamos la reflexión sobre el eje Y esta vez para que primero salga el segundo mínimo (ya que esta función de Matlab nos da el primer mínimo que encuentra).
    minRadio = funcionRadioDeCurva(coordXMinRadio); % El mínimo radio de curva.
    fprintf("La longitud del mínimo radio de curva es de: " + ...
        "<strong>%f</strong>, y ocurre en los puntos <strong>" + ...
        "(%f, %f)</strong> y <strong>(%f, %f)" + ...
        "</strong>", minRadio, coordXMinRadio, funcion(coordXMinRadio),...
        coordXMinRadio2, funcion(coordXMinRadio2));
end

% Ploteamos un círculo que, dependiendo de si "debajo" es true o false, aparecerá arriba o abajo de la curva
% Es necesario que hayan dos funciones (para la parte de arriba y de abajo del círculo) ya que, técnicamente, la ecuación del círculo no es una función (devuelve dos valores para un X determinado)
function plotCirculoSuperpuesto(coordX, coordY, radio, debajo)
    if debajo
        funcionCirculoPos = @(x) ...
            sqrt(radio^2 - x.^2 + 2*coordX*x - coordX.^2) + coordY - radio;
        funcionCirculoNeg = @(x) ...
            -sqrt(radio^2 - x.^2 + 2*coordX*x - coordX.^2) + coordY -radio;
    else
        funcionCirculoPos = @(x) ...
            sqrt(radio^2 - x.^2 + 2*coordX*x - coordX.^2) + coordY + radio;
        funcionCirculoNeg = @(x) ...
            -sqrt(radio^2 - x.^2 + 2*coordX*x - coordX.^2) + coordY +radio;
    end
    
    fplot(funcionCirculoNeg, "Color", "red"); % Ploteamos las funciones.
    fplot(funcionCirculoPos, "Color", "red");
end
