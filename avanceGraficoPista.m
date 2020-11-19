%% AUTORES GRUPO 4
%{
    Ian Javier Duran Román - A00829799
    Santiago Andrés Serrano Vacca - A01734988
    Alan Antonio De la Cruz Téllez - A01280638
    Josué Caleb Acosta de la Rosa - A01383048
    Maximiliano Zerecero Rodríguez - A00830773
%}

%% Inicialización

%Inicialización de Matlab
clc
clear
close all

% Modelo de ecuación: ax^3 + bx^2 + cx + d = y

% Inicialización de puntos y variable para manejar el ciclo
X0 = 10;
Y0 = 290;
XF = 280;
YF = 120;
ecuationFound = false;

%% CALCULO DE COORDENADAS
while(ecuationFound==false)
    
    % Se hallan coordenadas aleatorias para el primer punto medio
    X1 = abs(randi([X0 XF],1));
    Y1 = abs(randi([1 500],1));
    % Se hallan coordenadas aleatorias para el segundo punto medio
    X2 = abs(randi([X0 XF],1));
    Y2 = abs(randi([1 500],1));
    
    % Se forma el sistema de ecuaciones mediante matrices
    ec1 = [X0^3, X0^2, X0, 1];
    ec2 = [X1^3, X1^2, X1, 1];
    ec3 = [X2^3, X2^2, X2, 1];
    ec4 = [XF^3, XF^2, XF, 1];
    
    % Se crean matrices para resolver un sistema de ecuaciones
    
    matA = [ec1;ec2;ec3;ec4];
    resultados = [Y0;Y1;Y2;YF];
    
    % Si se usan estos tres se imprime: Warning: Matrix is singular to working precision
    % coef = inv(matA)*resultados;
    % coef = matA \ resultados;
    % coef = mldivide(matA, resultados);
    
    coef = pinv(matA)*resultados;
        
    % Se crea una funcion simbólica de la curva
    syms x
    eqn = coef(1)*x^3 + coef(2)*x^2 + coef(3)*x + coef(4);
    
    % Se determina los puntos críticos de la función
    puntosCriticos = double(solve(diff(eqn,x)));
    
    % Se calcula la longitud de arco y el radio de curvatura de la función
    arco = calc_arc(eqn, X0, XF);
    radio = calc_radius(eqn,puntosCriticos(1));
    
    % Se verifica si la ecuacion cumple con las condiciones necesarias
         % Curvatura de radio < 50
         % 300 < longitud de arco < 500
    if(radio<50 && (arco> 300 && arco<500))
        disp("¡Coordenadas encontradas!");
        ecuationFound = true;
    else
        disp("No se hallaron las coordenadas");
        ecuationFound = false;
    end
end

%% IMPRESIÓN DE DATOS IMPORTANTES

disp("Ecuacion:");
disp(eqn);
disp("Lista de coeficientes:");
fprintf("a3: %.10f\n", coef(1));
fprintf("a2: %.7f\n", coef(2));
fprintf("a1: %f\n", coef(3));
fprintf("a0: %f\n\n", coef(4));
disp("Longitud:");
disp(arco);
disp("Puntos Críticos:");
disp(double(puntosCriticos));
disp("Radio de Curvatura:");
disp(radio);
disp("Coordenadas: ")
disp("(" + num2str(X1) + "," + num2str(Y1)+ ")");
disp("(" + num2str(X2) + "," + num2str(Y2)+ ")");

%% GRAFICACION DE LA CURVA

% Se grafica la curva, con limites del punto inicial y final
fplot(eqn, [X0 XF])

% Se establecen los límites de la figura
%axis([(X0-50) (XF+50) 0 400])
axis equal
ylim([-40,400])
xlim([-40,400])

hold on

% Se dibujan los puntos de las coordenadas 
plot(X0,Y0, 'ro');
plot(XF,YF, 'ko');
plot(X1,Y1, 'bo');
plot(X2,Y2, 'bo');

%% DIBUJO DE CIRCULOS SOBREPUESTOS EN LA GRÁFICA

% Convierte la función simbólica en una función anónima
ecuacionCurva = matlabFunction(eqn);

% Se evalua la ecuación en los puntos críticos para obtener la coordenada
% de los centros de los círculos (Se obtiene el centro en el punto crítico,
% por lo que es necesario recorrer el centro)
xCentro1 = puntosCriticos(1);
xCentro2 = puntosCriticos(2);
yCentro1 = ecuacionCurva(xCentro1);
yCentro2 = ecuacionCurva(xCentro2);

% Recorre el centro para que el circulo pueda describir la curva de la
% ecuación, sin esto el centro del círculo se encontraría en los puntos
% mínimo y máximo locales de la ecuación
if(yCentro1 < yCentro2)
    yCentro1 = yCentro1 + radio;
    yCentro2 = yCentro2 - radio;
else
    yCentro1 = yCentro1 - radio;
    yCentro2 = yCentro2 + radio;
end

% Se calculan las variables para poder graficar un círculo.
th = 0: 0.001 : (2*pi);
xunit = radio * cos(th);
yunit = radio * sin(th);

%Se grafican los círculos trasladados, solo se grafica si el círculo se
%encuentra dentro de los límites de la curva
if(xCentro1>X0 && xCentro1<XF)
    plot(xunit+xCentro1, yunit+yCentro1,'g');
end
if(xCentro2>X0 && xCentro2 < XF)
    plot(xunit+xCentro2, yunit+yCentro2,'g');
end

hold off

% Se escribe la leyenda de la gráfica
legend('Curva','Punto inicial','Punto final','Punto intermedio', ...
       'Punto intermedio','Circulos que definen la curvatura')


% Puede escribir los resultados en un archivo
%{
fileID = fopen('coefs.txt','w');
fprintf(fileID,'%s\n',eqn);
fclose(fileID);
%}

%% DEFINICION DE FUNCIONES

% Funcion que calcula la longitud de una ecuacion dados sus limites 
function arc = calc_arc(func,a,b)
    arc = integral(matlabFunction(sqrt(1+diff(func)^2)), a, b);
end

% Funcion que determina el radio de curvatura de una ecuacion dado un punto
% de la ecuacion
function radius = calc_radius (func, punto)
    radioCurvatura = matlabFunction(((1 + diff(func))^(3/2) ) / diff(diff(func)));
    radius = abs(radioCurvatura(punto));
end
