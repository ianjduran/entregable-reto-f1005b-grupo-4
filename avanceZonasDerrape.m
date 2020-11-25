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
    ecuacion_modelo = coef(1)*x^3 + coef(2)*x^2 + coef(3)*x + coef(4);
    
    % Se determina los puntos críticos de la función
    [punto_min_max_1, punto_min_max_2] = find_min_max_points(ecuacion_modelo);
    
    % Se calcula la longitud de arco y el radio de curvatura de la función
    arco = calc_arc(ecuacion_modelo, X0, XF);
    radio = calc_radius(ecuacion_modelo,punto_min_max_1);
    
    % Se verifica si la ecuacion cumple con las condiciones necesarias
         % Curvatura de radio < 50
         % 300 < longitud de arco < 500
         % Ambos puntos criticos de la ecuacion se encuentran en el rango
         % de los puntos en X
        
    if(radio<50 && (arco> 300 && arco<500) && (min(punto_min_max_1,punto_min_max_2)-X0)>20 && (XF-max(punto_min_max_1,punto_min_max_2))>20)
        disp("¡Coordenadas encontradas!");
        ecuationFound = true;
    else
        disp("No se hallaron las coordenadas");
        ecuationFound = false;
    end
end

%% IMPRESIÓN DE DATOS IMPORTANTES

disp("Ecuacion:");
disp(ecuacion_modelo);
disp("Lista de coeficientes:");
fprintf("a3: %.10f\n", coef(1));
fprintf("a2: %.7f\n", coef(2));
fprintf("a1: %f\n", coef(3));
fprintf("a0: %f\n\n", coef(4));
disp("Longitud:");
disp(arco);
disp("Puntos Críticos:");
disp("Minimo Funcion: " + num2str(punto_min_max_1) )
disp("Maximo Funcion: " + num2str(punto_min_max_2) )

disp("Radio de Curvatura:");
disp(radio);
disp("Coordenadas: ")
disp("(" + num2str(X1) + "," + num2str(Y1)+ ")");
disp("(" + num2str(X2) + "," + num2str(Y2)+ ")");

%% GRAFICACION DE LA CURVA

% Se grafica la curva, con limites del punto inicial y final
fplot(ecuacion_modelo, [X0 XF], 'lineWidth', 1)

% Se establecen los límites de la figura
%axis([(X0-50) (XF+50) 0 400])
axis equal
ylim([-40,400])
xlim([-40,400])

hold on

% Se dibujan los puntos de las coordenadas 
plot(X0,Y0, 'ro');
plot(XF,YF, 'bo');
plot(X1,Y1, 'ko');
plot(X2,Y2, 'ko');



%% DIBUJO DE CIRCULOS SOBREPUESTOS EN LA GRÁFICA

% Convierte la función simbólica en una función anónima
ecuacionCurva = matlabFunction(ecuacion_modelo);

% Se evalua la ecuación en los puntos críticos para obtener la coordenada
% de los centros de los círculos (Se obtiene el centro en el punto crítico,
% por lo que es necesario recorrer el centro)
xCentro1 = punto_min_max_1;
xCentro2 = punto_min_max_2;
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

% Se convierte la ecuacion en una función anónima para evaluar la función
% en distintos puntos
funcionCurva = matlabFunction(ecuacion_modelo);

%% BUSQUEDA Y GRAFICACIÓN DE ZONAS DE DERRAPE

% PRIMERA ZONA DE DERRAPE

% Se inicializa la lista de coordenadas que tiene la primera zona de 
% derrape
zonaDerrapeX1 = [];
zonaDerrapeY1 = [];
% Se ejecuta si es que el punto mínimo local se encuentra dentro de la
% curva
if(punto_min_max_1>=X0)
    % Se considera hallar desde el primer punto crítico, al principio de la
    % curva, ya que el carro viene desde la izquierda.
    [zonaDerrapeX1, zonaDerrapeY1]= hallar_zona_derrape(ecuacion_modelo, ...
    punto_min_max_1, X0);
    plot(zonaDerrapeX1,zonaDerrapeY1, 'r','LineWidth',2);
end

% SEGUDNA ZONA DE DERRAPE

% Inicialización de la lista de coordenadas
zonaDerrapeX2 = [];
zonaDerrapeY2 = [];

if(punto_min_max_2<=XF)
     [zonaDerrapeX2, zonaDerrapeY2] = hallar_zona_derrape(ecuacion_modelo, ...
     punto_min_max_2, X0);
    plot(zonaDerrapeX2,zonaDerrapeY2, 'r','LineWidth',2);
end

% Se despliega información sobre la cantidad de coordenadas que existe en
% las zonas de derrape
disp("Zona Derrape 1:")
disp(length(zonaDerrapeX1));
disp("Zona Derrape2: ")
disp(length(zonaDerrapeX2));

modificador_tamanio_tangente = 4;
distGradasY=20;
distGradasX=-20;
if(~isempty(zonaDerrapeX1))
    dCurva=matlabFunction(diff(ecuacion_modelo));
    slope = dCurva(zonaDerrapeX1(1));
    x_primer_punto_critico = 0:(XF-X0)/modificador_tamanio_tangente;
    y_primer_punto_critico = (slope*x_primer_punto_critico);
    plot(x_primer_punto_critico+zonaDerrapeX1(1),y_primer_punto_critico+zonaDerrapeY1(1))
    
    if(zonaDerrapeY1(1)< zonaDerrapeY2(1))
        distGradasY = distGradasY *( -1);
    end
    plot(x_primer_punto_critico+zonaDerrapeX1(1)+ distGradasX,y_primer_punto_critico+zonaDerrapeY1(1) + distGradasY,'Color', [0.5 0.5 0.5] , 'lineWidth', 8)
    
end

if(~isempty(zonaDerrapeX2)>0)
    dCurva=matlabFunction(diff(ecuacion_modelo));
    slope = dCurva(zonaDerrapeX2(1));
    x_segundo_punto_critico = 0:(XF-X0)/modificador_tamanio_tangente;
    y_segundo_punto_critico = (slope*x_primer_punto_critico);
    plot(x_segundo_punto_critico+zonaDerrapeX2(1),y_segundo_punto_critico+zonaDerrapeY2(1))
    distGradas=20;
    
    
    plot(x_segundo_punto_critico+zonaDerrapeX2(1)+ distGradasX,y_segundo_punto_critico+zonaDerrapeY2(1) - distGradasY,'Color', [0.5 0.5 0.5] , 'lineWidth', 8)
end

hold off

% Se escribe la leyenda de la gráfica
legend('Curva','Punto inicial','Punto final','Punto intermedio', ...
       'Punto intermedio','Circulos que definen la curvatura')


% En caso que se requiera, se pueden almacenar los resultados en un archivo
%{
fileID = fopen('informacionRelevante.txt','w');
fprintf(fileID,'%s\n',eqn);
fclose(fileID);
%}

%% DEFINICION DE FUNCIONES

% Función que halla las listas de la coordenadas de la zona de derrape de
% la función

function [lista_coordenadas_x, lista_coordenadas_y] = ...
                               hallar_zona_derrape(eqn, ...
                               punto_final, punto_inicial)
    % Convierte la ecuacion para evaluarla en distintos puntos
    funcion_curva = matlabFunction(eqn);
    % Se inicializa la lista;
    lista_x = [];
    % Se recorre un ciclo que recorre de manera inversa los puntos, para
    % hallar desde el lado izquierdo el area de curvatura.
    for punto = punto_final:-1:punto_inicial
        % Se añade un elemento al principio de la lista
        lista_x = [punto lista_x];
        % Se deja de registrar los elementos que tengan una curvatura
        % mayor a 50
        if(calc_radius(eqn,punto)>50)
            lista_coordenadas_x = lista_x;
            break;
        end
    end
    % Se obtiene la lista de coordenadas en Y al evaluar la lista de
    % coordenadas en X
    if(~isempty(lista_x))
        lista_coordenadas_y = funcion_curva(lista_x);
    end
end

%Funcion que halla y devuelve los puntos maximos y minimos locales de la
% ecuación

function [punto_1, punto_2] = find_min_max_points(eqn)
    criticPoints = double(solve(diff(eqn)));
    punto_1 = criticPoints(1);
    punto_2 = criticPoints(2);
end

% Funcion que calcula la longitud de una ecuacion dados sus limites 
function arc = calc_arc(func,a,b)
    arc = integral(matlabFunction(sqrt(1+diff(func)^2)), a, b);
end

% Funcion que determina el radio de curvatura de una ecuacion dado un punto
% de la ecuacion
function radius = calc_radius (func, punto)
    radioCurvatura = matlabFunction(((1 + (diff(func)^2))^(3/2) ) / diff(diff(func)));
    radius = abs(radioCurvatura(punto));
end
