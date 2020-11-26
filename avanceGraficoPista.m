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
        fprintf("\n");
        ecuationFound = true;
    else
        disp("No se hallaron las coordenadas");
        ecuationFound = false;
    end
end

%% IMPRESIÓN DE DATOS IMPORTANTES

disp("Ecuación de la pista");
disp(ecuacion_modelo);
disp("Lista de coeficientes:");
fprintf("a3: %.10f\n", coef(1));
fprintf("a2: %.7f\n", coef(2));
fprintf("a1: %f\n", coef(3));
fprintf("a0: %f\n\n", coef(4));
disp("Longitud:");
disp(arco);
disp("Puntos Críticos:");
disp("Mínimo Función: " + num2str(punto_min_max_1) )
disp("Máximo Función: " + num2str(punto_min_max_2) )
fprintf("\n");
disp("Mínimo radio de curvatura:");
disp(radio);
disp("Coordenadas de puntos intermedios: ")
disp("(" + num2str(X1) + "," + num2str(Y1)+ ")");
disp("(" + num2str(X2) + "," + num2str(Y2)+ ")");
fprintf("\n");

%% GRAFICACION DE LA CURVA

% Se grafica la curva, con limites del punto inicial y final
fplot(ecuacion_modelo, [X0 XF], 'lineWidth', 1, "DisplayName", "Curva");

% Se establecen los límites de la figura
%axis([(X0-50) (XF+50) 0 400])
axis equal
ylim([-40,400])
xlim([-40,400])

hold on

% Se dibujan los puntos de las coordenadas 
plot(X0,Y0, 'ro', "DisplayName", "Punto inicial");
plot(XF,YF, 'bo', "DisplayName", "Punto final");
plot(X1,Y1, 'ko', "DisplayName", "Punto intermedio");
plot(X2,Y2, 'ko', "DisplayName", "Punto intermedio");



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
    plot(xunit+xCentro1, yunit+yCentro1,'g', "DisplayName", "Circulo de curvatura");
end
if(xCentro2>X0 && xCentro2 < XF)
    plot(xunit+xCentro2, yunit+yCentro2,'g', "DisplayName", "Circulo de curvatura");
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
    X0, punto_min_max_1, punto_min_max_2, 1);
    plot(zonaDerrapeX1,zonaDerrapeY1, 'r','LineWidth',2, "DisplayName", "Zona de derrape");
end

% SEGUDNA ZONA DE DERRAPE

% Inicialización de la lista de coordenadas
zonaDerrapeX2 = [];
zonaDerrapeY2 = [];

if(punto_min_max_2<=XF)
     [zonaDerrapeX2, zonaDerrapeY2] = hallar_zona_derrape(ecuacion_modelo, ...
     XF, punto_min_max_2, punto_min_max_1, -1);
    plot(zonaDerrapeX2,zonaDerrapeY2, 'r','LineWidth',2, "DisplayName", "Zona de derrape");
end

% Se despliega información sobre la cantidad de coordenadas que existe en
% las zonas de derrape
disp("Cantidad de coordenadas halladas por zona de derrape (método de Euler):")
disp("Zona Derrape 1:")
disp(length(zonaDerrapeX1));
disp("Zona Derrape 2: ")
disp(length(zonaDerrapeX2));

modificador_tamanio_tangente = 4;
distGradasY=20;
distGradasX=-20;
if(~isempty(zonaDerrapeX1))
    dCurva=matlabFunction(diff(ecuacion_modelo));
    slope = dCurva(zonaDerrapeX1(1));
    slopeInv =-1/slope;
    x_primer_punto_critico = 0:(XF-X0)/modificador_tamanio_tangente;
    y_primer_punto_critico = (slope*x_primer_punto_critico);
    plot(x_primer_punto_critico+zonaDerrapeX1(1),y_primer_punto_critico+zonaDerrapeY1(1), "DisplayName", "Tangente", "Color", [0.59 0.57 0])
    
    if(zonaDerrapeY1(1)< zonaDerrapeY2(1))
        distGradasY = distGradasY *( -1);
    end
    %Encontrar las 4 esquinas de las gradas
    %punto 1
    [xa,ya]=obtenerPuntosGradas(zonaDerrapeX1(1),zonaDerrapeY1(1),20, slopeInv, false);
    x11(1)=xa;
    y11(1)=ya;
    %punto2
   [xa,ya]=obtenerPuntosGradas(x11(1),y11(1),10, slopeInv,false);
   x11(2)=xa;
    y11(2)=ya;
    %punto 3
    [xa,ya]=obtenerPuntosGradas(x11(2),y11(2),80, slope,true);
    x11(3)=xa;
    y11(3)=ya;
    %punto 4
   [xa,ya]=obtenerPuntosGradas(x11(3),y11(3),10, slopeInv,true);
   x11(4)=xa;
    y11(4)=ya;
    
    line(x11, y11, "HandleVisibility", "Off");
    hold on;
    fill(x11, y11, 'b', "DisplayName", "Gradas");
    
    disp("Ecuacion recta tangente 1: "+slope+"x + "+(zonaDerrapeY1(1)-slope*zonaDerrapeX1(1)));
    
    %plot(x_primer_punto_critico+zonaDerrapeX1(1)+ distGradasX,y_primer_punto_critico+zonaDerrapeY1(1) + distGradasY,'Color', [0.5 0.5 0.5] , 'lineWidth', 8)
    
end

if(~isempty(zonaDerrapeX2)>0)
    dCurva=matlabFunction(diff(ecuacion_modelo));
    slope = dCurva(zonaDerrapeX2(1));
    slopeInv =-1/slope;
    x_segundo_punto_critico = 0:(XF-X0)/modificador_tamanio_tangente;
    y_segundo_punto_critico = (slope*x_primer_punto_critico);
    plot(x_segundo_punto_critico+zonaDerrapeX2(1),y_segundo_punto_critico+zonaDerrapeY2(1), "DisplayName", "Tangente", "Color", [0.59 0.57 0]);
    distGradas=20;
    
    
    %Encontrar las 4 esquinas de las gradas
    %punto 1
    [xa,ya]=obtenerPuntosGradas(zonaDerrapeX2(1),zonaDerrapeY2(1),20, slopeInv, false);
    x12(1)=xa;
    y12(1)=ya;
    %punto2
   [xa,ya]=obtenerPuntosGradas(x12(1),y12(1),10, slopeInv,false);
   x12(2)=xa;
    y12(2)=ya;
    %punto 3
    [xa,ya]=obtenerPuntosGradas(x12(2),y12(2),80, slope,true);
    x12(3)=xa;
    y12(3)=ya;
    %punto 4
   [xa,ya]=obtenerPuntosGradas(x12(3),y12(3),10, slopeInv,true);
   x12(4)=xa;
    y12(4)=ya;
    
    line(x12, y12, "HandleVisibility", "Off");
    hold on;
    fill(x12, y12, 'b', "DisplayName", "Gradas");
    
    disp("Ecuacion recta tangente 2: "+slope+"x + "+(zonaDerrapeY2(1)-slope*zonaDerrapeX2(1)));
    
    xRectTang = 140; % La recta tangente se calculará con este valor de x, puede cambiarse
    [rectaTangenteXPos, m, b] = getRectaTangente(ecuacion_modelo, xRectTang);
    fprintf("\nLa recta tangente al punto donde x = %f es: \ny = %fx + (%f)", xRectTang, m, b);
    fprintf("\nEl punto x = %f se puede cambiar en código para hallar la recta tangente en cualquier punto de la pista. Es la variable xRectTang.", xRectTang);
end

hold off

% Se escribe la leyenda de la gráfica
legend('Location','northeastoutside')


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
                               punto_i, punto_intermedio, punto_f, paso)
    % Convierte la ecuacion para evaluarla en distintos puntos
    funcion_curva = matlabFunction(eqn);
    % Se inicializa la lista;
    lista_x = [];
    % Se recorre un ciclo que recorre de manera inversa los puntos, para
    % hallar desde el lado izquierdo el area de curvatura.
    for punto = punto_i:paso:punto_intermedio
        if(calc_radius(eqn,punto)<=50)
            if(paso>0)
                lista_x = [lista_x punto];
            else
                lista_x = [punto lista_x];
            end
        end
    end
    for punto = punto_intermedio:paso:punto_f
        % Se añade un elemento al principio de la lista
        if(paso>0)
                lista_x = [lista_x punto];
        else
                lista_x = [punto lista_x];
        end
        % Se deja de registrar los elementos que tengan una curvatura
        % mayor a 50
        if(calc_radius(eqn,punto)>50)
            break;
        end
    end
    lista_coordenadas_x = lista_x;
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

function [xa,ya]=obtenerPuntosGradas(xInicial,yInicial,distanciaTotal, slope, derecha)
   
    c=yInicial-slope*xInicial;
    
    apoyo=xInicial;
    
    distancia=sqrt((xInicial-apoyo)^2+(yInicial-(slope*apoyo+c))^2);
    while (distancia<distanciaTotal)
        distancia=sqrt((xInicial-apoyo)^2+(yInicial-(slope*apoyo+c))^2);
        if (derecha==true)
            
            apoyo=apoyo+0.1;
        end
        if (derecha==false)
            
            apoyo=apoyo-0.1;
        end
    end
    if (derecha==true)
        xa=apoyo-0.1;
    end
    if (derecha==false)
        xa=apoyo+0.1;
    end
        ya=slope*xa+c;
end

function [rectaTangente, m, b] = getRectaTangente(curva, xPos)
    syms x
    coeficientes = coeffs(curva);
    a1 = coeficientes(2);
    a2 = coeficientes(3);
    a3 = coeficientes(4);
    % Derivada de la función de la curva
    funcionPrima = @(x) 3*a3*x.^2 + 2*a2*x + a1;
    % La recta es de la forma y = mx + b, hallamos b:
    curvaMatlabFunction = matlabFunction(curva);
    yPos = curvaMatlabFunction(xPos);
    b = yPos - 3*a3*xPos.^3 - 2*a2*xPos.^2 - a1*xPos;
    rectaTangente = funcionPrima(xPos)*x + b;
    coefsRectaTangente = coeffs(rectaTangente);
    b = coefsRectaTangente(1);
    m = coefsRectaTangente(2);
end
