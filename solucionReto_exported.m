classdef solucionReto_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        GridLayout                      matlab.ui.container.GridLayout
        LeftPanel                       matlab.ui.container.Panel
        CrearcurvaButton                matlab.ui.control.Button
        VelocidadincialmsEditFieldLabel  matlab.ui.control.Label
        VelocidadincialmsEditField      matlab.ui.control.NumericEditField
        Puntoincial10290Label           matlab.ui.control.Label
        Puntofinal280120Label           matlab.ui.control.Label
        PuntoactualLabel                matlab.ui.control.Label
        LLuviaactivadaSwitchLabel       matlab.ui.control.Label
        LLuviaactivadaSwitch            matlab.ui.control.Switch
        CrearferrariButton              matlab.ui.control.Button
        Friccion08miuLabel              matlab.ui.control.Label
        CrearelferrarisolodespuesdecreadaydibujadalacurvaLabel  matlab.ui.control.Label
        VelocidadanimacionSliderLabel   matlab.ui.control.Label
        VelocidadanimacionSlider        matlab.ui.control.Slider
        VelmxpermitidaenpuntoactualLabel  matlab.ui.control.Label
        EstadoLabel                     matlab.ui.control.Label
        EnergiaPerdidaLabel             matlab.ui.control.Label
        MasadelferrarikgEditFieldLabel  matlab.ui.control.Label
        MasadelferrarikgEditField       matlab.ui.control.NumericEditField
        DistanciaderrapadaLabel         matlab.ui.control.Label
        RightPanel                      matlab.ui.container.Panel
        EcuacindelacurvaLabel           matlab.ui.control.Label
        PuntomaximoLabel                matlab.ui.control.Label
        PuntominimoLabel                matlab.ui.control.Label
        LongitudLabel                   matlab.ui.control.Label
        RadiodeCurvatura1Label          matlab.ui.control.Label
        EcuacionText                    matlab.ui.control.Label
        Zonadederrape1Label             matlab.ui.control.Label
        Zonadederrape2Label             matlab.ui.control.Label
        RadiodeCurvatura2Label          matlab.ui.control.Label
        VelmxpermitidaenzonadederrapeLabel  matlab.ui.control.Label
        UIAxes                          matlab.ui.control.UIAxes
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    
    methods (Access = private)
        
        %Encontrar los puntos maximos y minimos de la curva de la pista
        function [punto_1, punto_2] = find_min_max_points(app,eqn)
            criticPoints = double(solve(diff(eqn)));
            punto_1 = criticPoints(1);
            punto_2 = criticPoints(2);
        end
        
        %Encontrar el largo de la curva (largo de la pista) 
        function [arc] = calc_arc(app,func,a,b)
            arc = integral(matlabFunction(sqrt(1+diff(func)^2)), a, b);
            
        end
        
        %Radio de curvatura en un punto de la ecuacion
        function radius = calc_radius (app,func, punto)
            radioCurvatura = matlabFunction(((1 + (diff(func)^2))^(3/2) ) / diff(diff(func)));
            radius = abs(radioCurvatura(punto));
        end
        
        %Encuentra las zonas de derrape de la curva
        function [lista_coordenadas_x, lista_coordenadas_y] = ...
                                       hallar_zona_derrape(app,eqn, ...
                                       punto_i, punto_intermedio, punto_f, paso)
        
            % Convierte la ecuacion para evaluarla en distintos puntos
            funcion_curva = matlabFunction(eqn);
            % Se inicializa la lista;
            lista_x = [];
            % Se recorre un ciclo que recorre de manera inversa los puntos, para
            % hallar desde el lado izquierdo el area de curvatura.
            for punto = punto_i:paso:punto_intermedio
                if(calc_radius(app,eqn,punto)<=50)
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
                if(calc_radius(app,eqn,punto)>50)
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
        
        %Funcion de apoyo para dibujar las gradas.
        function [xa,ya]=obtenerPuntosGradas(app,xInicial,yInicial,distanciaTotal, slope, derecha)
   
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
        
        %obtener la pendiente de la tangente en x punto de la curva
        function [rectaTangente, m, b] = getRectaTangente(app,curva, xPos)
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
        
        %obtener la velocidad maxima permitida en x punto de la pista
        function [velocidad] = obtener_velocidad(app, eqn, x, coef_friccion, G)
            radio = calc_radius(app,eqn,x);
            velocidad = sqrt(radio*G*coef_friccion);
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: CrearcurvaButton
        function CrearcurvaButtonPushed(app, event)
            
            
            % Inicialización de puntos y variable para manejar el ciclo
            X0 = 10;
            Y0 = 290;
            XF = 280;
            YF = 120;
            
            ecuationFound = false;
            coef_friccion=0.8;
            
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
                x = sym('x');
                ecuacion_modelo = coef(1)*x^3 + coef(2)*x^2 + coef(3)*x + coef(4);
                
                % Se determina los puntos críticos de la función
                [punto_min_max_1, punto_min_max_2] = find_min_max_points(app, ecuacion_modelo);
                
                % Se calcula la longitud de arco y el radio de curvatura de la función
                arco = calc_arc(app,ecuacion_modelo, X0, XF);
                radio1 = calc_radius(app,ecuacion_modelo,punto_min_max_1);
                radio2 = calc_radius(app,ecuacion_modelo,punto_min_max_2);
                
                % Se verifica si la ecuacion cumple con las condiciones necesarias
                     % Curvatura de radio < 50
                     % 300 < longitud de arco < 500
                     % Ambos puntos criticos de la ecuacion se encuentran en el rango
                     % de los puntos en X
                if(radio1<50 && radio2<50 && (arco> 300 && arco<500) && (min(punto_min_max_1,punto_min_max_2)-X0)>20 && (XF-max(punto_min_max_1,punto_min_max_2))>20)
                    ecuationFound = true;
                else
                    app.EcuacindelacurvaLabel.Text="Ecuacion de la curva: Cargando...";
                    app.PuntomaximoLabel.Text= "Punto maximo: Cargando...";
                    app.PuntominimoLabel.Text="Punto minimo: Cargando...";
                    app.LongitudLabel.Text="Longitud: Cargando...";
                    app.RadiodeCurvatura1Label.Text="Radio de curvatura 1: Cargando...";
                    app.RadiodeCurvatura2Label.Text="Radio de curvatura 2: Cargando...";
                    app.Zonadederrape1Label.Text="Zona de derrape 1: Cargando...";
                    app.Zonadederrape2Label.Text="Zona de derrape 2: Cargando...";
                    app.VelmxpermitidaenzonadederrapeLabel.Text="Vel. máx. permitida en zona de derrape: Cargando...";
                    drawnow;
                    ecuationFound = false;
                end
            end
            
            %% IMPRESIÓN DE DATOS IMPORTANTES

            % Convierte la función simbólica en una función anónima
            ecuacionCurva = matlabFunction(ecuacion_modelo);
            
            %Minimos y maximos de la grafica
            xMin=punto_min_max_1;
            xMax=punto_min_max_2;
            yMin=ecuacionCurva(xMin);
            yMax=ecuacionCurva(xMax);
            
            if(yMin>yMax)
                xTemp=xMin;
                yTemp=yMin;
                xMin=xMax;
                yMin=yMax;
                xMax=xTemp;
                yMax=yTemp;
            end
            
            %actualizacion de las labels del UI
            app.EcuacindelacurvaLabel.Text="Ecuacion de la curva: ";
            app.EcuacionText.Text=char(ecuacion_modelo);
            app.LongitudLabel.Text="Longitud: "+num2str(arco);
            app.PuntomaximoLabel.Text= "Punto maximo: ("+num2str(xMax)+", "+num2str(yMax)+")";
            app.PuntominimoLabel.Text="Punto minimo: ("+num2str(xMin)+", "+num2str(yMin)+")";
            app.RadiodeCurvatura1Label.Text="Radio de curvatura 1: "+num2str(radio1);
            app.RadiodeCurvatura2Label.Text="Radio de curvatura 2: "+num2str(radio2);
            %drawnow;
            
            %% GRAFICACION DE LA CURVA
                                    
            % Se establecen los límites de la figura
            axis(app.UIAxes, [-40,... 
                              400,...
                              -40,... 
                              400])
            
            hold(app.UIAxes, 'on');
            
            % plot for the trajectory
            fplot(app.UIAxes, ecuacion_modelo, [X0 XF], 'lineWidth', 1, "DisplayName", "Curva");
            
            % Se dibujan los puntos de las coordenadas 
            plot(app.UIAxes,X0,Y0, 'ro', "DisplayName", "Punto inicial");
            plot(app.UIAxes,XF,YF, 'bo', "DisplayName", "Punto final");
            
          
         
                        
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
                [zonaDerrapeX1, zonaDerrapeY1]= hallar_zona_derrape(app, ecuacion_modelo, ...
                X0, punto_min_max_1, punto_min_max_2, 1);
                plot(app.UIAxes,zonaDerrapeX1,zonaDerrapeY1, 'r','LineWidth',2, "DisplayName", "Zona de derrape");
            end
            
            % SEGUDNA ZONA DE DERRAPE
            
            % Inicialización de la lista de coordenadas
            zonaDerrapeX2 = [];
            zonaDerrapeY2 = [];
            
            if(punto_min_max_2<=XF)
                 [zonaDerrapeX2, zonaDerrapeY2] = hallar_zona_derrape(app, ecuacion_modelo, ...
                 XF, punto_min_max_2, punto_min_max_1, -1);
                plot(app.UIAxes,zonaDerrapeX2,zonaDerrapeY2, 'r','LineWidth',2, "DisplayName", "Zona de derrape");
            end
            
            % Se despliega información sobre la cantidad de coordenadas que existe en
            % las zonas de derrape
            app.Zonadederrape1Label.Text="Zona de derrape 1: ("+num2str(zonaDerrapeX1(1))+", "+num2str(zonaDerrapeX1(length(zonaDerrapeX1)))+")";
            app.Zonadederrape2Label.Text="Zona de derrape 2: ("+num2str(zonaDerrapeX2(1))+", "+num2str(zonaDerrapeX2(length(zonaDerrapeX2)))+")";
            %drawnow;
            
            modificador_tamanio_tangente = 4;
            distGradasY=20;
            distGradasX=-20;
            if(~isempty(zonaDerrapeX1))
                dCurva=matlabFunction(diff(ecuacion_modelo));
                slope = dCurva(zonaDerrapeX1(1));
                slopeInv =-1/slope;
                x_primer_punto_critico = 0:(XF-X0)/modificador_tamanio_tangente;
                y_primer_punto_critico = (slope*x_primer_punto_critico);
                plot(app.UIAxes,x_primer_punto_critico+zonaDerrapeX1(1),y_primer_punto_critico+zonaDerrapeY1(1), "DisplayName", "Tangente", "Color", [0.59 0.57 0])
                
                if(zonaDerrapeY1(1)< zonaDerrapeY2(1))
                    distGradasY = distGradasY *( -1);
                end
                %Encontrar las 4 esquinas de las gradas
                %punto 1
                [xa,ya]=obtenerPuntosGradas(app,zonaDerrapeX1(1),zonaDerrapeY1(1),20, slopeInv, false);
                x11(1)=xa;
                y11(1)=ya;
                %punto2
               [xa,ya]=obtenerPuntosGradas(app,x11(1),y11(1),10, slopeInv,false);
               x11(2)=xa;
                y11(2)=ya;
                %punto 3
                [xa,ya]=obtenerPuntosGradas(app,x11(2),y11(2),80, slope,true);
                x11(3)=xa;
                y11(3)=ya;
                %punto 4
               [xa,ya]=obtenerPuntosGradas(app,x11(3),y11(3),10, slopeInv,true);
               x11(4)=xa;
                y11(4)=ya;
                
                line(app.UIAxes,x11, y11, "HandleVisibility", "Off");
                %hold(app.UIAxes, 'on');
                fill(app.UIAxes,x11, y11, 'b', "DisplayName", "Gradas");
                
                disp("Ecuacion recta tangente 1: "+slope+"x + "+(zonaDerrapeY1(1)-slope*zonaDerrapeX1(1)));
                
                %plot(x_primer_punto_critico+zonaDerrapeX1(1)+ distGradasX,y_primer_punto_critico+zonaDerrapeY1(1) + distGradasY,'Color', [0.5 0.5 0.5] , 'lineWidth', 8)
                
            end
            
            if(~isempty(zonaDerrapeX2)>0)
                dCurva=matlabFunction(diff(ecuacion_modelo));
                slope = dCurva(zonaDerrapeX2(1));
                slopeInv =-1/slope;
                x_segundo_punto_critico = 0:(XF-X0)/modificador_tamanio_tangente;
                y_segundo_punto_critico = (slope*x_primer_punto_critico);
                plot(app.UIAxes,x_segundo_punto_critico+zonaDerrapeX2(1),y_segundo_punto_critico+zonaDerrapeY2(1), "DisplayName", "Tangente", "Color", [0.59 0.57 0]);
                distGradas=20;
                
                
                %Encontrar las 4 esquinas de las gradas
                %punto 1
                [xa,ya]=obtenerPuntosGradas(app,zonaDerrapeX2(1),zonaDerrapeY2(1),20, slopeInv, false);
                x12(1)=xa;
                y12(1)=ya;
                %punto2
               [xa,ya]=obtenerPuntosGradas(app,x12(1),y12(1),10, slopeInv,false);
               x12(2)=xa;
                y12(2)=ya;
                %punto 3
                [xa,ya]=obtenerPuntosGradas(app,x12(2),y12(2),80, slope,true);
                x12(3)=xa;
                y12(3)=ya;
                %punto 4
               [xa,ya]=obtenerPuntosGradas(app,x12(3),y12(3),10, slopeInv,true);
               x12(4)=xa;
                y12(4)=ya;
                
                line(app.UIAxes,x12, y12, "HandleVisibility", "Off");
                %hold(app.UIAxes,'on');
                fill(app.UIAxes,x12, y12, 'b', "DisplayName", "Gradas");
                
            end
            
            puntos_lista_velocidad = X0:XF;
            
            lista_velocidad = obtener_velocidad(app,ecuacion_modelo, zonaDerrapeX1, str2num(char(extractBetween(app.Friccion08miuLabel.Text,": "," miu"))), 9.81);
            velocidad_maxima_permitida = min(lista_velocidad);
            app.VelmxpermitidaenzonadederrapeLabel.Text="Vel. máx. permitida en zona de derrape: "+num2str(velocidad_maxima_permitida)+" m/s";
                    
            hold off
            
            % Se escribe la leyenda de la gráfica
            legend(app.UIAxes,'Location','northeastoutside')
            
            % Se habilitan distintas opciones para realizar simulaciones
            % sobre la gráfica
            
            
            app.LLuviaactivadaSwitchLabel.Visible = 'on';
            app.LLuviaactivadaSwitch.Visible = 'on';
            app.CrearferrariButton.Visible = 'on';
            app.CrearcurvaButton.Visible = 'off';
            app.Friccion08miuLabel.Visible = 'on';
            
            app.VelocidadincialmsEditFieldLabel.Visible = 'on';
            app.VelocidadincialmsEditField.Visible = 'on';
            app.MasadelferrarikgEditField.Visible = 'on';
            app.MasadelferrarikgEditFieldLabel.Visible = 'on';
        
        end

        % Value changed function: LLuviaactivadaSwitch
        function LLuviaactivadaSwitchValueChanged(app, event)
            value = app.LLuviaactivadaSwitch.Value;
            puntos_lista_velocidad = 10:290;
            if(value=="Off")
                app.Friccion08miuLabel.Text="Friccion: 0.8 miu";
                lista_velocidad = obtener_velocidad(app,str2sym(app.EcuacionText.Text), ...
                    puntos_lista_velocidad, 0.8, 9.81);        
            else
                app.Friccion08miuLabel.Text="Friccion: 0.4 miu";
                lista_velocidad = obtener_velocidad(app,str2sym(app.EcuacionText.Text), ...
                    puntos_lista_velocidad, 0.4, 9.81);
            end

            velocidad_maxima_permitida = min(lista_velocidad);
            app.VelmxpermitidaenzonadederrapeLabel.Text = ...
                "Velocidad Maxima en toda la pista: " + ...
                num2str(velocidad_maxima_permitida)+"ms^1";
        end

        % Button pushed function: CrearferrariButton
        function CrearferrariButtonPushed(app, event)
            
            %constante de la gravedad
            G = 9.81;
            velocidad=app.VelocidadincialmsEditField.Value;
            coef_friccion=0.8;
            masa=app.MasadelferrarikgEditField.Value;
            
            %ecuacion de la curva para usarla
            ecuacionCurva = matlabFunction(str2sym(app.EcuacionText.Text)); 
            
            % Obtencion de variables por teclado
            if(app.LLuviaactivadaSwitch.Value=="Off")
                coef_friccion = 0.8;
            else
                coef_friccion = 0.4;
            end
            
            x = 10;
            y = 290;
            XF = 280;
            YF = 120;
            
            p = plot(app.UIAxes,x,y,'o','MarkerFaceColor','red');
            
            derrapeEmpieza1 = str2num(char(extractBetween(app.Zonadederrape1Label.Text,"(",",")));
            derrapeTermina1 = str2num(char(extractBetween(app.Zonadederrape1Label.Text,", ",")")));
            derrapeEmpieza2 = str2num(char(extractBetween(app.Zonadederrape2Label.Text,"(",",")));
            derrapeTermina2 = str2num(char(extractBetween(app.Zonadederrape2Label.Text,", ",")")));
            
            %{
                Mueve el marcador del punto a lo largo de la linea
                actualizando el valor de sus coordenadas X y Y en un ciclo.
            %}

            i=x+1;
            x=i;
            salir=false;
            while i<XF && salir==false
                rapidez=app.VelocidadanimacionSlider.Value;
                velMax=obtener_velocidad(app, ...
                                        str2sym(app.EcuacionText.Text), ...
                                        x, coef_friccion, G);
                p.XData = i;
                y=ecuacionCurva(i);
                p.YData = y;
                app.PuntoactualLabel.Text="Punto actual: (" + ...
                                          num2str(x)+", "+num2str(y)+")";
                app.VelmxpermitidaenpuntoactualLabel.Text= ...
                    "Vel. máx permitida en punto actual: " + ...
                    num2str(velMax);
                app.EstadoLabel.Text="Estado: En curso";
                app.EnergiaPerdidaLabel.Text="Energia perdida: 0J";
                app.DistanciaderrapadaLabel.Text="Distancia derrapada: 0m";
                drawnow;
                pause(rapidez);
                
                % Verifica si la velocidad inicial del carro es mayor a la
                % permitida, en caso de que si sea, se procede a ejecutar
                % el siguiente bloque de instrucciones gracias al booleano
                % salir
                if velocidad>velMax && ...
                   ((x>=derrapeEmpieza1 && x<=derrapeTermina1) || ...
                   (x>=derrapeEmpieza2 && x<=derrapeTermina2))
                    
                    salir=true;
                else
                    i=i+1;
                    x=i;
                end
                    
            end
            
            % Determina la nueva trayectoria del carro, calcula la
            % distancia maxima recorrida, asi como el calor que se ha
            % generado por la friccion y el derrape
            if salir
                dCurva=matlabFunction(diff(str2sym(app.EcuacionText.Text)));
                slope = dCurva(x);
                distancia=(velocidad^2)/(2*G*coef_friccion);
                energiaPerd=0.5*masa*velocidad^2;
                [xa,ya]=obtenerPuntosGradas(app,x,y,distancia, slope, true);
                app.VelmxpermitidaenpuntoactualLabel.Text = ...
                    "Vel. máx permitida en punto actual: -";
                app.EnergiaPerdidaLabel.Text="Energia perdida: " + ...
                    num2str(energiaPerd)+"J";
                app.EstadoLabel.Text="Estado: Desviado";
                app.DistanciaderrapadaLabel.Text= ...
                    "Distancia derrapada: " + num2str(distancia)+"m";
                line(app.UIAxes,[x xa],[y ya]);
                for i=x:xa
                    rapidez=app.VelocidadanimacionSlider.Value;
                    y=y+slope;
                    p.YData=y;
                    p.XData=i;
                    app.PuntoactualLabel.Text = ...
                        "Punto actual: ("+num2str(i)+", "+num2str(y)+")";
                    
                    drawnow;
                    pause(rapidez);
                end
            end
            
            
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 2x1 grid
                app.GridLayout.RowHeight = {572, 572};
                app.GridLayout.ColumnWidth = {'1x'};
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 1;
            else
                % Change to a 1x2 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {270, '1x'};
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 2;
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 839 572];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {270, '1x'};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;

            % Create CrearcurvaButton
            app.CrearcurvaButton = uibutton(app.LeftPanel, 'push');
            app.CrearcurvaButton.ButtonPushedFcn = createCallbackFcn(app, @CrearcurvaButtonPushed, true);
            app.CrearcurvaButton.Position = [68 92 100 22];
            app.CrearcurvaButton.Text = 'Crear curva';

            % Create VelocidadincialmsEditFieldLabel
            app.VelocidadincialmsEditFieldLabel = uilabel(app.LeftPanel);
            app.VelocidadincialmsEditFieldLabel.HorizontalAlignment = 'center';
            app.VelocidadincialmsEditFieldLabel.Visible = 'off';
            app.VelocidadincialmsEditFieldLabel.Position = [14 516 88 27];
            app.VelocidadincialmsEditFieldLabel.Text = {'Velocidad incial'; '(m/s)'};

            % Create VelocidadincialmsEditField
            app.VelocidadincialmsEditField = uieditfield(app.LeftPanel, 'numeric');
            app.VelocidadincialmsEditField.Limits = [1 Inf];
            app.VelocidadincialmsEditField.Visible = 'off';
            app.VelocidadincialmsEditField.Position = [117 521 100 22];
            app.VelocidadincialmsEditField.Value = 1;

            % Create Puntoincial10290Label
            app.Puntoincial10290Label = uilabel(app.LeftPanel);
            app.Puntoincial10290Label.Position = [13 374 122 22];
            app.Puntoincial10290Label.Text = 'Punto incial: (10, 290)';

            % Create Puntofinal280120Label
            app.Puntofinal280120Label = uilabel(app.LeftPanel);
            app.Puntofinal280120Label.Position = [13 353 124 22];
            app.Puntofinal280120Label.Text = 'Punto final: (280, 120)';

            % Create PuntoactualLabel
            app.PuntoactualLabel = uilabel(app.LeftPanel);
            app.PuntoactualLabel.Position = [13 302 185 22];
            app.PuntoactualLabel.Text = 'Punto actual:';

            % Create LLuviaactivadaSwitchLabel
            app.LLuviaactivadaSwitchLabel = uilabel(app.LeftPanel);
            app.LLuviaactivadaSwitchLabel.HorizontalAlignment = 'center';
            app.LLuviaactivadaSwitchLabel.Visible = 'off';
            app.LLuviaactivadaSwitchLabel.Position = [16 405 89 22];
            app.LLuviaactivadaSwitchLabel.Text = 'LLuvia activada';

            % Create LLuviaactivadaSwitch
            app.LLuviaactivadaSwitch = uiswitch(app.LeftPanel, 'slider');
            app.LLuviaactivadaSwitch.ValueChangedFcn = createCallbackFcn(app, @LLuviaactivadaSwitchValueChanged, true);
            app.LLuviaactivadaSwitch.Visible = 'off';
            app.LLuviaactivadaSwitch.Position = [36 442 45 20];

            % Create CrearferrariButton
            app.CrearferrariButton = uibutton(app.LeftPanel, 'push');
            app.CrearferrariButton.ButtonPushedFcn = createCallbackFcn(app, @CrearferrariButtonPushed, true);
            app.CrearferrariButton.Visible = 'off';
            app.CrearferrariButton.Position = [68 62 100 22];
            app.CrearferrariButton.Text = 'Crear ferrari';

            % Create Friccion08miuLabel
            app.Friccion08miuLabel = uilabel(app.LeftPanel);
            app.Friccion08miuLabel.Visible = 'off';
            app.Friccion08miuLabel.Position = [126 426 94 22];
            app.Friccion08miuLabel.Text = 'Friccion: 0.8 miu';

            % Create CrearelferrarisolodespuesdecreadaydibujadalacurvaLabel
            app.CrearelferrarisolodespuesdecreadaydibujadalacurvaLabel = uilabel(app.LeftPanel);
            app.CrearelferrarisolodespuesdecreadaydibujadalacurvaLabel.HorizontalAlignment = 'center';
            app.CrearelferrarisolodespuesdecreadaydibujadalacurvaLabel.WordWrap = 'on';
            app.CrearelferrarisolodespuesdecreadaydibujadalacurvaLabel.Position = [12 15 206 38];
            app.CrearelferrarisolodespuesdecreadaydibujadalacurvaLabel.Text = '*Crear el ferrari solo despues de creada y dibujada la curva*';

            % Create VelocidadanimacionSliderLabel
            app.VelocidadanimacionSliderLabel = uilabel(app.LeftPanel);
            app.VelocidadanimacionSliderLabel.HorizontalAlignment = 'right';
            app.VelocidadanimacionSliderLabel.Position = [74 139 116 22];
            app.VelocidadanimacionSliderLabel.Text = 'Velocidad animacion';

            % Create VelocidadanimacionSlider
            app.VelocidadanimacionSlider = uislider(app.LeftPanel);
            app.VelocidadanimacionSlider.Limits = [0.0003 2];
            app.VelocidadanimacionSlider.MajorTicks = [0.05 0.7 1.35 2];
            app.VelocidadanimacionSlider.MajorTickLabels = {'Veloz', '', '', 'Lento'};
            app.VelocidadanimacionSlider.MinorTicks = [0.0003 0.0503 0.1003 0.1503 0.2003 0.2503 0.3003 0.3503 0.4003 0.4503 0.5003 0.5503 0.6003 0.6503 0.7003 0.7503 0.8003 0.8503 0.9003 0.9503 1.0003 1.0503 1.1003 1.1503 1.2003 1.2503 1.3003 1.3503 1.4003 1.4503 1.5003 1.5503 1.6003 1.6503 1.7003 1.7503 1.8003 1.8503 1.9003 1.9503 2];
            app.VelocidadanimacionSlider.Position = [15 197 234 3];
            app.VelocidadanimacionSlider.Value = 1;

            % Create VelmxpermitidaenpuntoactualLabel
            app.VelmxpermitidaenpuntoactualLabel = uilabel(app.LeftPanel);
            app.VelmxpermitidaenpuntoactualLabel.Position = [14 318 289 27];
            app.VelmxpermitidaenpuntoactualLabel.Text = 'Vel. máx. permitida en punto actual: ';

            % Create EstadoLabel
            app.EstadoLabel = uilabel(app.LeftPanel);
            app.EstadoLabel.Position = [13 266 239 22];
            app.EstadoLabel.Text = 'Estado: ';

            % Create EnergiaPerdidaLabel
            app.EnergiaPerdidaLabel = uilabel(app.LeftPanel);
            app.EnergiaPerdidaLabel.Position = [13 245 194 22];
            app.EnergiaPerdidaLabel.Text = 'Energia Perdida:';

            % Create MasadelferrarikgEditFieldLabel
            app.MasadelferrarikgEditFieldLabel = uilabel(app.LeftPanel);
            app.MasadelferrarikgEditFieldLabel.HorizontalAlignment = 'center';
            app.MasadelferrarikgEditFieldLabel.WordWrap = 'on';
            app.MasadelferrarikgEditFieldLabel.Visible = 'off';
            app.MasadelferrarikgEditFieldLabel.Position = [13 479 105 34];
            app.MasadelferrarikgEditFieldLabel.Text = 'Masa del ferrari: (kg)';

            % Create MasadelferrarikgEditField
            app.MasadelferrarikgEditField = uieditfield(app.LeftPanel, 'numeric');
            app.MasadelferrarikgEditField.Limits = [740 1000];
            app.MasadelferrarikgEditField.Visible = 'off';
            app.MasadelferrarikgEditField.Position = [117 485 100 22];
            app.MasadelferrarikgEditField.Value = 740;

            % Create DistanciaderrapadaLabel
            app.DistanciaderrapadaLabel = uilabel(app.LeftPanel);
            app.DistanciaderrapadaLabel.Position = [13 224 241 22];
            app.DistanciaderrapadaLabel.Text = 'Distancia derrapada: ';

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;

            % Create EcuacindelacurvaLabel
            app.EcuacindelacurvaLabel = uilabel(app.RightPanel);
            app.EcuacindelacurvaLabel.VerticalAlignment = 'top';
            app.EcuacindelacurvaLabel.WordWrap = 'on';
            app.EcuacindelacurvaLabel.Position = [22 541 236 17];
            app.EcuacindelacurvaLabel.Text = 'Ecuación de la curva:';

            % Create PuntomaximoLabel
            app.PuntomaximoLabel = uilabel(app.RightPanel);
            app.PuntomaximoLabel.Position = [22 447 260 22];
            app.PuntomaximoLabel.Text = 'Punto maximo:';

            % Create PuntominimoLabel
            app.PuntominimoLabel = uilabel(app.RightPanel);
            app.PuntominimoLabel.Position = [22 426 260 22];
            app.PuntominimoLabel.Text = 'Punto minimo:';

            % Create LongitudLabel
            app.LongitudLabel = uilabel(app.RightPanel);
            app.LongitudLabel.Position = [22 405 248 22];
            app.LongitudLabel.Text = 'Longitud: ';

            % Create RadiodeCurvatura1Label
            app.RadiodeCurvatura1Label = uilabel(app.RightPanel);
            app.RadiodeCurvatura1Label.Position = [22 384 321 22];
            app.RadiodeCurvatura1Label.Text = 'Radio de Curvatura 1:';

            % Create EcuacionText
            app.EcuacionText = uilabel(app.RightPanel);
            app.EcuacionText.WordWrap = 'on';
            app.EcuacionText.Position = [22 468 354 79];
            app.EcuacionText.Text = '';

            % Create Zonadederrape1Label
            app.Zonadederrape1Label = uilabel(app.RightPanel);
            app.Zonadederrape1Label.Position = [23 344 259 22];
            app.Zonadederrape1Label.Text = 'Zona de derrape 1:';

            % Create Zonadederrape2Label
            app.Zonadederrape2Label = uilabel(app.RightPanel);
            app.Zonadederrape2Label.Position = [22 323 276 22];
            app.Zonadederrape2Label.Text = 'Zona de derrape 2:';

            % Create RadiodeCurvatura2Label
            app.RadiodeCurvatura2Label = uilabel(app.RightPanel);
            app.RadiodeCurvatura2Label.Position = [23 363 321 22];
            app.RadiodeCurvatura2Label.Text = 'Radio de Curvatura 2:';

            % Create VelmxpermitidaenzonadederrapeLabel
            app.VelmxpermitidaenzonadederrapeLabel = uilabel(app.RightPanel);
            app.VelmxpermitidaenzonadederrapeLabel.Position = [22 302 322 22];
            app.VelmxpermitidaenzonadederrapeLabel.Text = 'Vel. máx permitida en zona de derrape:';

            % Create UIAxes
            app.UIAxes = uiaxes(app.RightPanel);
            title(app.UIAxes, 'Pista de Formula 1')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Position = [7 15 380 251];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = solucionReto_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end