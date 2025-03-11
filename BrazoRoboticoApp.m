classdef BrazoRoboticoApp < matlab.apps.AppBase

    % Propiedades que corresponden a los componentes de la app
    properties (Access = public)
        UIFigure             matlab.ui.Figure
        GridLayout           matlab.ui.container.GridLayout
        
        % Panel izquierdo - Control de ángulos
        ControlPanel         matlab.ui.container.Panel
        Angulo1Label         matlab.ui.control.Label
        Angulo1EditField     matlab.ui.control.NumericEditField
        Angulo1DegreesLabel  matlab.ui.control.Label
        
        Angulo2Label         matlab.ui.control.Label
        Angulo2EditField     matlab.ui.control.NumericEditField
        Angulo2DegreesLabel  matlab.ui.control.Label
        
        Angulo3Label         matlab.ui.control.Label
        Angulo3EditField     matlab.ui.control.NumericEditField
        Angulo3DegreesLabel  matlab.ui.control.Label
        
        TiempoSimLabel       matlab.ui.control.Label
        TiempoSimEditField   matlab.ui.control.NumericEditField
        
        ModoSimulacionLabel  matlab.ui.control.Label
        ModoSimulacionDropDown matlab.ui.control.DropDown
        
        ActualizarButton     matlab.ui.control.Button
        SimularModeloButton  matlab.ui.control.Button
        MechanicsExplorerButton matlab.ui.control.Button
        DetenerSimButton     matlab.ui.control.Button
        
        % Panel central - Cinemática directa
        CinematicaPanel      matlab.ui.container.Panel
        MatrizTitleLabel     matlab.ui.control.Label
        
        MatrizRow1Label      matlab.ui.control.Label
        MatrizRow2Label      matlab.ui.control.Label
        MatrizRow3Label      matlab.ui.control.Label
        MatrizRow4Label      matlab.ui.control.Label
        
        PosicionTitleLabel   matlab.ui.control.Label
        PosicionTextArea     matlab.ui.control.TextArea
        
        EstadoSimscapeLabel  matlab.ui.control.Label
        EstadoSimscapeTextArea matlab.ui.control.TextArea
        
        % Panel derecho - Visualización
        BrazoAxes            matlab.ui.control.UIAxes
    end
    
    % Propiedades para el cálculo del brazo robótico
    properties (Access = private)
        L1 = 10      % Longitud eslabón 1 (cm)
        L2 = 10.2    % Longitud eslabón 2 (cm)
        L3 = 8       % Longitud eslabón 3 (cm)
        d1 = 0       % Parámetro DH
        d2 = 0       % Parámetro DH
        d3 = 0       % Parámetro DH
        alpha_1 = 0  % Parámetro DH
        alpha_2 = 0  % Parámetro DH
        alpha_3 = 0  % Parámetro DH
    end
    
    methods (Access = private)
        
        function actualizarBrazo(app)
            % Obtener valores de los ángulos
            theta1 = app.Angulo1EditField.Value;
            theta2 = app.Angulo2EditField.Value;
            theta3 = app.Angulo3EditField.Value;
            
            % Convertir a radianes para los cálculos
            Theta_1 = deg2rad(theta1);
            Theta_2 = deg2rad(theta2);
            Theta_3 = deg2rad(theta3);
            
            % Crear variables en el workspace para Simulink/Simscape
            % Crear valores de tiempo para simulación (0 a tiempo seleccionado)
            t = [0; app.TiempoSimEditField.Value];
            
            % Crear señales constantes para los ángulos
            senal_angulo1 = [theta1; theta1];
            senal_angulo2 = [theta2; theta2];
            senal_angulo3 = [theta3; theta3];
            
            % Asignar al workspace
            assignin('base', 'tiempo', t);
            assignin('base', 'senal_angulo1', [t, senal_angulo1]);
            assignin('base', 'senal_angulo2', [t, senal_angulo2]);
            assignin('base', 'senal_angulo3', [t, senal_angulo3]);
            
            % También asignar ángulos en radianes para uso en Simscape si es necesario
            assignin('base', 'angulo_revolute1', Theta_1);
            assignin('base', 'angulo_revolute2', Theta_2);
            assignin('base', 'angulo_revolute3', Theta_3);
            
            % Inicializar o actualizar la estructura simData
            if ~evalin('base', 'exist(''simData'', ''var'')')
                simData = struct();
            else
                simData = evalin('base', 'simData');
            end
            
            % Configurar datos para transformaciones rígidas
            simData.RigidTransform(1).axis = [0 0 1];  % Eje Z
            simData.RigidTransform(1).angle = Theta_1;
            simData.RigidTransform(1).translation = [app.L1*10 0 0];  % Convertir a mm
            
            simData.RigidTransform(2).axis = [0 0 1];
            simData.RigidTransform(2).angle = Theta_2;
            simData.RigidTransform(2).translation = [app.L2*10 0 0];
            
            simData.RigidTransform(3).axis = [0 0 1];
            simData.RigidTransform(3).angle = Theta_3;
            simData.RigidTransform(3).translation = [app.L3*10 0 0];
            
            % Enviar al workspace
            assignin('base', 'simData', simData);
            
            % Actualizar estado Simscape
            app.EstadoSimscapeTextArea.Value = 'Ángulos actualizados en workspace. Listo para simular.';
            
            % Calcular matrices de transformación
            A1 = [cos(Theta_1) -sin(Theta_1)*cos(app.alpha_1) sin(Theta_1)*sin(app.alpha_1) app.L1*cos(Theta_1);
                  sin(Theta_1) cos(Theta_1)*cos(app.alpha_1) -cos(Theta_1)*sin(app.alpha_1) app.L1*sin(Theta_1);
                  0     sin(app.alpha_1)    cos(app.alpha_1)        app.d1;
                  0 0 0 1];
            
            A2 = [cos(Theta_2) -sin(Theta_2)*cos(app.alpha_2) sin(Theta_2)*sin(app.alpha_2) app.L2*cos(Theta_2);
                  sin(Theta_2) cos(Theta_2)*cos(app.alpha_2) -cos(Theta_2)*sin(app.alpha_2) app.L2*sin(Theta_2);
                  0 sin(app.alpha_2) cos(app.alpha_2) app.d2;
                  0 0 0 1];
            
            A3 = [cos(Theta_3) -sin(Theta_3)*cos(app.alpha_3) sin(Theta_3)*sin(app.alpha_3) app.L3*cos(Theta_3);
                  sin(Theta_3) cos(Theta_3)*cos(app.alpha_3) -cos(Theta_3)*sin(app.alpha_3) app.L3*sin(Theta_3);
                  0 sin(app.alpha_3) cos(app.alpha_3) app.d3;
                  0 0 0 1];
            
            % Calcular matrices de transformación acumuladas
            T01 = A1;
            T02 = A1*A2;
            T03 = A1*A2*A3; % Matriz de transformación total
            
            % Actualizar interfaz con la matriz
            app.MatrizRow1Label.Text = sprintf('╔ %6.3f  %6.3f  %6.3f  %6.1f ╗', T03(1,1), T03(1,2), T03(1,3), T03(1,4));
            app.MatrizRow2Label.Text = sprintf('║ %6.3f  %6.3f  %6.3f  %6.1f ║', T03(2,1), T03(2,2), T03(2,3), T03(2,4));
            app.MatrizRow3Label.Text = sprintf('║ %6.3f  %6.3f  %6.3f  %6.1f ║', T03(3,1), T03(3,2), T03(3,3), T03(3,4));
            app.MatrizRow4Label.Text = sprintf('╚ %6.3f  %6.3f  %6.3f  %6.1f ╝', T03(4,1), T03(4,2), T03(4,3), T03(4,4));
            
            % Actualizar posición del efector final
            pos_str = sprintf('X = %6.1f cm\nY = %6.1f cm\nZ = %6.1f cm', T03(1,4), T03(2,4), T03(3,4));
            app.PosicionTextArea.Value = pos_str;
            
            % Obtener coordenadas de las articulaciones
            x0 = 0;
            y0 = 0;
            
            x1 = T01(1,4);
            y1 = T01(2,4);
            
            x2 = T02(1,4);
            y2 = T02(2,4);
            
            x3 = T03(1,4);
            y3 = T03(2,4);
            
            % Dibujar el brazo
            cla(app.BrazoAxes);
            hold(app.BrazoAxes, 'on');
            
            % Dibujar eslabones con líneas de colores
            h1 = plot(app.BrazoAxes, [x0 x1], [y0 y1], 'b-', 'LineWidth', 3); % Eslabón 1 (base)
            h2 = plot(app.BrazoAxes, [x1 x2], [y1 y2], 'r-', 'LineWidth', 3); % Eslabón 2 (medio)
            h3 = plot(app.BrazoAxes, [x2 x3], [y2 y3], 'g-', 'LineWidth', 3); % Eslabón 3 (efector final)
            
            % Agregar marcadores en las juntas
            h4 = plot(app.BrazoAxes, [x0 x1 x2], [y0 y1 y2], 'ko', 'MarkerFaceColor', 'black', 'MarkerSize', 10);
            
            % Destacar la posición final
            h5 = plot(app.BrazoAxes, x3, y3, 'ko', 'MarkerFaceColor', 'black', 'MarkerSize', 12);
            
            % Agregar líneas de referencia en la posición final
            plot(app.BrazoAxes, [x3-2 x3+2], [y3 y3], 'c-', 'LineWidth', 2);
            plot(app.BrazoAxes, [x3 x3], [y3-2 y3+2], 'c-', 'LineWidth', 2);
            
            % Agregar círculo alrededor de la posición final
            t = linspace(0, 2*pi, 100);
            radio = 1.5;
            circle_x = x3 + radio * cos(t);
            circle_y = y3 + radio * sin(t);
            plot(app.BrazoAxes, circle_x, circle_y, 'm:', 'LineWidth', 2);
            
            % Mostrar coordenadas cerca del efector final
            text(app.BrazoAxes, x3+2, y3, sprintf('(%0.1f, %0.1f)', x3, y3), 'FontSize', 8);
            
            % Configurar ejes
            app.BrazoAxes.XLim = [-40 40];
            app.BrazoAxes.YLim = [-40 40];
            grid(app.BrazoAxes, 'on');
            
            % También mostrar en la consola para cumplir con el formato del script original
            fprintf('\nMatriz de Transformación Homogénea:\n');
            fprintf('╔ %6.3f  %6.3f  %6.3f  %6.1f ╗\n', T03(1,1), T03(1,2), T03(1,3), T03(1,4));
            fprintf('║ %6.3f  %6.3f  %6.3f  %6.1f ║\n', T03(2,1), T03(2,2), T03(2,3), T03(2,4));
            fprintf('║ %6.3f  %6.3f  %6.3f  %6.1f ║\n', T03(3,1), T03(3,2), T03(3,3), T03(3,4));
            fprintf('╚ %6.3f  %6.3f  %6.3f  %6.1f ╝\n', T03(4,1), T03(4,2), T03(4,3), T03(4,4));
            
            fprintf('\nPosición del Efector Final:\n');
            fprintf('X = %6.1f cm\n', T03(1,4));
            fprintf('Y = %6.1f cm\n', T03(2,4));
            fprintf('Z = %6.1f cm\n', T03(3,4));
        end
        
        function iniciarSimulacion(app)
            % Obtener tiempo de simulación
            tsim = app.TiempoSimEditField.Value;
            
            % Obtener modo de simulación
            sim_mode_val = app.ModoSimulacionDropDown.Value;
            
            % Actualizar estado
            app.EstadoSimscapeTextArea.Value = 'Iniciando simulación...';
            drawnow;
            
            try
                % Verificar si el modelo está cargado, si no, cargarlo
                if ~bdIsLoaded('BrazoCoplanar1')
                    % Intentar cargar el modelo
                    try
                        load_system('BrazoCoplanar1');
                        app.EstadoSimscapeTextArea.Value = 'Modelo cargado correctamente';
                    catch
                        % Si no existe, mostrar mensaje para crearlo
                        uialert(app.UIFigure, ...
                                'El modelo "BrazoCoplanar1" no existe. Por favor cree el modelo según la imagen de Simscape adjunta.', ...
                                'Modelo no encontrado', 'Icon', 'warning');
                        app.EstadoSimscapeTextArea.Value = 'Error: Modelo no encontrado';
                        return;
                    end
                end
                
                % Configurar tiempo de simulación
                set_param('BrazoCoplanar1', 'StopTime', num2str(tsim));
                
                % Si es paso a paso, configurar simulación en modo paso a paso
                if strcmp(sim_mode_val, 'Paso a paso')
                    set_param('BrazoCoplanar1', 'SimulationMode', 'stepped');
                else
                    set_param('BrazoCoplanar1', 'SimulationMode', 'normal');
                end
                
                % Iniciar simulación
                app.EstadoSimscapeTextArea.Value = 'Simulando...';
                drawnow;
                
                sim('BrazoCoplanar1');
                
                % Comprobar si existen resultados de simulación
                if evalin('base', 'exist(''simout'', ''var'')')
                    app.leerResultadosSimulacion();
                else
                    app.EstadoSimscapeTextArea.Value = 'Simulación completada. No hay datos de salida disponibles.';
                end
                
            catch e
                uialert(app.UIFigure, ['Error al simular: ' e.message], 'Error de Simulación', 'Icon', 'error');
                app.EstadoSimscapeTextArea.Value = ['Error: ' e.message];
            end
        end
        
        function leerResultadosSimulacion(app)
            try
                % Verificar existencia de variables de salida
                if evalin('base', 'exist(''simout'', ''var'')')
                    % Obtener datos del efector final
                    simout_data = evalin('base', 'simout');
                    
                    % Extraer último valor (posición final)
                    if isstruct(simout_data)
                        if isfield(simout_data, 'signals')
                            pos_final = simout_data.signals.values(end, :);
                            tiempo_final = simout_data.time(end);
                            
                            % Actualizar texto de estado
                            app.EstadoSimscapeTextArea.Value = sprintf('Simulación completada.\nTiempo: %.2fs\nPos. Final (Simscape):\nX: %.2f\nY: %.2f\nZ: %.2f', ...
                                tiempo_final, pos_final(1), pos_final(2), pos_final(3));
                        else
                            app.EstadoSimscapeTextArea.Value = 'Simulación completada.\nEstructura de datos no reconocida.';
                        end
                    else
                        app.EstadoSimscapeTextArea.Value = 'Simulación completada.\nDatos de salida en formato no reconocido.';
                    end
                else
                    app.EstadoSimscapeTextArea.Value = 'Simulación completada.\nNo hay datos de salida disponibles.';
                end
            catch e
                app.EstadoSimscapeTextArea.Value = ['Error al leer resultados: ' e.message];
            end
        end
        
        function abrirMechanicsExplorer(app)
            try
                % Verificar si el modelo está cargado
                if bdIsLoaded('BrazoCoplanar1')
                    % Abrir Mechanics Explorer
                    set_param('BrazoCoplanar1', 'SimMechanicsOpenMechanicsExplorer', 'on');
                else
                    uialert(app.UIFigure, 'Primero debe cargar el modelo Simscape', 'Modelo no cargado', 'Icon', 'warning');
                end
            catch e
                uialert(app.UIFigure, ['Error al abrir Mechanics Explorer: ' e.message], 'Error', 'Icon', 'error');
            end
        end
        
        function detenerSimulacion(app)
            try
                if bdIsLoaded('BrazoCoplanar1')
                    if strcmp(get_param('BrazoCoplanar1', 'SimulationStatus'), 'running')
                        set_param('BrazoCoplanar1', 'SimulationCommand', 'stop');
                        app.EstadoSimscapeTextArea.Value = 'Simulación detenida por el usuario';
                    end
                end
            catch e
                uialert(app.UIFigure, ['Error al detener simulación: ' e.message], 'Error', 'Icon', 'warning');
            end
        end
    end
    
    % Callbacks que manejan los eventos de la UI
    methods (Access = private)
        
        % Botón de Actualizar
        function ActualizarButtonPushed(app, event)
            app.actualizarBrazo();
        end
        
        % Botón de Simular Modelo
        function SimularModeloButtonPushed(app, event)
            app.iniciarSimulacion();
        end
        
        % Botón de Mechanics Explorer
        function MechanicsExplorerButtonPushed(app, event)
            app.abrirMechanicsExplorer();
        end
        
        % Botón de Detener Simulación
        function DetenerSimButtonPushed(app, event)
            app.detenerSimulacion();
        end
        
        % Función para actualizar automáticamente cuando cambian los ángulos
        function AnguloValueChanged(app, event)
            % Esta función se puede conectar a todos los campos de ángulos
            % para una actualización automática al cambiar los valores
            % app.actualizarBrazo();
            % Está comentada porque en la interfaz original solo actualiza al pulsar el botón
        end
    end
    
    % Métodos de componentes
    methods (Access = private)
        
        % Crear componentes de UI
        function createComponents(app)
            % Crear UIFigure y ocultar hasta que se completen todos los componentes
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100, 100, 1200, 600];
            app.UIFigure.Name = 'Control de Brazo Robótico';
            app.UIFigure.Color = [0.94 0.94 0.94]; % Color de fondo similar al original
            
            % Crear GridLayout con 3 columnas
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {'1x', '1x', '1.5x'};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.Padding = [10 10 10 10];
            app.GridLayout.ColumnSpacing = 10;
            
            % Panel izquierdo - Control de ángulos
            app.ControlPanel = uipanel(app.GridLayout);
            app.ControlPanel.Title = 'Control de Ángulos';
            app.ControlPanel.Layout.Row = 1;
            app.ControlPanel.Layout.Column = 1;
            app.ControlPanel.FontWeight = 'bold';
            
            % Crear layout interno para el panel de control
            controlLayout = uigridlayout(app.ControlPanel);
            controlLayout.ColumnWidth = {'1x', '1x'};
            controlLayout.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x', '1x'};
            controlLayout.Padding = [10 30 10 10];
            
            % Ángulo 1
            app.Angulo1Label = uilabel(controlLayout);
            app.Angulo1Label.Text = 'Ángulo 1 (θ1)';
            app.Angulo1Label.Layout.Row = 1;
            app.Angulo1Label.Layout.Column = 1;
            
            app.Angulo1EditField = uieditfield(controlLayout, 'numeric');
            app.Angulo1EditField.Value = 15;
            app.Angulo1EditField.Layout.Row = 2;
            app.Angulo1EditField.Layout.Column = 1;
            
            app.Angulo1DegreesLabel = uilabel(controlLayout);
            app.Angulo1DegreesLabel.Text = '°';
            app.Angulo1DegreesLabel.Layout.Row = 2;
            app.Angulo1DegreesLabel.Layout.Column = [1 2];
            app.Angulo1DegreesLabel.HorizontalAlignment = 'right';
            
            % Ángulo 2
            app.Angulo2Label = uilabel(controlLayout);
            app.Angulo2Label.Text = 'Ángulo 2 (θ2)';
            app.Angulo2Label.Layout.Row = 3;
            app.Angulo2Label.Layout.Column = 1;
            
            app.Angulo2EditField = uieditfield(controlLayout, 'numeric');
            app.Angulo2EditField.Value = 20;
            app.Angulo2EditField.Layout.Row = 4;
            app.Angulo2EditField.Layout.Column = 1;
            
            app.Angulo2DegreesLabel = uilabel(controlLayout);
            app.Angulo2DegreesLabel.Text = '°';
            app.Angulo2DegreesLabel.Layout.Row = 4;
            app.Angulo2DegreesLabel.Layout.Column = [1 2];
            app.Angulo2DegreesLabel.HorizontalAlignment = 'right';
            
            % Modo de simulación
            app.ModoSimulacionLabel = uilabel(controlLayout);
            app.ModoSimulacionLabel.Text = 'Modo Simulación';
            app.ModoSimulacionLabel.Layout.Row = 3;
            app.ModoSimulacionLabel.Layout.Column = 2;
            
            app.ModoSimulacionDropDown = uidropdown(controlLayout);
            app.ModoSimulacionDropDown.Items = {'Simulación completa', 'Paso a paso'};
            app.ModoSimulacionDropDown.Value = 'Simulación completa';
            app.ModoSimulacionDropDown.Layout.Row = 4;
            app.ModoSimulacionDropDown.Layout.Column = 2;
            
            % Ángulo 3
            app.Angulo3Label = uilabel(controlLayout);
            app.Angulo3Label.Text = 'Ángulo 3 (θ3)';
            app.Angulo3Label.Layout.Row = 5;
            app.Angulo3Label.Layout.Column = 1;
            
            app.Angulo3EditField = uieditfield(controlLayout, 'numeric');
            app.Angulo3EditField.Value = 90;
            app.Angulo3EditField.Layout.Row = 6;
            app.Angulo3EditField.Layout.Column = 1;
            
            app.Angulo3DegreesLabel = uilabel(controlLayout);
            app.Angulo3DegreesLabel.Text = '°';
            app.Angulo3DegreesLabel.Layout.Row = 6;
            app.Angulo3DegreesLabel.Layout.Column = [1 2];
            app.Angulo3DegreesLabel.HorizontalAlignment = 'right';
            
            % Tiempo de simulación
            app.TiempoSimLabel = uilabel(controlLayout);
            app.TiempoSimLabel.Text = 'Tiempo Sim (s)';
            app.TiempoSimLabel.Layout.Row = 5;
            app.TiempoSimLabel.Layout.Column = 2;
            
            app.TiempoSimEditField = uieditfield(controlLayout, 'numeric');
            app.TiempoSimEditField.Value = 10;
            app.TiempoSimEditField.Layout.Row = 6;
            app.TiempoSimEditField.Layout.Column = 2;
            
            % Botones
            app.ActualizarButton = uibutton(controlLayout, 'push');
            app.ActualizarButton.Text = 'Actualizar';
            app.ActualizarButton.Layout.Row = 7;
            app.ActualizarButton.Layout.Column = 1;
            app.ActualizarButton.ButtonPushedFcn = @app.ActualizarButtonPushed;
            
            app.SimularModeloButton = uibutton(controlLayout, 'push');
            app.SimularModeloButton.Text = 'Simular Modelo';
            app.SimularModeloButton.Layout.Row = 7;
            app.SimularModeloButton.Layout.Column = 2;
            app.SimularModeloButton.ButtonPushedFcn = @app.SimularModeloButtonPushed;
            
            % Botones adicionales en nueva fila
            controlLayout.RowHeight{8} = '1x';
            
            app.MechanicsExplorerButton = uibutton(controlLayout, 'push');
            app.MechanicsExplorerButton.Text = 'Mechanics Explorer';
            app.MechanicsExplorerButton.Layout.Row = 8;
            app.MechanicsExplorerButton.Layout.Column = 1;
            app.MechanicsExplorerButton.ButtonPushedFcn = @app.MechanicsExplorerButtonPushed;
            
            app.DetenerSimButton = uibutton(controlLayout, 'push');
            app.DetenerSimButton.Text = 'Detener Sim';
            app.DetenerSimButton.Layout.Row = 8;
            app.DetenerSimButton.Layout.Column = 2;
            app.DetenerSimButton.ButtonPushedFcn = @app.DetenerSimButtonPushed;
            
            % Panel central - Cinemática directa
            app.CinematicaPanel = uipanel(app.GridLayout);
            app.CinematicaPanel.Title = 'Cinemática Directa';
            app.CinematicaPanel.Layout.Row = 1;
            app.CinematicaPanel.Layout.Column = 2;
            app.CinematicaPanel.FontWeight = 'bold';
            
            % Crear layout interno para el panel de cinemática
            cinematicaLayout = uigridlayout(app.CinematicaPanel);
            cinematicaLayout.ColumnWidth = {'1x'};
            cinematicaLayout.RowHeight = {'0.7x', '0.3x', '0.3x', '0.3x', '0.3x', '0.5x', '0.7x', '0.5x', '0.7x'};
            cinematicaLayout.Padding = [10 10 10 10];
            
            % Título de la matriz
            app.MatrizTitleLabel = uilabel(cinematicaLayout);
            app.MatrizTitleLabel.Text = 'Matriz de Transformación Homogénea';
            app.MatrizTitleLabel.Layout.Row = 1;
            app.MatrizTitleLabel.Layout.Column = 1;
            app.MatrizTitleLabel.FontWeight = 'bold';
            app.MatrizTitleLabel.HorizontalAlignment = 'center';
            
            % Filas de la matriz
            app.MatrizRow1Label = uilabel(cinematicaLayout);
            app.MatrizRow1Label.Text = '';
            app.MatrizRow1Label.Layout.Row = 2;
            app.MatrizRow1Label.Layout.Column = 1;
            app.MatrizRow1Label.FontName = 'Courier New';
            
            app.MatrizRow2Label = uilabel(cinematicaLayout);
            app.MatrizRow2Label.Text = '';
            app.MatrizRow2Label.Layout.Row = 3;
            app.MatrizRow2Label.Layout.Column = 1;
            app.MatrizRow2Label.FontName = 'Courier New';
            
            app.MatrizRow3Label = uilabel(cinematicaLayout);
            app.MatrizRow3Label.Text = '';
            app.MatrizRow3Label.Layout.Row = 4;
            app.MatrizRow3Label.Layout.Column = 1;
            app.MatrizRow3Label.FontName = 'Courier New';
            
            app.MatrizRow4Label = uilabel(cinematicaLayout);
            app.MatrizRow4Label.Text = '';
            app.MatrizRow4Label.Layout.Row = 5;
            app.MatrizRow4Label.Layout.Column = 1;
            app.MatrizRow4Label.FontName = 'Courier New';
            
            % Título de posición
            app.PosicionTitleLabel = uilabel(cinematicaLayout);
            app.PosicionTitleLabel.Text = 'Posición del Efector Final';
            app.PosicionTitleLabel.Layout.Row = 6;
            app.PosicionTitleLabel.Layout.Column = 1;
            app.PosicionTitleLabel.FontWeight = 'bold';
            app.PosicionTitleLabel.HorizontalAlignment = 'center';
            
            % Texto de posición
            app.PosicionTextArea = uitextarea(cinematicaLayout);
            app.PosicionTextArea.Value = '';
            app.PosicionTextArea.Layout.Row = 7;
            app.PosicionTextArea.Layout.Column = 1;
            app.PosicionTextArea.FontName = 'Courier New';
            app.PosicionTextArea.Editable = 'off';
            
            % Título de estado Simscape
            app.EstadoSimscapeLabel = uilabel(cinematicaLayout);
            app.EstadoSimscapeLabel.Text = 'Estado Simscape';
            app.EstadoSimscapeLabel.Layout.Row = 8;
            app.EstadoSimscapeLabel.Layout.Column = 1;
            app.EstadoSimscapeLabel.FontWeight = 'bold';
            app.EstadoSimscapeLabel.HorizontalAlignment = 'center';
            
            % Texto de estado Simscape
            app.EstadoSimscapeTextArea = uitextarea(cinematicaLayout);
            app.EstadoSimscapeTextArea.Value = 'No simulado';
            app.EstadoSimscapeTextArea.Layout.Row = 9;
            app.EstadoSimscapeTextArea.Layout.Column = 1;
            app.EstadoSimscapeTextArea.FontName = 'Courier New';
            app.EstadoSimscapeTextArea.Editable = 'off';
            
            % Panel derecho - Visualización
            app.BrazoAxes = uiaxes(app.GridLayout);
            app.BrazoAxes.Layout.Row = 1;
            app.BrazoAxes.Layout.Column = 3;
            app.BrazoAxes.Title.String = 'Visualización del Brazo Robótico';
            app.BrazoAxes.XLabel.String = 'X (cm)';
            app.BrazoAxes.YLabel.String = 'Y (cm)';
            app.BrazoAxes.XLim = [-40 40];
            app.BrazoAxes.YLim = [-40 40];
            app.BrazoAxes.Box = 'on';
            grid(app.BrazoAxes, 'on');
            
            % Hacer visible la UIFigure después de crear todos los componentes
            app.UIFigure.Visible = 'on';
        end
    end
    
    % Métodos públicos
    methods (Access = public)
        
        % Constructora
        function app = BrazoRoboticoApp
            % Crear componentes de UI
            createComponents(app)
            
            % Inicializar
            app.actualizarBrazo();
            
            % Registrar la app con App Designer
            registerApp(app, app.UIFigure)
        end
        
        % Código que se ejecuta cuando se cierra la app
        function delete(app)
            % Eliminar UIFigure cuando la app se cierra
            delete(app.UIFigure)
        end
    end
end