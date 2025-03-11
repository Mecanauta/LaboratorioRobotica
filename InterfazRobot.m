 % SCRIPT DEL BRAZO ROBOTICO DE 4DOF CON INTERFAZ, INTEGRACIÓN SIMSCAPE Y CONTROL ESP32
clear all
clear force
clc

% Crear función principal para evitar problemas de ámbito de variables
function brazo_robotico_gui_main()
    % Crear la figura principal
    fig = figure('Position', [300 300 1300 650], 'Name', 'Control de Brazo Robótico 4DOF', 'NumberTitle', 'off');

    % Panel para controles
    panel_control = uipanel('Title', 'Control de Ángulos', 'Position', [0.05 0.05 0.25 0.9]);

    % Panel para matriz de cinemática directa
    panel_matriz = uipanel('Title', 'Cinemática Directa', 'Position', [0.32 0.05 0.28 0.9]);
    
    % Panel para comunicación ESP32
    panel_esp32 = uipanel('Title', 'Comunicación ESP32', 'Position', [0.62 0.7 0.35 0.25]);

    % Longitudes de los eslabones (en cm) - variables globales
    global L1 L2 L3 L4 d1 d2 d3 d4 alpha_1 alpha_2 alpha_3 alpha_4 a_1 a_2 a_3 a_4;
    L1 = 16.3;    % cm
    L2 = 14.99 ;  % cm
    L3 = 15.12;     % cm
    L4 = 15.5;     % cm - Nuevo eslabón

    % Parámetros DH
    a_1 = 0;
    a_2 = L2;
    a_3 = L3;
    a_4 = L4;
    d1 = L1; 
    d2 = 0;
    d3 = 0;
    d4 = 0;
    alpha_1 = deg2rad(90);
    alpha_2 = 0;
    alpha_3 = 0;
    alpha_4 = 0;
    
    % Manejadores de componentes GUI
    global edit1 edit2 edit3 edit4 matriz_row1 matriz_row2 matriz_row3 matriz_row4 pos_text ax estado_simscape sim_mode tiempo_sim;
    global serial_port puerto_com status_com baud_rate feedback_text robot_mode;

    % Campos de texto para cada ángulo
    % Ángulo 1
    uicontrol('Parent', panel_control, 'Style', 'text', 'String', 'Ángulo 1 (θ1)', ...
        'Position', [20 350 100 20]);
    edit1 = uicontrol('Parent', panel_control, 'Style', 'edit', ...
        'Position', [20 330 100 20], ...
        'String', '15');
    text1 = uicontrol('Parent', panel_control, 'Style', 'text', ...
        'Position', [130 330 50 20], 'String', '°');

    % Ángulo 2
    uicontrol('Parent', panel_control, 'Style', 'text', 'String', 'Ángulo 2 (θ2)', ...
        'Position', [20 290 100 20]);
    edit2 = uicontrol('Parent', panel_control, 'Style', 'edit', ...
        'Position', [20 270 100 20], ...
        'String', '20');
    text2 = uicontrol('Parent', panel_control, 'Style', 'text', ...
        'Position', [130 270 50 20], 'String', '°');

    % Ángulo 3
    uicontrol('Parent', panel_control, 'Style', 'text', 'String', 'Ángulo 3 (θ3)', ...
        'Position', [20 230 100 20]);
    edit3 = uicontrol('Parent', panel_control, 'Style', 'edit', ...
        'Position', [20 210 100 20], ...
        'String', '90');
    text3 = uicontrol('Parent', panel_control, 'Style', 'text', ...
        'Position', [130 210 50 20], 'String', '°');
    
    % Ángulo 4 
    uicontrol('Parent', panel_control, 'Style', 'text', 'String', 'Ángulo 4 (θ4)', ...
        'Position', [20 170 100 20]);
    edit4 = uicontrol('Parent', panel_control, 'Style', 'edit', ...
        'Position', [20 150 100 20], ...
        'String', '0');
    text4 = uicontrol('Parent', panel_control, 'Style', 'text', ...
        'Position', [130 150 50 20], 'String', '°');
        
    % Tiempo de simulación
    uicontrol('Parent', panel_control, 'Style', 'text', 'String', 'Tiempo Sim (s)', ...
        'Position', [150 230 100 20]);
    tiempo_sim = uicontrol('Parent', panel_control, 'Style', 'edit', ...
        'Position', [150 210 100 20], ...
        'String', '10');

    % Selector para modo de simulación
    uicontrol('Parent', panel_control, 'Style', 'text', 'String', 'Modo Simulación', ...
        'Position', [150 290 100 20]);
    sim_mode = uicontrol('Parent', panel_control, 'Style', 'popup', ...
        'Position', [150 260 100 30], ...
        'String', {'Simulación completa', 'Paso a paso'}, ...
        'Value', 1, 'Tag', 'sim_mode');
    
    % Modo de operación 
    uicontrol('Parent', panel_control, 'Style', 'text', 'String', 'Modo de Operación', ...
        'Position', [150 170 100 20]);
    robot_mode = uicontrol('Parent', panel_control, 'Style', 'popup', ...
        'Position', [150 150 100 20], ...
        'String', {'Simulación', 'Robot Real'}, ...
        'Value', 1, 'Tag', 'robot_mode', ...
        'Callback', @cambiarModoOperacion);

    % Botón para actualizar
    uicontrol('Parent', panel_control, 'Style', 'pushbutton', ...
        'Position', [20 100 100 30], ...
        'String', 'Actualizar', ...
        'Callback', @actualizarBrazo);
    
    % Botón para ejecutar movimiento
    uicontrol('Parent', panel_control, 'Style', 'pushbutton', ...
        'Position', [150 100 100 30], ...
        'String', 'Ejecutar Movimiento', ...
        'Callback', @ejecutarMovimiento);
    
    % Botón para simular en Simscape
    uicontrol('Parent', panel_control, 'Style', 'pushbutton', ...
        'Position', [20 60 100 30], ...
        'String', 'Simular Modelo', ...
        'Callback', @iniciarSimulacion);
        
    % Botón para detener simulación o movimiento
    uicontrol('Parent', panel_control, 'Style', 'pushbutton', ...
        'Position', [150 60 100 30], ...
        'String', 'Detener', ...
        'Callback', @detenerOperacion);
    
    % Botón de emergencia para robot real
    uicontrol('Parent', panel_control, 'Style', 'pushbutton', ...
        'Position', [85 15 100 30], ...
        'String', 'PARADA EMERGENCIA', ...
        'BackgroundColor', [0.8 0 0], ...
        'ForegroundColor', [1 1 1], ...
        'FontWeight', 'bold', ...
        'Callback', @paradaEmergencia);

    % Crear textos para la matriz
    titulo_matriz = uicontrol('Parent', panel_matriz, 'Style', 'text', ...
        'Position', [10 400 260 50], ...
        'String', 'Matriz de Transformación Homogénea', ...
        'FontSize', 10, 'FontWeight', 'bold');

    % Crear 4 textos separados para cada fila de la matriz
    matriz_row1 = uicontrol('Parent', panel_matriz, 'Style', 'text', ...
        'Position', [10 350 300 30], ...
        'String', '', 'FontName', 'Courier New', 'FontSize', 10, 'HorizontalAlignment', 'left');

    matriz_row2 = uicontrol('Parent', panel_matriz, 'Style', 'text', ...
        'Position', [10 320 300 30], ...
        'String', '', 'FontName', 'Courier New', 'FontSize', 10, 'HorizontalAlignment', 'left');

    matriz_row3 = uicontrol('Parent', panel_matriz, 'Style', 'text', ...
        'Position', [10 290 300 30], ...
        'String', '', 'FontName', 'Courier New', 'FontSize', 10, 'HorizontalAlignment', 'left');

    matriz_row4 = uicontrol('Parent', panel_matriz, 'Style', 'text', ...
        'Position', [10 260 300 30], ...
        'String', '', 'FontName', 'Courier New', 'FontSize', 10, 'HorizontalAlignment', 'left');

    % Texto para posición del efector final
    titulo_pos = uicontrol('Parent', panel_matriz, 'Style', 'text', ...
        'Position', [10 220 260 30], ...
        'String', 'Posición del Efector Final', ...
        'FontSize', 10, 'FontWeight', 'bold');

    pos_text = uicontrol('Parent', panel_matriz, 'Style', 'text', ...
        'Position', [10 140 260 80], ...
        'String', '', 'FontName', 'Courier New', 'FontSize', 10);

    % Texto para estado de Simscape/ESP32
    titulo_estado = uicontrol('Parent', panel_matriz, 'Style', 'text', ...
        'Position', [10 100 260 30], ...
        'String', 'Estado del Sistema', ...
        'FontSize', 10, 'FontWeight', 'bold');
        
    estado_simscape = uicontrol('Parent', panel_matriz, 'Style', 'text', ...
        'Position', [10 40 260 60], ...
        'String', 'No simulado', 'FontName', 'Courier New', 'FontSize', 10, ...
        'Tag', 'estado_simscape');

    % Configuración ESP32 
    % Puerto COM
    uicontrol('Parent', panel_esp32, 'Style', 'text', 'String', 'Puerto COM:', ...
        'Position', [10 120 80 20]);
    puerto_com = uicontrol('Parent', panel_esp32, 'Style', 'edit', ...
        'Position', [100 120 80 20], ...
        'String', 'COM4');
    
    % Velocidad de comunicación
    uicontrol('Parent', panel_esp32, 'Style', 'text', 'String', 'Baudios:', ...
        'Position', [190 120 60 20]);
    baud_rate = uicontrol('Parent', panel_esp32, 'Style', 'popup', ...
        'Position', [260 120 70 20], ...
        'String', {'9600', '19200', '38400', '57600', '115200'}, ...
        'Value', 5);  % 115200 por defecto
    
    % Botones de conexión
    uicontrol('Parent', panel_esp32, 'Style', 'pushbutton', ...
        'Position', [10 80 100 30], ...
        'String', 'Conectar ESP32', ...
        'Callback', @conectarESP32);
    
    uicontrol('Parent', panel_esp32, 'Style', 'pushbutton', ...
        'Position', [120 80 100 30], ...
        'String', 'Desconectar', ...
        'Callback', @desconectarESP32);
    
    % Estado de conexión
    uicontrol('Parent', panel_esp32, 'Style', 'text', 'String', 'Estado Conexión:', ...
        'Position', [10 40 100 20]);
    status_com = uicontrol('Parent', panel_esp32, 'Style', 'text', ...
        'Position', [120 40 200 20], ...
        'String', 'No conectado', ...
        'HorizontalAlignment', 'left', ...
        'ForegroundColor', [0.7 0 0]);
    
    % Panel para retroalimentación de posición real
    uicontrol('Parent', panel_esp32, 'Style', 'text', ...
        'Position', [10 15 80 20], ...
        'String', 'Feedback:', ...
        'HorizontalAlignment', 'left');
    feedback_text = uicontrol('Parent', panel_esp32, 'Style', 'text', ...
        'Position', [100 5 230 30], ...
        'String', 'Sin datos', ...
        'HorizontalAlignment', 'left');
    
    % Área de visualización
    ax = axes('Position', [0.62 0.1 0.35 0.55]);
    title(ax, 'Visualización del Brazo Robótico 4DOF');
    xlabel(ax, 'X (cm)');
    ylabel(ax, 'Y (cm)');
    zlabel(ax, 'Z (cm)');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 3); % Vista 3D para mejor visualización del 4DOF
    xlim(ax, [-100 100]);
    ylim(ax, [-100 100]);
    zlim(ax, [-100 100]);
    
    % Inicializar variables de control (NUEVAS)
    serial_port = []; % Puerto serie para comunicación ESP32
    timer_feedback = []; % Timer para actualización de feedback
    
    % Inicializar la visualización
    actualizarBrazo([], []);
    
    % Configurar cierre de figura para limpiar recursos
    set(fig, 'CloseRequestFcn', @cerrarAplicacion);
end

% Función para cambiar el modo de operación
function cambiarModoOperacion(src, ~)
    global robot_mode estado_simscape;
    
    modo = get(robot_mode, 'Value');
    if modo == 1 % Simulación
        set(estado_simscape, 'String', 'Modo simulación activado');
    else % Robot Real
        % Verificar conexión con ESP32
        if ~isSerialConnected()
            set(robot_mode, 'Value', 1); % Volver a modo simulación
            warndlg('Debe conectar el ESP32 antes de cambiar a modo Robot Real', 'Conexión requerida');
            set(estado_simscape, 'String', 'Conexión con ESP32 requerida para modo Robot Real');
        else
            set(estado_simscape, 'String', 'Modo Robot Real activado. Esperando comandos.');
        end
    end
end

% Función para verificar conexión serial
function connected = isSerialConnected()
    global serial_port;
    connected = ~isempty(serial_port) && isvalid(serial_port) && strcmpi(get(serial_port, 'Status'), 'open');
end

% Función para actualizar la visualización
function actualizarBrazo(~, ~)
    global L1 L2 L3 L4 d1 d2 d3 d4 alpha_1 alpha_2 alpha_3 alpha_4 a_1 a_2 a_3 a_4;
    global edit1 edit2 edit3 edit4 matriz_row1 matriz_row2 matriz_row3 matriz_row4 pos_text ax;
    
    % Obtener valores de los campos de texto
    theta1 = str2double(get(edit1, 'String'));
    theta2 = str2double(get(edit2, 'String'));
    theta3 = str2double(get(edit3, 'String'));
    theta4 = str2double(get(edit4, 'String'));
    
    % Validar entradas numéricas
    if isnan(theta1) || isnan(theta2) || isnan(theta3) || isnan(theta4)
        warndlg('Por favor ingrese valores numéricos válidos para todos los ángulos', 'Entrada inválida');
        return;
    end
    
    % Verificar rango de valores (ajustar según límites mecánicos)
    if any([abs(theta1) > 180, abs(theta2) > 180, abs(theta3) > 180, abs(theta4) > 180])
        warndlg('Los ángulos deben estar en el rango de -180° a 180°', 'Rango inválido');
        return;
    end
    
    % Convertir a radianes para los cálculos
    Theta_1 = deg2rad(theta1);
    Theta_2 = deg2rad(theta2);
    Theta_3 = deg2rad(theta3);
    Theta_4 = deg2rad(theta4); 
    
    % Crear variables en el workspace para Simulink/Simscape
    % Crear valores de tiempo para simulación (0 a 10 segundos)
    t = [0; 10];
    
    % Crear señales constantes para los ángulos (en grados para Simscape)
    senal_angulo1 = [theta1; theta1];
    senal_angulo2 = [theta2; theta2];
    senal_angulo3 = [theta3; theta3];
    senal_angulo4 = [theta4; theta4]; % NUEVA señal para ángulo 4
    
    % Asignar al workspace
    assignin('base', 'tiempo', t);
    assignin('base', 'senal_angulo1', [t, senal_angulo1]);
    assignin('base', 'senal_angulo2', [t, senal_angulo2]);
    assignin('base', 'senal_angulo3', [t, senal_angulo3]);
    assignin('base', 'senal_angulo4', [t, senal_angulo4]); % NUEVA señal para ángulo 4
    
    % También asignar ángulos en radianes para uso en Simscape si es necesario
    assignin('base', 'angulo_revolute1', Theta_1);
    assignin('base', 'angulo_revolute2', Theta_2);
    assignin('base', 'angulo_revolute3', Theta_3);
    assignin('base', 'angulo_revolute4', Theta_4); % NUEVO ángulo 4

    % Inicializar o actualizar la estructura simData
    if ~evalin('base', 'exist(''simData'', ''var'')')
        simData = struct();
    else
        simData = evalin('base', 'simData');
    end
    
    % Configurar datos para transformaciones rígidas (ahora con 4 articulaciones)
    simData.RigidTransform(1).axis = [0 0 1];  % Eje Z
    simData.RigidTransform(1).angle = Theta_1;
    simData.RigidTransform(1).translation = [L1*10 0 0];  % Convertir a mm
    
    simData.RigidTransform(2).axis = [0 0 1];
    simData.RigidTransform(2).angle = Theta_2;
    simData.RigidTransform(2).translation = [L2*10 0 0];
    
    simData.RigidTransform(3).axis = [0 0 1];
    simData.RigidTransform(3).angle = Theta_3;
    simData.RigidTransform(3).translation = [L3*10 0 0];
    
    % NUEVA transformación para el cuarto ángulo
    simData.RigidTransform(4).axis = [0 0 1];
    simData.RigidTransform(4).angle = Theta_4;
    simData.RigidTransform(4).translation = [L4*10 0 0];

    % Enviar al workspace
    assignin('base', 'simData', simData);
    
    % Actualizar estado
    estado_simscape = findobj('Tag', 'estado_simscape');
    set(estado_simscape, 'String', 'Ángulos actualizados en workspace. Listo para simular.');
    
    % Calcular matrices de transformación (DH)
    A1 = [cos(Theta_1) -sin(Theta_1)*cos(alpha_1) sin(Theta_1)*sin(alpha_1) a_1*cos(Theta_1);
          sin(Theta_1) cos(Theta_1)*cos(alpha_1) -cos(Theta_1)*sin(alpha_1) a_1*sin(Theta_1);
          0 sin(alpha_1) cos(alpha_1) d1;
          0 0 0 1];
    
    A2 = [cos(Theta_2) -sin(Theta_2)*cos(alpha_2) sin(Theta_2)*sin(alpha_2) a_2*cos(Theta_2);
          sin(Theta_2) cos(Theta_2)*cos(alpha_2) -cos(Theta_2)*sin(alpha_2) a_2*sin(Theta_2);
          0 sin(alpha_2) cos(alpha_2) d2;
          0 0 0 1];
    
    A3 = [cos(Theta_3) -sin(Theta_3)*cos(alpha_3) sin(Theta_3)*sin(alpha_3) a_3*cos(Theta_3);
          sin(Theta_3) cos(Theta_3)*cos(alpha_3) -cos(Theta_3)*sin(alpha_3) a_3*sin(Theta_3);
          0 sin(alpha_3) cos(alpha_3) d3;
          0 0 0 1];
    
    % NUEVA matriz para el cuarto ángulo
    A4 = [cos(Theta_4) -sin(Theta_4)*cos(alpha_4) sin(Theta_4)*sin(alpha_4) a_4*cos(Theta_4);
          sin(Theta_4) cos(Theta_4)*cos(alpha_4) -cos(Theta_4)*sin(alpha_4) a_4*sin(Theta_4);
          0 sin(alpha_4) cos(alpha_4) d4;
          0 0 0 1];
    
    % Calcular matrices de transformación acumuladas
    T01 = A1;
    T02 = A1*A2;
    T03 = A1*A2*A3;
    T04 = A1*A2*A3*A4; % NUEVA matriz acumulada total (hasta el 4° ángulo)
    
    % Actualizar interfaz con la matriz de transformación total (T04)
    set(matriz_row1, 'String', sprintf('╔ %6.3f  %6.3f  %6.3f  %6.1f ╗', T04(1,1), T04(1,2), T04(1,3), T04(1,4)));
    set(matriz_row2, 'String', sprintf('║ %6.3f  %6.3f  %6.3f  %6.1f ║', T04(2,1), T04(2,2), T04(2,3), T04(2,4)));
    set(matriz_row3, 'String', sprintf('║ %6.3f  %6.3f  %6.3f  %6.1f ║', T04(3,1), T04(3,2), T04(3,3), T04(3,4)));
    set(matriz_row4, 'String', sprintf('╚ %6.3f  %6.3f  %6.3f  %6.1f ╝', T04(4,1), T04(4,2), T04(4,3), T04(4,4)));
    
    % Actualizar posición del efector final
    pos_str = sprintf('X = %6.1f cm\nY = %6.1f cm\nZ = %6.1f cm', T04(1,4), T04(2,4), T04(3,4));
    set(pos_text, 'String', pos_str);
    
    % Obtener coordenadas de las articulaciones usando matrices DH
    x0 = 0;
    y0 = 0;
    z0 = 0;
    
    x1 = T01(1,4);
    y1 = T01(2,4);
    z1 = T01(3,4);
    
    x2 = T02(1,4);
    y2 = T02(2,4);
    z2 = T02(3,4);
    
    x3 = T03(1,4);
    y3 = T03(2,4);
    z3 = T03(3,4);
    
    x4 = T04(1,4);
    y4 = T04(2,4);
    z4 = T04(3,4);
    
    % Dibujar el brazo
    cla(ax);
    hold(ax, 'on');
    
    % Dibujar eslabones con líneas de colores (ahora con 4 eslabones)
    h1 = plot3(ax, [x0 x1], [y0 y1], [z0 z1], 'b-', 'LineWidth', 3); % Eslabón 1 (base)
    h2 = plot3(ax, [x1 x2], [y1 y2], [z1 z2], 'r-', 'LineWidth', 3); % Eslabón 2
    h3 = plot3(ax, [x2 x3], [y2 y3], [z2 z3], 'g-', 'LineWidth', 3); % Eslabón 3
    h4 = plot3(ax, [x3 x4], [y3 y4], [z3 z4], 'm-', 'LineWidth', 3); % Eslabón 4 (NUEVO)
    
    % Agregar marcadores en las juntas
    joint_markers = plot3(ax, [x0 x1 x2 x3], [y0 y1 y2 y3], [z0 z1 z2 z3], 'ko', 'MarkerFaceColor', 'black', 'MarkerSize', 10);
    
    % Destacar la posición final
    efector_final = plot3(ax, x4, y4, z4, 'ko', 'MarkerFaceColor', 'black', 'MarkerSize', 12);
    
    % Agregar marcadores en los ejes para el efector final
    tam_marker = 2;
    plot3(ax, [x4-tam_marker x4+tam_marker], [y4 y4], [z4 z4], 'r-', 'LineWidth', 2); % Marca X
    plot3(ax, [x4 x4], [y4-tam_marker y4+tam_marker], [z4 z4], 'g-', 'LineWidth', 2); % Marca Y
    plot3(ax, [x4 x4], [y4 y4], [z4-tam_marker z4+tam_marker], 'b-', 'LineWidth', 2); % Marca Z
    
    % Agregar círculo alrededor de la posición final
    t = linspace(0, 2*pi, 100);
    radio = 1.5;
    circle_x = x4 + radio * cos(t);
    circle_y = y4 + radio * sin(t);
    circle_z = z4 * ones(size(t));
    plot3(ax, circle_x, circle_y, circle_z, 'm:', 'LineWidth', 2);
    
    % Mostrar coordenadas cerca del efector final
    text(ax, x4+2, y4, z4, sprintf('(%0.1f, %0.1f, %0.1f)', x4, y4, z4), 'FontSize', 8);
    
    % Configurar ejes
    axis(ax, 'equal');
    grid(ax, 'on');
    xlim(ax, [-70 70]);
    ylim(ax, [-70 70]);
    zlim(ax, [0 70]);
    
    % Mejorar visualización 3D
    view(ax, [45 30]); % Vista isométrica
    
    % También mostrar en la consola
    disp('Matriz de Transformación Homogénea (T04):');
    disp(T04);
    
    disp('Posición del Efector Final:');
    fprintf('X = %6.1f cm\n', T04(1,4));
    fprintf('Y = %6.1f cm\n', T04(2,4));
    fprintf('Z = %6.1f cm\n', T04(3,4));
end

% Función para iniciar la simulación en Simscape
function iniciarSimulacion(~, ~)
    global tiempo_sim robot_mode;
    
    % Si estamos en modo robot real, no iniciar simulación
    if get(robot_mode, 'Value') == 2
        warndlg('Cambie a modo Simulación para iniciar la simulación Simscape', 'Modo incorrecto');
        return;
    end
    
    % Obtener tiempo de simulación
    tsim = str2double(get(tiempo_sim, 'String'));
    if isnan(tsim) || tsim <= 0
        set(tiempo_sim, 'String', '10');
        tsim = 10;
        warndlg('El tiempo de simulación debe ser positivo', 'Tiempo inválido');
    end
    
    % Obtener modo de simulación
    sim_mode_obj = findobj('Tag', 'sim_mode');
    sim_mode_val = get(sim_mode_obj, 'Value');
    
    % Actualizar estado
    estado_simscape = findobj('Tag', 'estado_simscape');
    set(estado_simscape, 'String', 'Iniciando simulación...');
    drawnow;
    
    try
        % Verificar si el modelo está cargado, si no, cargarlo
        if ~bdIsLoaded('BrazoCoplanar4DOF')
            % Intentar cargar el modelo
            try
                load_system('BrazoCoplanar4DOF');
                set(estado_simscape, 'String', 'Modelo cargado correctamente');
            catch
                % Si no existe, mostrar mensaje para crearlo
                warndlg(['El modelo "BrazoCoplanar4DOF" no existe. ', ...
                    'Por favor cree el modelo Simscape con 4DOF según la documentación.'], ...
                    'Modelo no encontrado');
                set(estado_simscape, 'String', 'Error: Modelo no encontrado');
                return;
            end
        end
        
        % Configurar tiempo de simulación
        set_param('BrazoCoplanar4DOF', 'StopTime', num2str(tsim));
        
        % Si es paso a paso, configurar simulación en modo paso a paso
        if sim_mode_val == 2 % Paso a paso
            set_param('BrazoCoplanar4DOF', 'SimulationMode', 'stepped');
        else
            set_param('BrazoCoplanar4DOF', 'SimulationMode', 'normal');
        end
        
        % Iniciar simulación
        set(estado_simscape, 'String', 'Simulando...');
        drawnow;
        
        sim('BrazoCoplanar4DOF');
        
        % Comprobar si existen resultados de simulación
        if evalin('base', 'exist(''simout'', ''var'')')
            leerResultadosSimulacion();
        else
            set(estado_simscape, 'String', 'Simulación completada. No hay datos de salida disponibles.');
        end
        
    catch e
        errordlg(['Error al simular: ' e.message], 'Error de Simulación');
        set(estado_simscape, 'String', ['Error: ' e.message]);
    end
end

% Función para leer resultados de la simulación
function leerResultadosSimulacion()
    estado_simscape = findobj('Tag', 'estado_simscape');
    
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
                    set(estado_simscape, 'String', sprintf('Simulación completada.\nTiempo: %.2fs\nPos. Final (Simscape):\nX: %.2f\nY: %.2f\nZ: %.2f', ...
                        tiempo_final, pos_final(1), pos_final(2), pos_final(3)));
                else
                    set(estado_simscape, 'String', 'Simulación completada.\nEstructura de datos no reconocida.');
                end
            else
                set(estado_simscape, 'String', 'Simulación completada.\nDatos de salida en formato no reconocido.');
            end
        else
            set(estado_simscape, 'String', 'Simulación completada.\nNo hay datos de salida disponibles.');
        end
    catch e
        set(estado_simscape, 'String', ['Error al leer resultados: ' e.message]);
    end
end

% Función para detener operaciones (simulación o movimiento robot)
function detenerOperacion(~, ~)
    global robot_mode serial_port;
    
    % Determinar qué detener según el modo
    if get(robot_mode, 'Value') == 1
        % Detener simulación Simscape
        try
            % Verificar si hay modelos cargados
            modelos = {'BrazoCoplanar4DOF', 'BrazoCoplanar1'};
            for i = 1:length(modelos)
                if bdIsLoaded(modelos{i})
                    if strcmp(get_param(modelos{i}, 'SimulationStatus'), 'running')
                        set_param(modelos{i}, 'SimulationCommand', 'stop');
                        estado_simscape = findobj('Tag', 'estado_simscape');
                        set(estado_simscape, 'String', 'Simulación detenida por el usuario');
                    end
                end
            end
        catch e
            warndlg(['Error al detener simulación: ' e.message], 'Error');
        end
    else
        % Detener movimiento del robot real
        if isSerialConnected()
            try
                % Enviar comando de parada al ESP32
                fprintf(serial_port, 'STOP\n');
                estado_simscape = findobj('Tag', 'estado_simscape');
                set(estado_simscape, 'String', 'Movimiento detenido');
            catch e
                warndlg(['Error al detener robot: ' e.message], 'Error de comunicación');
            end
        end
    end
end

% Función para parada de emergencia
function paradaEmergencia(~, ~)
    global serial_port;
    
    % Intentar detener simulaciones en curso
    try
        modelos = {'BrazoCoplanar4DOF', 'BrazoCoplanar1'};
        for i = 1:length(modelos)
            if bdIsLoaded(modelos{i})
                if strcmp(get_param(modelos{i}, 'SimulationStatus'), 'running')
                    set_param(modelos{i}, 'SimulationCommand', 'stop');
                end
            end
        end
    catch
        % Ignorar errores y continuar con la parada de emergencia
    end
    
    % Enviar señal de emergencia al ESP32 si está conectado
    if isSerialConnected()
        try
            % Enviar comando de emergencia con triple confirmación
            fprintf(serial_port, 'EMERGENCY\n');
            pause(0.1);
            fprintf(serial_port, 'EMERGENCY\n');
            pause(0.1);
            fprintf(serial_port, 'EMERGENCY\n');
            
            % Actualizar estados
            estado_simscape = findobj('Tag', 'estado_simscape');
            set(estado_simscape, 'String', 'PARADA DE EMERGENCIA ACTIVADA', 'ForegroundColor', [0.8 0 0]);
            status_com = findobj('Tag', 'status_com');
            set(status_com, 'String', 'Parada de emergencia', 'ForegroundColor', [0.8 0 0]);
        catch
            % Intentar cerrar la conexión como último recurso
            try
                fclose(serial_port);
            catch
                % Ignorar errores
            end
        end
    end
    
    % Mostrar mensaje de emergencia
    h = warndlg('PARADA DE EMERGENCIA ACTIVADA. Revise el robot antes de reanudar.', 'EMERGENCIA', 'modal');
    setappdata(0, 'EmergencyDialog', h);
end

% Función para conectar con ESP32
function conectarESP32(~, ~)
    global serial_port puerto_com baud_rate status_com timer_feedback;
    
    % Si ya hay una conexión abierta, cerrarla primero
    if isSerialConnected()
        try
            fclose(serial_port);
            delete(serial_port);
            serial_port = [];
        catch
            % Ignorar errores al cerrar
        end
    end
    
    % Detener timer si existe
    if ~isempty(timer_feedback) && isvalid(timer_feedback)
        stop(timer_feedback);
        delete(timer_feedback);
    end
    
    % Obtener puerto y velocidad
    port = get(puerto_com, 'String');
    baud_rates = get(baud_rate, 'String');
    selected_baud = str2double(baud_rates{get(baud_rate, 'Value')});
    
    % Actualizar estado
    set(status_com, 'String', 'Conectando...', 'ForegroundColor', [0.7 0.7 0]);
    drawnow;
    
    try
        % Crear objeto de puerto serie
        serial_port = serialport(port, selected_baud);
        
        % Configurar timeout
        configureTerminator(serial_port, "LF");
        serial_port.Timeout = 2;  % 2 segundos de timeout
        
        % Verificar conexión enviando un comando de prueba
        flush(serial_port);
        writeline(serial_port, "TEST");
        
        % Esperar respuesta con timeout
        tic;
        while toc < 2
            if serial_port.NumBytesAvailable > 0
                response = readline(serial_port);
                if contains(response, "OK")
                    % Conexión exitosa
                    set(status_com, 'String', ['Conectado a ' port], 'ForegroundColor', [0 0.7 0]);
                    
                    % Iniciar timer para recibir feedback
                    timer_feedback = timer('ExecutionMode', 'fixedRate', ...
                        'Period', 0.5, ...
                        'TimerFcn', @actualizarFeedback);
                    start(timer_feedback);
                    
                    return;
                end
            end
            pause(0.1);
        end
        
        % Si llegamos aquí, no hubo respuesta válida
        set(status_com, 'String', 'Error: No responde', 'ForegroundColor', [0.7 0 0]);
        fclose(serial_port);
        delete(serial_port);
        serial_port = [];
        
    catch e
        set(status_com, 'String', ['Error: ' e.message], 'ForegroundColor', [0.7 0 0]);
        if ~isempty(serial_port) && isvalid(serial_port)
            delete(serial_port);
        end
        serial_port = [];
    end
end

% Función para desconectar ESP32
function desconectarESP32(~, ~)
    global serial_port status_com timer_feedback robot_mode;
    
    % Cambiar a modo simulación si estamos en modo robot real
    if get(robot_mode, 'Value') == 2
        set(robot_mode, 'Value', 1);
    end
    
    % Detener timer de feedback
    if ~isempty(timer_feedback) && isvalid(timer_feedback)
        stop(timer_feedback);
        delete(timer_feedback);
        timer_feedback = [];
    end
    
    % Cerrar conexión serial
    if isSerialConnected()
        try
            % Enviar comando de desconexión al ESP32
            writeline(serial_port, "DISCONNECT");
            pause(0.2);
            
            % Cerrar conexión
            fclose(serial_port);
            delete(serial_port);
            serial_port = [];
            
            set(status_com, 'String', 'Desconectado', 'ForegroundColor', [0.7 0 0]);
        catch e
            warndlg(['Error al desconectar: ' e.message], 'Error');
            set(status_com, 'String', ['Error: ' e.message], 'ForegroundColor', [0.7 0 0]);
        end
    else
        set(status_com, 'String', 'No conectado', 'ForegroundColor', [0.7 0 0]);
    end
end

% Función para actualizar feedback del robot real
function actualizarFeedback(~, ~)
    global serial_port feedback_text;
    
    if isSerialConnected()
        try
            % Solicitar posición actual
            writeline(serial_port, "GET_POS");
            
            % Esperar respuesta con timeout
            tic;
            while toc < 0.5 && serial_port.NumBytesAvailable == 0
                pause(0.01);
            end
            
            if serial_port.NumBytesAvailable > 0
                % Leer respuesta
                response = readline(serial_port);
                
                % Procesar respuesta
                if startsWith(response, "POS:")
                    % Formato esperado: "POS:theta1,theta2,theta3,theta4"
                    data = strsplit(response(5:end), ',');
                    if length(data) >= 4
                        % Actualizar texto de feedback
                        feedback_str = sprintf('θ1:%s° θ2:%s° θ3:%s° θ4:%s°', ...
                            data{1}, data{2}, data{3}, data{4});
                        set(feedback_text, 'String', feedback_str);
                    end
                end
            end
        catch
            % Ignorar errores temporales de comunicación
        end
    end
end

% Función para ejecutar movimiento en el robot real
function ejecutarMovimiento(~, ~)
    global robot_mode serial_port edit1 edit2 edit3 edit4 estado_simscape;
    
    % Verificar modo de operación
    if get(robot_mode, 'Value') == 1
        % En modo simulación, actualizar brazo
        actualizarBrazo([], []);
        return;
    end
    
    % Verificar conexión con ESP32
    if ~isSerialConnected()
        warndlg('Robot no conectado. Conecte el ESP32 primero.', 'Conexión requerida');
        return;
    end
    
    % Obtener ángulos
    theta1 = str2double(get(edit1, 'String'));
    theta2 = str2double(get(edit2, 'String'));
    theta3 = str2double(get(edit3, 'String'));
    theta4 = str2double(get(edit4, 'String'));
    
    % Validar ángulos
    if any(isnan([theta1, theta2, theta3, theta4]))
        warndlg('Por favor ingrese valores numéricos válidos para todos los ángulos', 'Entrada inválida');
        return;
    end
    
    % Verificar rango de valores (ajustar según límites mecánicos)
    if any([abs(theta1) > 180, abs(theta2) > 180, abs(theta3) > 180, abs(theta4) > 180])
        warndlg('Los ángulos deben estar en el rango de -180° a 180°', 'Rango inválido');
        return;
    end
    
    try
        % Enviar comando de movimiento al ESP32
        command = sprintf("MOVE:%d,%d,%d,%d", round(theta1), round(theta2), round(theta3), round(theta4));
        writeline(serial_port, command);
        
        % Actualizar estado
        set(estado_simscape, 'String', 'Enviando comando de movimiento al robot...');
        
        % Esperar confirmación de recepción
        tic;
        while toc < 1
            if serial_port.NumBytesAvailable > 0
                response = readline(serial_port);
                if contains(response, "MOVE_OK")
                    set(estado_simscape, 'String', 'Movimiento iniciado. Ejecutando...');
                    return;
                elseif contains(response, "ERROR")
                    set(estado_simscape, 'String', ['Error en robot: ' response]);
                    return;
                end
            end
            pause(0.1);
        end
        
        % Si no hay respuesta en 1 segundo
        set(estado_simscape, 'String', 'Advertencia: No se recibió confirmación del robot');
    catch e
        warndlg(['Error al enviar comando: ' e.message], 'Error de comunicación');
        set(estado_simscape, 'String', ['Error de comunicación: ' e.message]);
    end
end

% Función para cerrar aplicación
function cerrarAplicacion(src, ~)
    global serial_port timer_feedback;
    
    % Detener timer de feedback
    if ~isempty(timer_feedback) && isvalid(timer_feedback)
        stop(timer_feedback);
        delete(timer_feedback);
    end
    
    % Cerrar conexión serial si está abierta
    if isSerialConnected()
        try
            % Enviar comando de desconexión al ESP32
            writeline(serial_port, "DISCONNECT");
            pause(0.2);
            
            % Cerrar conexión
            fclose(serial_port);
            delete(serial_port);
        catch
            % Ignorar errores al cerrar
        end
    end
    
    % Cerrar modelos Simulink abiertos
    try
        modelos = {'BrazoCoplanar4DOF', 'BrazoCoplanar1'};
        for i = 1:length(modelos)
            if bdIsLoaded(modelos{i})
                close_system(modelos{i}, 0);
            end
        end
    catch
        % Ignorar errores al cerrar modelos
    end
    
    % Cerrar la figura
    delete(src);
end

% Ejecutar la función principal
brazo_robotico_gui_main();