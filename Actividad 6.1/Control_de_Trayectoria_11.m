% LIMPIEZA DE PANTALLA
clear all
close all
clc

% 1. TIEMPO DE SIMULACIÓN
tf = 7.4;           % Tiempo total de simulación [s]
ts = 0.01;          % Tiempo de muestreo [s] (más que suficiente)
t = 0:ts:tf;        % Vector de tiempo
N = length(t);      % Total de muestras

% 2. CONDICIONES INICIALES DEL ROBOT
x1(1) = 0;
y1(1) = 0;
phi(1) = 0;
hx(1) = x1(1);
hy(1) = y1(1);

% --- Trayectoria tipo CORAZÓN ---

% Redefinir vector de tiempo solo para 1 ciclo de 0 a 2pi
t = linspace(0, 2*pi, N);

% Ecuaciones paramétricas del corazón
hxd = 16 * sin(t).^3;
hyd = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);

% Derivadas numéricas (velocidades deseadas)
hxdp = [diff(hxd), 0] / ts;
hydp = [diff(hyd), 0] / ts;

% 4. BUCLE DE CONTROL
for k = 1:N
    % Error de posición
    hxe(k) = hxd(k) - hx(k);
    hye(k) = hyd(k) - hy(k);
    he = [hxe(k); hye(k)];
    Error(k) = norm(he);

    % Jacobiana
    J = [cos(phi(k)), -sin(phi(k));
         sin(phi(k)),  cos(phi(k))];

    % Ganancia de control
    K = [50.3 0; 0 15.4];

    % Velocidades deseadas
    hdp = [hxdp(k); hydp(k)];

    % Ley de control
    qpRef = pinv(J) * (hdp + K * he);

    v(k) = qpRef(1); % Velocidad lineal
    w(k) = qpRef(2); % Velocidad angular

    % Integración (modelo cinemático)
    phi(k+1) = phi(k) + ts * w(k);
    x1(k+1) = x1(k) + ts * v(k) * cos(phi(k));
    y1(k+1) = y1(k) + ts * v(k) * sin(phi(k));

    % Posición del punto de control
    hx(k+1) = x1(k+1);
    hy(k+1) = y1(k+1);
end

% 5. VISUALIZACIÓN 3D
scene = figure;
set(scene, 'Color', 'white');
set(gca, 'FontWeight', 'bold');
sizeScreen = get(0, 'ScreenSize');
set(scene, 'position', sizeScreen);
camlight('headlight');
axis equal
grid on
box on
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
view([-0.1 90]);
axis([-60 60 -60 60 0 1]);

% Graficar posición inicial
scale = 4;
MobileRobot_5;
H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale); hold on;

% Trayectorias
H2 = plot3(hx(1), hy(1), 0, 'r', 'LineWidth', 2);
H3 = plot3(hxd, hyd, zeros(1, N), 'g--', 'LineWidth', 2);

% Animación paso a paso
step = 10;
for k = 1:step:N
    delete(H1);
    delete(H2);
    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    H2 = plot3(hx(1:k), hy(1:k), zeros(1,k), 'r', 'LineWidth', 2);
    pause(ts);
end

% 6. GRÁFICAS FINALES
graph = figure;
set(graph, 'position', sizeScreen);
subplot(311)
plot(t, v, 'b', 'LineWidth', 2), grid on
xlabel('Tiempo [s]')
ylabel('m/s')
legend('Velocidad Lineal (v)')

subplot(312)
plot(t, w, 'g', 'LineWidth', 2), grid on
xlabel('Tiempo [s]')
ylabel('rad/s')
legend('Velocidad Angular (w)')

subplot(313)
plot(t, Error, 'r', 'LineWidth', 2), grid on
xlabel('Tiempo [s]')
ylabel('Error [m]')
legend('Error de posición')
