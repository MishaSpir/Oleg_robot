clc;
close all; 
clear;

%визуальные параметры для Олега
rob_telo = [ 1  1 -1.5 -1.5 1  % x
             1 -1  -1    1  1]; % y
rob_r_coleso = [ 1  1   -1  -1 1  % x
                 1 1.5  1.5  1 1]; % y
rob_l_coleso = [  1    1   -1   -1  1  % x
                 -1  -1.5 -1.5  -1 -1]; % y


%создание пространства
my_form = figure('Position', [1000 300 500 500], 'Visible', 'off');
areaSize = 10;
grid on
axis equal
xlim([-10 areaSize])
ylim([-10 areaSize])
my_form.Visible = 1;





dt = 1; %дельта t (малый промежуток времени), сек
L = 1;  %пол-колеи, метры
R = 0.2;%радиусы колёс, метры

wl = 10; %угловая скорость левого колеса, рад/сек
wr = 20;%угловая скорость правого колеса,   рад/ сек

%начальные параметры
x_pred = 0;
y_pred = 0;
tetha_pred = pi/2;

tetha = tetha_pred;
x = x_pred;
y = y_pred;

Vl = wl*R;
Vr = wr*R;
r =  L *((Vr + Vl)/(Vr-Vl));


%(имитация чтения энкодеров)
l_l = R* wl*dt; %длина дуги, кот. прошло левое колесо
l_r = R* wr*dt; %длина дуги, кот. прошло правое колесо 

%считаем прямую кинематику
delta_tetha = (l_r - l_l)/(2*L);
delta_x = r * sin(delta_tetha);
delta_y = r - r * cos(delta_tetha);
 
%так как мы один раз задали скорости, то delta_x,delta_y,delta_tetha будут
%постоянные в бесконечном цикле незачем  их пересчитывать
 
%------------для понимания------------
%delta_x, delta_y, delta_tetha - локальные коор-ты робота
% а локальный фрейм робота перемещается и поворачивается каждый раз 
%x, y, tetha  - глобальные коо-ты робота


while(1)
    % для того, чтобы из локального фрейма получить коор-ты точек для
    % глобального фрейма, делаем вот это:
    x = delta_x * cos(tetha) - delta_y * sin(tetha) + x_pred;
    y = delta_x * sin(tetha) + delta_y * cos(tetha) + y_pred;
    tetha = tetha_pred + delta_tetha;

    %запоминаем пред. коо-ты
    y_pred = y;
    x_pred = x;
    tetha_pred = tetha;

%вектор
V_orig(1,1) = x;
V_orig(2,1) = y;
V_orig(1,2) = x+2*cos(tetha);
V_orig(2,2) = y+2*sin(tetha);



center = [x; y];  
% Матрица поворота
 R = [cos(tetha) -sin(tetha)
      sin(tetha) cos(tetha)];
%мф берём тело робота, поворачиваем его и смещаем в центр робота
 P_rotated = R * (rob_telo) + center;
 rob_r_coleso_rotated = R * rob_r_coleso + center; 
 rob_l_coleso_rotated = R * rob_l_coleso + center;
 % Очищаем и перерисовываем
 clf;
 plot(V_orig(1,:), V_orig(2,:),'r');
 hold on;
 grid on;
 plot(P_rotated(1,:), P_rotated(2,:),'b');
 hold on;
 plot(rob_r_coleso_rotated(1,:), rob_r_coleso_rotated(2,:),'b');
 hold on;
 plot(rob_l_coleso_rotated(1,:), rob_l_coleso_rotated(2,:),'b');
 xlim([-10 areaSize])
 ylim([-10 areaSize])
   pause(1);
end


