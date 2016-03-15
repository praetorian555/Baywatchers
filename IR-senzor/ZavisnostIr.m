clear all
close all
clc

%% srednje vrednosti, dobijene na osnovu merenja

cm80 = [856, 850, 854, 849, 849];
cm70 = [866, 860, 870, 868, 861];
cm60 = [888, 890, 891, 887, 888];
cm50 = [934, 936, 935, 937, 927];
cm40 = [1042, 1043, 1033, 1021, 1026];
cm30 = [1192, 1171, 1174, 1163, 1168];
cm26 = [1378, 1398, 1396, 1277, 1296];
cm24 = [1497, 1478, 1457, 1449, 1445];
cm22 = [1613, 1571, 1595, 1612, 1627];
cm20 = [1628, 1611, 1577, 1587, 1563];
cm18 = [1934, 1958, 1987, 1952, 1960];
cm16 = [2223, 2203, 2193, 2188, 2205];
cm14 = [2574, 2634, 2597, 2584, 2557];
cm12 = [3122, 3045, 3041, 2960, 2946];
cm10 = [3427, 3479, 3477, 3480, 3380];

mean80 = round(mean(cm80));
mean70 = round(mean(cm70));
mean60 = round(mean(cm60));
mean50 = round(mean(cm50));
mean40 = round(mean(cm40));
mean30 = round(mean(cm30));
mean26 = round(mean(cm26));
mean24 = round(mean(cm24));
mean22 = round(mean(cm22));
mean20 = round(mean(cm20));
mean18 = round(mean(cm18));
mean16 = round(mean(cm16));
mean14 = round(mean(cm14));
mean12 = round(mean(cm12));
mean10 = round(mean(cm10));

x = [10,12,14,16,18,20,22,24,26,30,40,50,60,70,80];
y = [mean10,mean12,mean14,mean16,mean18, mean20,mean22,mean24,mean26, mean30, mean40, mean50, mean60, mean70, mean80];

figure(1)
plot(x,y);
title('Zavisnost ir senzora od rastojanja');