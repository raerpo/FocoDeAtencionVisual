clear all
clc

imagenPrueba = rgb2gray(imread('imagen0_26.jpeg'));

numPlantillas = 200;


%Plantilla 0 Grados
prob0Grados = 0;
for i=1:numPlantillas,
    imagenPlantilla = rgb2gray(imread(strcat('0GradosPlantillas/imagen0_',num2str(i),'.jpeg')));
    prob0Grados = prob0Grados + corr2(imagenPrueba, imagenPlantilla);
end
prob0Grados = prob0Grados/numPlantillas;

%Plantilla 45 Grados
prob45Grados = 0;
for i=1:numPlantillas,
    imagenPlantilla = rgb2gray(imread(strcat('45GradosPlantillas/imagen0_',num2str(i),'.jpeg')));
    prob45Grados = prob45Grados + corr2(imagenPrueba, imagenPlantilla);
end
prob45Grados = prob45Grados/numPlantillas;

%Plantilla 90 Grados
prob90Grados = 0;
for i=1:numPlantillas,
    imagenPlantilla = rgb2gray(imread(strcat('90GradosPlantillas/imagen0_',num2str(i),'.jpeg')));
    prob90Grados = prob90Grados + corr2(imagenPrueba, imagenPlantilla);
end
prob90Grados = prob90Grados/numPlantillas;

%Plantilla 135 Grados
prob135Grados = 0;
for i=1:numPlantillas,
    imagenPlantilla = rgb2gray(imread(strcat('135GradosPlantillas/imagen0_',num2str(i),'.jpeg')));
    prob135Grados = prob135Grados + corr2(imagenPrueba, imagenPlantilla);
end
prob135Grados = prob135Grados/numPlantillas;

%Plantilla 180 Grados
prob180Grados = 0;
for i=1:numPlantillas,
    imagenPlantilla = rgb2gray(imread(strcat('180GradosPlantillas/imagen0_',num2str(i),'.jpeg')));
    prob180Grados = prob180Grados + corr2(imagenPrueba, imagenPlantilla);
end
prob180Grados = prob180Grados/numPlantillas;

%Plantilla 225 Grados
prob225Grados = 0;
for i=1:numPlantillas,
    imagenPlantilla = rgb2gray(imread(strcat('225GradosPlantillas/imagen0_',num2str(i),'.jpeg')));
    prob225Grados = prob225Grados + corr2(imagenPrueba, imagenPlantilla);
end
prob225Grados = prob225Grados/numPlantillas;

%Plantilla 270 Grados
prob270Grados = 0;
for i=1:numPlantillas,
    imagenPlantilla = rgb2gray(imread(strcat('270GradosPlantillas/imagen0_',num2str(i),'.jpeg')));
    prob270Grados = prob270Grados + corr2(imagenPrueba, imagenPlantilla);
end
prob270Grados = prob270Grados/numPlantillas;

%Plantilla 315 Grados
prob315Grados = 0;
for i=1:numPlantillas,
    imagenPlantilla = rgb2gray(imread(strcat('315GradosPlantillas/imagen0_',num2str(i),'.jpeg')));
    prob315Grados = prob315Grados + corr2(imagenPrueba, imagenPlantilla);
end
prob315Grados = prob315Grados/numPlantillas;

salida = [prob0Grados, prob45Grados, prob90Grados, prob135Grados, prob180Grados, prob225Grados, prob270Grados, prob315Grados];
x = [0, 45, 90, 135, 180, 225, 270, 315];
subplot(1,2,2);
bar(x, salida)
ylabel('Promedio de coeficientes de correlación');
xlabel('Angulo de la cabeza');


imagen = subplot(1,2,1);
maximo = max(salida);
indice = find(salida == maximo);
imshow(imagenPrueba);

title(imagen,num2str(x(indice)));