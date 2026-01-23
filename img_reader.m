% Cargo la imagen
img = iread("img\test1.png", 'double');

% Mostrar imagen original
figure;
subplot(1,2,1);
idisp(img);
title('Imagen original');

% Histograma de imagen original
subplot(1,2,2);
ihist(img);
title('Histograma original');

stop=input('continuar?');
close all

% Corrección gamma
img = igamm(img, 0.9);

% Mostrar imagen con gamma corregido
figure;
subplot(1,2,1);
idisp(img);
title('Imagen con gamma corregido');

% Histograma después de gamma
subplot(1,2,2);
ihist(img);
title('Histograma después de gamma');

stop=input('continuar?');
close all

% Pasar a escala de grises
img_grey = imono(img);
img_grey = ihist(img_grey);


% Normalizar histograma
grey_normal = inormhist(img_grey);

% Mostrar imagen normalizada
figure;
subplot(1,2,1);
idisp(img_grey);
title('Imagen normalizada');

% Histograma normalizado
subplot(1,2,2);
ihist(grey_normal);
title('Histograma normalizado');

stop=input('continuar?');
close all


bordes = icanny(img);

idisp(bordes);
stop=input('continuar?');
close all
% Buscar los verdes en base a un threshold



% Tenemos q usar Hough para los bordes