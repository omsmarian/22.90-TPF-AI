% Cargo la imagen
img = iread("img\test1.png", 'double');

%img_gamma = igamm(img, 0.45);
R = idouble(img(:,:,1));
G = idouble(img(:,:,2));
B = idouble(img(:,:,3));
mask_G = G > 0.5;
mask_R = R < 0.5;

diff_im = imsubtract(G, rgb2gray(img));
%diff_im = im2bw(diff_im,0.18);
%diff_im = bwareaopen(diff_im, ceil(300));
green = cat(3, zeros(size(R)), mask_G, zeros(size(B)));
red = cat(3, mask_R, zeros(size(G)), zeros(size(B)));

green2 = inormhist(imono(green));
red2 = inormhist(imono(red));
diff_im = imsubtract(green2, red2);

idisp(green);