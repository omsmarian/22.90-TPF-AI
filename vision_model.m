function [P1_mm, P2_mm, debug] = vision_model(imgInput, opts)
%VISION_MODEL Detecta línea roja y devuelve puntos para el robot.
%   [P1_mm, P2_mm, debug] = VISION_MODEL(imgInput, opts)
%   imgInput: imagen RGB o path/filename.
%   opts.showFigures (default: false)
%   Si no detecta líneas, devuelve 0 en P1_mm y P2_mm.

    if nargin < 2 || isempty(opts)
        opts = struct();
    end

    if ~isfield(opts, 'showFigures')
        opts.showFigures = false;
    end

    debug = struct('ok', false, 'reason', '');

    % 1. Leer imagen
    if ischar(imgInput) || (isstring(imgInput) && isscalar(imgInput))
        img = iread(imgInput);
    else
        img = imgInput;
    end

    if opts.showFigures
        fig = figure(1);
        clf(fig);
        tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

        axOriginal = nexttile;
        imshow(img, 'Parent', axOriginal);
        title(axOriginal, 'Imagen original');

        axEdges = nexttile;
        axWarp = nexttile;
        axLine = nexttile;
    end

    % 2. Detección de verde (sobre imagen sin rojo)
    img = igamm(img, 0.75);

    I = img;
    I(:,:,2) = niblack(I(:,:,2), -0.05, 5);

    HSV = my_rgb2hsv(I);
    H = HSV(:,:,1);
    S = HSV(:,:,2);

    % 3. Realce de verde
    green_enhanced = (H > 0.25 & H < 0.65) ...
                    & (S > 0.15 & S < 0.75);

    kgaus = kgauss(5);
    green_enhanced = iconv(green_enhanced, kgaus);
    T = otsu(green_enhanced);
    green_enhanced = green_enhanced > T;

    if opts.showFigures
        imshow(green_enhanced, 'Parent', axEdges);
        title(axEdges, 'Detección de bordes');
        hold(axEdges, 'on');
    end

    % 4. Detección de blobs y esquinas extremas
    candidatos = iblobs(green_enhanced, 'class', 1, 'boundary');

    rectangulo = [];
    for i = 1:length(candidatos)
        b = candidatos(i);
        aspect_ratio = b.b / b.a;
        if b.area > 300 && b.circularity < 0.65 && (aspect_ratio >= 0.3 && aspect_ratio <= 0.65)
            rectangulo = [rectangulo; b];
        end
    end

    if isempty(rectangulo)
        P1_mm = 0;
        P2_mm = 0;
        debug.reason = 'No se detectaron esquinas con los filtros actuales.';
        return;
    end

    if length(rectangulo) < 4
        P1_mm = 0;
        P2_mm = 0;
        debug.reason = 'Se detectaron menos de 4 esquinas.';
        return;
    end

    center_u = mean([rectangulo.uc]);
    center_v = mean([rectangulo.vc]);

    if opts.showFigures
        plot(axEdges, center_u, center_v, 'bx', 'MarkerSize', 20, 'LineWidth', 2);
    end

    corners = [];
    for i = 1:length(rectangulo)
        b = rectangulo(i);
        edge = b.edge';
        dist_sq = (edge(:,1) - center_u).^2 + (edge(:,2) - center_v).^2;
        [~, idx_max] = max(dist_sq);
        corner_point = edge(idx_max, :);
        corners = [corners; corner_point];

        if opts.showFigures
            plot(axEdges, edge(:,1), edge(:,2), 'y.', 'MarkerSize', 2);
            plot(axEdges, corner_point(1), corner_point(2), 'o', ...
                'MarkerSize', 12, ...
                'MarkerFaceColor', [0.5 0 0.5], ...
                'Color', 'white');
            text(axEdges, corner_point(1)+15, corner_point(2)+15, ...
                sprintf('C%d', i), 'Color','cyan', 'FontWeight','bold');
        end
    end

    if opts.showFigures
        hold(axEdges, 'off');
    end

    if size(corners, 1) < 4
        P1_mm = 0;
        P2_mm = 0;
        debug.reason = 'No se obtuvieron 4 esquinas válidas.';
        return;
    end

    % 5. Warp de alta resolución
    corners_sorted = sortrows(corners, 2);
    top = sortrows(corners_sorted(1:2, :), 1);
    bot = sortrows(corners_sorted(3:4, :), 1);
    posi = [top(1,:)', top(2,:)', bot(2,:)', bot(1,:)'];

    ancho_px = norm(top(1,:) - top(2,:));
    alto_px = ancho_px * (150/200);

    posf = [1, ancho_px, ancho_px, 1; 
            1, 1,        alto_px,  alto_px];

    matH = homography(posi, posf);
    warped_raw = homwarp(matH, img, 'full');

    [h_orig, w_orig, ~] = size(img);
    esquinas_img_orig = [1, w_orig, w_orig, 1; 
                         1, 1, h_orig, h_orig];
    esquinas_dest = homtrans(matH, esquinas_img_orig);

    offset_u = min(esquinas_dest(1,:));
    offset_v = min(esquinas_dest(2,:));

    start_u = round(1 - offset_u) + 1;
    start_v = round(1 - offset_v) + 1;

    img_warped = warped_raw(start_v : start_v + round(alto_px) - 1, ...
                            start_u : start_u + round(ancho_px) - 1, :);

    if isempty(img_warped)
        P1_mm = 0;
        P2_mm = 0;
        debug.reason = 'Warp vacío: revisar esquinas detectadas.';
        return;
    end

    if opts.showFigures
        imshow(img_warped, 'Parent', axWarp);
        title(axWarp, sprintf('Warp (%.1f x %.1f px)', ancho_px, alto_px));
    end

    % 6. Detección de línea roja (extremos por distancia)
    R = img_warped(:,:,1);
    G = img_warped(:,:,2);
    B = img_warped(:,:,3);

    red_enhanced = R - max(G, B);

    vals = red_enhanced(:);
    th = max(vals) * 0.5;
    if th < 0.03
        th = 1;
    end

    mask_red = red_enhanced > th;
    [v_inds, u_inds] = find(mask_red);

    if isempty(u_inds)
        P1_mm = 0;
        P2_mm = 0;
        debug.reason = 'No se detectaron píxeles rojos.';
        return;
    end

    pixels = [u_inds, v_inds];
    centroide = mean(pixels);

    dist_al_centro = sum((pixels - centroide).^2, 2);
    [~, idx1] = max(dist_al_centro);
    p1 = pixels(idx1, :);

    dist_a_p1 = sum((pixels - p1).^2, 2);
    [~, idx2] = max(dist_a_p1);
    p2 = pixels(idx2, :);

    % 7. Conversión a mm
    ANCHO_REAL_MM = 200;
    ALTO_REAL_MM = 150;

    W_pix = size(img_warped, 2);
    H_pix = size(img_warped, 1);
    scale_x = ANCHO_REAL_MM / W_pix;
    scale_y = ALTO_REAL_MM / H_pix;

    P1_mm = [p1(1) * scale_x, p1(2) * scale_y];
    P2_mm = [p2(1) * scale_x, p2(2) * scale_y];

    if opts.showFigures
        imshow(img_warped, 'Parent', axLine);
        title(axLine, 'Línea medida');
        hold(axLine, 'on');

        plot(axLine, [p1(1) p2(1)], [p1(2) p2(2)], 'g-', 'LineWidth', 2);
        plot(axLine, p1(1), p1(2), 'bo', 'MarkerSize', 12, 'LineWidth', 3);
        plot(axLine, p2(1), p2(2), 'bo', 'MarkerSize', 12, 'LineWidth', 3);

        text(axLine, p1(1)+10, p1(2), sprintf('P1: [%.1f, %.1f]', P1_mm), 'Color', 'yellow', 'FontWeight', 'bold');
        text(axLine, p2(1)+10, p2(2), sprintf('P2: [%.1f, %.1f]', P2_mm), 'Color', 'yellow', 'FontWeight', 'bold');

        hold(axLine, 'off');
    end

    debug.ok = true;
    debug.reason = '';
end
