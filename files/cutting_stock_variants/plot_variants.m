%% Автоматически сгенерировано cutting_stock.c — не редактировать вручную,
%% при необходимости правьте write_octave_script() в исходнике.
close all;
try
  graphics_toolkit('qt');
catch
  try
    graphics_toolkit('gnuplot');
  catch
  end
end

data = dlmread('cutting_stock_variants/data.csv', ',');
id     = data(:,1);
waste  = data(:,2);
unused = data(:,3);
cuts   = data(:,4);
ok     = data(:,5);

fid = fopen('cutting_stock_variants/labels.txt', 'r');
labels = {};
line = fgetl(fid);
while ischar(line)
  labels{end+1} = line;
  line = fgetl(fid);
end
fclose(fid);

mask = ok == 1;
if sum(mask) == 0
  error('Нет ни одного допустимого варианта для отображения');
end
id_ok     = id(mask);
waste_ok  = waste(mask);
unused_ok = unused(mask);
cuts_ok   = cuts(mask);
labels_ok = labels(mask);

%% --- 3D scatter: номер варианта / неиспользованные / остаток ---
figure('visible', 'off');
scatter3(id_ok, unused_ok, waste_ok, 90, cuts_ok, 'filled');
xlabel('Номер варианта');
ylabel('Число неиспользованных заготовок');
zlabel('Суммарный остаток, мм');
title('Варианты раскроя (цвет = суммарное число резов)');
colorbar;
colormap(jet);
grid on;
for i = 1:numel(id_ok)
  text(id_ok(i), unused_ok(i), waste_ok(i), ['  ' labels_ok{i}], 'fontsize', 8);
endfor
print('cutting_stock_variants/variants_3d.png', '-dpng', '-r150');

%% --- 2D scatter: остаток vs неиспользованные (читается точнее) ---
figure('visible', 'off');
scatter(waste_ok, unused_ok, 70, cuts_ok, 'filled');
xlabel('Суммарный остаток, мм');
ylabel('Число неиспользованных заготовок');
title('Компромисс остаток / неиспользованные заготовки (цвет = число резов)');
colorbar;
colormap(jet);
grid on;
for i = 1:numel(id_ok)
  text(waste_ok(i), unused_ok(i), ['  ' labels_ok{i}], 'fontsize', 8);
endfor
print('cutting_stock_variants/variants_2d.png', '-dpng', '-r150');

printf('OK: variants_3d.png и variants_2d.png сохранены в cutting_stock_variants\n');
