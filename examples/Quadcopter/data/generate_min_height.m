% Generate the minimum altitude profile, simulating some mountains

clc; clear
%close all;

%%
x_range = -2e2:2e2:1e4;
y_range = -2e2:2e2:1e4;
[X,Y] = meshgrid(x_range, y_range);

%
close all;
mount = { [4e3,4e3,200, 1e3, 1.5], ...
          [4e3,1e3,600, 6e2, 2], ...
          [6e3,8e3,800, 7e2, 3], ...
          [9e3,4e3,700, 9e2, 2], ...
          [3e3,6e3,400, 9e2, 2], ...
          [2.6e3,2e3,500, 4e2, 2], ...
          };

%
Z = X.*0;
for i=1:numel(mount)
    tmp = mount{i};
    x0 = tmp(1);
    y0 = tmp(2);
    z0 = tmp(3);
    sigma = tmp(4);
    factor = tmp(5);
    
    distance_sqr = ((X-x0).^2 + (Y-y0).^2);
    distance = sqrt(distance_sqr);
    
    %Z = Z + z0*exp(-0.5*distance_sqr./sigma^2); 
    Z = Z + z0*(1+sin(-atan(distance./sigma-1*factor)))/2;
end

% translate to zer
index = find(X == 0 &  Y == 0);
Z = Z - Z(index);

figure(1); clf;
surf(X,Y,Z); hold on;
%axis equal;
xlim([0,1e4]);
ylim([0,1e4]);
zlim([0,1e3]);
xlabel('x'); ylabel('y')

return
%% write to file
filenames = {'ground_altitude_profile_x.txt',...
             'ground_altitude_profile_y.txt',...
             'ground_altitude_profile_z.txt' };
datanames = {'x','y','z' };
datas = {unique(X),...
         unique(Y),...
         Z };
save('ground_altitude_profile.mat', 'X','Y','Z');
for j=1:numel(filenames)
    fid = fopen(filenames{j}, 'w');
    fprintf('Writing file...%s', filenames{j});
    fprintf(fid, datanames{j});
    fprintf(fid, '\n');
    data = datas{j};
    for i=1:numel(data)
       fprintf(fid, '%.2f\n', data(i)); 
    end
    fclose(fid);
    fprintf('saved\n');
end;
fprintf('ALL DONE\n');

