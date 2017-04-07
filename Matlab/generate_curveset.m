clear all; clf;
rng default;

base_dir='..\\Data\\DeepBraille\\XVels\\Set1';
dt=0.05;
duration=25.0;
len=141;
lengthscale=3.75;
n_samples=5;

gp=[];
hfig=figure(1);
for page=0:9
   for curve=0:6
       dirname=sprintf('%s\\%02d.cur\\curve%d\\', base_dir, page, curve);
       mkdir(dirname);
       clf;
       for smpl=0:n_samples-1
           %fprintf('Page=%d  Curve=%d  Smpl=%d\n', page, curve, smpl);
           filename=sprintf('%s\\Vel%02d.h5', dirname, smpl);
           [gp, t_smpl, v_smpl] = generate_tv_curve(gp, dt, duration, len, lengthscale);          
           tv = [t_smpl v_smpl]';
           fprintf('%s\n', filename);
           if exist(filename, 'file'), delete(filename); end
           h5create(filename, '/vels', size(tv));
           h5write(filename, '/vels', tv);
       end
       plotname=sprintf('%s\\XVels.pdf', dirname);
       print(hfig, plotname, '-dpdf');
   end    
end

