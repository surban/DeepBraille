clear all; clf;
rng default;

dt=0.1;
duration=25.0;
len=150;
lengthscale=3.75;


gp = [];
for smpl=1:10
    [gp, t_smpl, v_smpl] = generate_tv_curve(gp, dt, duration, len, lengthscale);
    tv = [t_smpl v_smpl];
end

