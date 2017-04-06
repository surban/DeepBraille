function [gp, t_pred, v_smpl] = generate_tv_curve(gp, dt, duration, len, lengthscale)

n_deriv_points = 100;

%% constraints
baselen = 10.0;
t_trn=[0; duration; duration*1.1];
x_trn=[0; baselen;  baselen*1.1];

%% GP
if isempty(gp)
    opt=optimset('TolX', 1e-1, 'TolFun', 1e-1, 'Display', 'iter');
    cfs=gpcf_sexp('magnSigma2', 17.1, 'lengthScale', lengthscale, ...
                  'magnSigma2_prior', prior_fixed(), 'lengthScale_prior', prior_fixed());
    gpa = gp_set('cf', {cfs}, 'jitterSigma2', 1e-9);
    gpa.xv=linspace(0, duration, n_deriv_points)';
    gpam=gp_monotonic(gpa, t_trn, x_trn, 'nvd', 1, 'optimize', 'on', ...
                      'opt', opt, 'optimf', @fminlbfgs);
    gp=gp_optim(gpam, t_trn, x_trn);
end

%% predict mean and covariance
t_pred=(0:dt:duration)';
[~,sigmasq]=gp_pred(gp, t_trn, x_trn, t_pred);
[mu,Sigma]=gp_jpred(gp, t_trn, x_trn, t_pred);

%% sample curve
curve_okay=false;
while ~curve_okay
    x_smpl = mvnrnd(mu, Sigma, 1)';    
    x_smpl = x_smpl .* (len / baselen);
    v_smpl = diff(x_smpl) ./ diff(t_pred);
    v_smpl = [v_smpl; 0.0001];
    v_okay = v_smpl > 0.0 | t_pred < 0.1 | t_pred > duration*0.99;        
    curve_okay = all(v_okay);
    %if ~curve_okay, fprintf('rejecting\n'); end
end    

%% scale mu, sigma for plotting
mu = mu .* (len / baselen);
sigmasq = sigmasq .* (len / baselen)^2;

%% plot      
subplot(2,1,1); hold on;
plot(t_pred, x_smpl, 'k');   
plot(t_pred, mu, 'b')
plot(t_pred, mu + sqrt(sigmasq), 'g');
plot(t_pred, mu - sqrt(sigmasq), 'g');
axis manual; axis([0 duration 0 len]); 
xlabel('t'); ylabel('x'); title('position vs time');

subplot(2,1,2); hold on;
v_mu = [diff(mu) ./ diff(t_pred); 0.0001];
v_mu_plus = [diff(mu + sqrt(sigmasq)) ./ diff(t_pred); 0.0001];
v_mu_minus = [diff(mu - sqrt(sigmasq)) ./ diff(t_pred); 0.0001];
plot(t_pred, v_mu, 'b');
plot(t_pred, v_mu_plus, 'g');
plot(t_pred, v_mu_minus, 'g');
plot(t_pred, v_smpl, 'k');
axis manual; axis([0 duration -0.1 max(v_smpl)*1.1]); 
xlabel('t'); ylabel('v'); title('velocity vs time');

