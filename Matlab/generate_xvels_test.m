clear all; clf; 

x=[0; 10; 11];
y=[0; 10; 11];
xpred=linspace(-1, 11, 100)';


% Basic GP
%pl=prior_t();
%pm=prior_sqrtt();
%lengthscale=0.7;
lengthscale=1.5;
cfs=gpcf_sexp('magnSigma2', 17.1, 'lengthScale', lengthscale, ...
              'magnSigma2_prior', prior_fixed(), 'lengthScale_prior', prior_fixed());
%'magnSigma2_prior', pm, 'lengthScale_prior', pl);
gpa = gp_set('cf', {cfs}, 'jitterSigma2', 1e-7);

%% 1) sexp
subplot(3,1,1); hold on;
%gpiaa=gp_ia(gpa, x, y);
gpiaa=gp_optim(gpa, x, y);

[~,Varft]=gp_pred(gpiaa, x, y, xpred);
[Eft,Covft]=gp_jpred(gpiaa, x, y, xpred);

plot(xpred, Eft, 'b')
plot(xpred, Eft + sqrt(Varft), 'g');
plot(xpred, Eft - sqrt(Varft), 'g');
plot(x, y, 'r*');

%% sample
rng default;
for smpl=1:6
    ysmpl = mvnrnd(Eft, Covft, 1)';
    plot(xpred, ysmpl, 'k');
end 

axis manual; axis([0 10 -10.1 10.1]); 
xlabel('X'); ylabel('Y'); title('no monoticity');

%% monotonicity
subplot(3,1,2); hold on;
opt=optimset('TolX',1e-1,'TolFun',1e-1,'Display','iter');
gpam=gpa; %gpam.xv=x(2:2:end);
gpam.xv=linspace(0, 11, 100)';
gpam=gp_monotonic(gpam, x, y, 'nvd', 1, 'optimize', 'on', ...
                  'opt', opt, 'optimf', @fminlbfgs);
%gpiaam=gp_ia(gpam, x, y);
gpiaam=gp_optim(gpam, x, y);

[~,Varftm]=gp_pred(gpiaam, x, y, xpred);
[Eftm,Covftm]=gp_jpred(gpiaam, x, y, xpred);
plot(xpred, Eftm, 'b')
plot(xpred, Eftm + sqrt(Varftm), 'g');
plot(xpred, Eftm - sqrt(Varftm), 'g');
plot(x, y, 'r*')

%% sample
subplot(3,1,3); hold on;
rng default;
for smpl=1:10
    okay=false;
    while ~okay
        ysmpl = mvnrnd(Eftm, Covftm, 1)';    
        ydiff = diff(ysmpl) ./ diff(xpred);
        ydiff = [ydiff; 0.0001];
        yokay = ydiff > 0.0 | xpred < 0.2 | xpred > 9.8;        
        okay = all(yokay);
        %if ~okay
        %    fprintf('rejecting\n');
        %end
    end    
    subplot(3,1,2); plot(xpred, ysmpl, 'k');   
    subplot(3,1,3); plot(xpred, ydiff, 'k');
end

subplot(3,1,2);
axis manual; axis([0 10 -0.1 10.1]); 
xlabel('X'); ylabel('Y'); title('monoticity');

subplot(3,1,3);
axis manual; axis([0 10 -0.5 4.0]); %axis tight;
title('derivative');
