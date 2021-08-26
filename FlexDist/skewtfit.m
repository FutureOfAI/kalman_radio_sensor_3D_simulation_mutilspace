function [parmhat, se, parmci, output] = skewtfit(X, alpha, flag)
%SKEWTFIT Parameter estimates and confidence intervals for skewed t data.
%   PARMHAT = SKEWTFIT(X) returns maximum likelihood estimates of
%   the two-parameters skewed t (SKEWT) distribution.
%   PARMHAT(1) is the shape parameter NU, PARMHAT(2) is
%   is the skew parameter LAMBDA.
%
%   [PARMHAT,SE] = SKEWTFIT(X) returns approximate standard errors for the
%   parameter estimates.
%
%   [PARMHAT,SE,PARMCI] = SKEWTFIT(X) returns 95% confidence intervals for
%   the parameter estimates.
%
%   [PARMHAT,SE,PARMCI,OUTPUT] = SKEWTFIT(X) further returns a structure
%   OUTPUT containing the fitted skewness and kurtosis, and the maximum
%   log-likelihood.
%
%   [...] = SKEWTFIT(X,ALPHA) returns 100(1-ALPHA) percent
%   confidence intervals for the parameter estimates.
%
%   Pass in [] for ALPHA to use the default values.
%
%   [...] = SKEWTFIT(X,ALPHA,FLAG), when FLAG = 1 further
%   estimates the mean M, and the standard deviation SIGMA given the
%   data in X. {Default FLAG = 0}.
%   PARMHAT(3) is the mean M, and PARMHAT(4) is the standard deviation SIGMA.
%
%   SKEWTFIT needs the Optimization Toolboxes.
%
%   See also SKEWTCDF, SKEWTINV, SKEWTPDF, SKEWTRND.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Copyright (c) 14 March 2015 by Ahmed Ben Saïda             %
%                 LaREMFiQ Laboratory, IHEC Sousse - Tunisia             %
%                       Email: ahmedbensaida@yahoo.com                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~isvector(X)
    error(message('Input variable ''X'' must be a vector.'));
end

if nargin < 2 || isempty(alpha)
    alpha = 0.05;
end

if nargin < 3 || isempty(flag) || (flag == 0)
    flag = []; % Ensure flag exists.
else
    % Any number different from zero will give 1.
    flag = logical(flag);
end

% Check if the Optimization toolbox is installed. 1 if installed, 0 if not.
existOptim = license('test','optim_toolbox');

% The default options include turning fmincon's display off.  This
% function gives its own warning/error messages, and the caller can turn
% display on to get the text output from fmincon if desired.

if existOptim
    Options = optimset('fmincon');
else
    Options = optimset('fminsearch');
end

Options = optimset(Options,...
                        'Display'    , 'off',...
                        'MaxFunEvals', 1000 ,...
                        'MaxIter'    , 800  ,...
                        'Diagnostics', 'off',...
                        'Algorithm'  , 'active-set');

if isa(X,'single')
    X = double(X);
end

% Initial guess of M and SIGMA.
m0     = mean(X);
sigma0 = std(X,0);

%
% Use fsolve to solve skewness and kurtosis for the initial guess of
% eta, psi and lambda. Solve for the stansardized X since it gives more
% consistent results.
%

nu0     = 8;
lambda0 = 0;

parmhat = [nu0, lambda0];

if existOptim
    Opt = optimoptions('fsolve','Display','off','Algorithm',...
    'levenberg-marquardt','ScaleProblem','jacobian');

    parmhat = fsolve(@SkewnessKurtosis, parmhat, Opt, (X - m0)/sigma0);
end

if flag
    % Further estimate the mean and statandard deviantion.
    parmhat = [parmhat, m0, sigma0];
else
    % Standardize the data.
    X = (X - m0) / sigma0;
end

% Specify a tolerance level for linear constraints.
global TOLERANCE
TOLERANCE = 2 * optimget(Options,'TolCon',1e-7);

LB = [2 + TOLERANCE, -1 + TOLERANCE, -Inf * flag, TOLERANCE * flag]; % set the lower bounds.

UB = [Inf, 1 - TOLERANCE, Inf * flag, Inf * flag]; % set the upper bounds.

%
% Maximize the log-likelihood with respect to parmhat. Since FMINCON
% minimizes the function, SGTLIKE gives the nagative of the log-likelihood.
%

if existOptim
    [parmhat,logL,err,out] = fmincon(@skewtlike, parmhat, [], [], [], [], LB, ...
        UB, [], Options, X);
    
    % We can also ensure that the estimated parameters satisfy the
    % theoritical skewness and kurtosis of the SGT. But this method is not
    % recommended, since it does not give statisfying results.

    % [parmhat,logL,err,out] = fmincon(@skewtlike, parmhat, [], [], [], [], LB, ...
    %     UB, @SkewnessKurtosis, Options, X);

else
    % Use a private function FMINSEARCHBND.
    [parmhat,logL,err,out] = fminsearchbnd(@skewtlike, parmhat, LB, UB, ...
        Options, X);
end

% Convert minimum negative loglikelihood to maximum likelihood:
logL = -logL;

% Computes the estimated Skewness and Kurtosis.
SkewKurt = SkewnessKurtosis(parmhat, X)' + [skewness(X, 0), kurtosis(X, 0)];

% Construct the OUTPUT structure.
output.LogLikelihood = logL;
output.Skewness      = SkewKurt(1);
output.Kurtosis      = SkewKurt(2);

if (err == 0)
    % fmincon may print its own output text; in any case give something
    % more statistical here, controllable via warning IDs.
    if out.funcCount >= Options.MaxFunEvals
        warning('Number of function evaluation has exceeded the limit.');
    else
        warning('Number of iterations has exceeded the limit.');
    end
elseif (err < 0)
    error('No solution found.');
end

if nargout > 1
    probs = [alpha/2, 1-alpha/2];
    acov  = varcov(parmhat, X);
    se    = sqrt(diag(acov))';

    % Because NU must be always > 2, compute the CI for NU using a
    % normal approximation for log(NU-2), and transform back to the
    % original scale. se(log(NU-2)) is: se(NU) / (NU - 2),
    % (Delta method).
    nuCI    = exp(norminv(probs, log(parmhat(1)-2), se(1)/(parmhat(1)-2)))+2;
    
    % Because -1 < lambda < 1, compute the CI for lambda using a
    % normal approximation for atanh(lambda), and transform back to the
    % original scale. se(atanh(lambda)) is: se(lambda) / (1 - lambda^2),
    % (Delta method).
    lambdaCI = tanh(norminv(probs, atanh(parmhat(2)), ...
                se(2)/(1-parmhat(2)^2)));
    
    if flag
        % Compute the CI for M using a normal distribution.
        mCI      = norminv(probs, parmhat(3), se(3));

        % Compute the CI for sigma using a normal approximation for
        % log(sigma), and transform back to the original scale.
        % se(log(sigma)) is: se(sigma) / sigma.
        sigmaCI = exp(norminv(probs, log(parmhat(4)), ...
                    se(4)/parmhat(4)));
               
    else
        mCI     = [];
        sigmaCI = [];
    end
    
    parmci = [nuCI; lambdaCI; mCI; sigmaCI];

end

if isa(X,'single')
    parmhat = single(parmhat);
    if nargout > 1
        parmci = single(parmci);
    end
end

%-------------------------------------------------------------------------%
%                           Helper functions                              %
%-------------------------------------------------------------------------%

function [C, Ceq] = SkewnessKurtosis(parms, x)
% Skewness and Kurtosis of the SGT.

global TOLERANCE

nu     = parms(1);
lambda = parms(2);

% Avoid constarints violations.
nu(nu <= 2) = 2 + TOLERANCE;
lambda(lambda >=  1) =  1 - TOLERANCE;
lambda(lambda <= -1) = -1 + TOLERANCE;

try
    
    if nu <= 200
        % Skewness and kurtosis of the SGT.
        Theta = beta(1/2,nu/2) / sqrt((1 + 3 * lambda^2) * ...
                beta(1/2,nu/2) * beta(3/2,nu/2-1) - 16 * ...
                lambda^2 / (nu-1)^2);
    
        Skew = 4 * Theta^3 * lambda * (1 + lambda^2) * beta(2,(nu-3)/2) / ...
            beta(1/2,nu/2) - 4 * Theta * lambda / ...
            ((nu-1) * beta(1/2,nu/2)) * (3 + 16 * Theta^2 * lambda^2 / ...
            ((nu-1)^2 * beta(1/2,nu/2)^2));

        Kurt = 3 * Theta^4 * (1 + 5 * lambda^2 * (2 + lambda^2)) / ...
            ((nu-4) * (nu-2)) + 16 * Theta^2 * lambda^2 / ...
            ((nu-1)^2 * beta(1/2,nu/2)^2) * (6 - 16 * Theta^2 * ...
            (1+lambda^2) / (nu-3) + 12 * Theta^2 * lambda^2 * ...
            gamma((nu-1)/2)^2 / (pi * gamma(nu/2)^2));
    else
        % Skewness and kurtosis of the skewed normal.
        Theta = 1 / sqrt(0.5 + (3/2 - 4/pi) * lambda^2);
        
        Skew = Theta^3 * lambda * (pi + 16 * lambda^2 - 5 * pi * ...
            lambda^2) / pi^(3/2);

        Kurt = Theta^4 * (3*pi^2 + 10*pi*(3*pi-8)*lambda^2 + ...
            lambda^4*(15*pi^2+16*pi-192)) / (4*pi^2);

    end
    
catch
    % For some parameters, the theoritical skewness and kurtosis cannot be
    % computed, so give a reasonable answer.
    
    Skew = skewness(x, 0);
    Kurt = kurtosis(x, 0);
    
end

% Nonlinear equality constraint.
Ceq = [Skew - skewness(x, 0); Kurt - kurtosis(x, 0)];

if nargout >= 2
    % Use with FMINCON.
    C = [0; 0]; % Nonlinear inequality constraint.
else
    % Use with FSOLVE for initial parameters guess.
    C   = Ceq;
    Ceq = [];
end

%-------------------------------------------------------------------------%
function LogL = skewtlike(parms, x)
% SKEWT negative log-likelihood.

global TOLERANCE

nu     = parms(1);
lambda = parms(2);

% Avoid constarints violations.
nu(nu <= 2) = 2 + TOLERANCE;
lambda(lambda >=  1) =  1 - TOLERANCE;
lambda(lambda <= -1) = -1 + TOLERANCE;

if length(parms) > 2
    m      = parms(3);
    sigma  = parms(4);
else
    m     = 0;
    sigma = 1;
end

% Note: Although there is no theoretical upper limit to the degree-of-freedom
%       parameter 'NU', the GAMMA function rapidly approaches infinity as
%       'NU' increases. To prevent a NaN (i.e., infinity/infinity) condition,
%       the upper limit of the NU parameter is arbitrarily set
%       to 200, well beyond the point at which SKEWT and skewed normal
%       distributions are essentially identical.
%

LogL = zeros(size(x));

if nu <= 200
% 
%  SKEWT.
%
    Theta = beta(1/2,nu/2) / sqrt((1 + 3 * lambda^2) * ...
            beta(1/2,nu/2) * beta(3/2,nu/2-1) - 16 * ...
            lambda^2 / (nu-1)^2);
    mu    = m - 4 * Theta * lambda * sigma / ((nu - 1) * beta(1/2,nu/2));
    
    LogL(:)  =  log(Theta) + log(sigma) + betaln(1/2,nu/2) + ...
        ((nu + 1)/2) * log(1 + (x - mu).^2 ./ (1 + lambda * ...
        sign(x - mu)).^2 / (Theta * sigma)^2);
    
else
% 
%  Skewed normal.
%
    Theta = 1 / sqrt(0.5 + (3/2 - 4/pi) * lambda^2);
    mu = m - 2 * Theta * sigma * lambda / sqrt(pi);

    LogL(:) = log(Theta) + log(sigma) + 0.5 * log(pi) + ...
        (x - mu).^2 ./(1 + lambda * sign(x - mu)).^2 / (Theta * sigma)^2;

end

% The log-likelihood obtained is the negative of the real LogL.
LogL = sum(LogL);

%
% Catch conditions that produce anomalous log-likelihood function values.
% Typically, what happens is that input parameter values will result in
% an unstable inverse filter, which produces LogL = inf. This, in turn, will
% not allow FMINCON to update the iteration. By setting the LogL to a large, 
% but finite, value, we can safeguard against anomalies and allows the 
% iteration to continue.
%

LogL(~isfinite(LogL))  =  1.0e+20;
LogL(~(~imag(LogL)))   =  1.0e+20;

%-------------------------------------------------------------------------
function covarianceMatrix = varcov(parmhat, y)                               
%VARCOV Error covariance matrix of maximum likelihood parameter estimates

delta = 1e-10;  % Offset for numerical differentiation

% Evaluate the loglikelihood objective function at the MLE parameter
% estimates. In contrast to the optimization, which is interested in the
% single scalar objective function value logL, here we are interested in
% the loglikelihoods for each observation of y(t), which sum to -logL.

LogLikelihoods = skewtlike(parmhat, y);

g0 = -LogLikelihoods;

% Initialize the perturbed parameter vector and the scores matrix. For T
% observations in y(t) and K parameters estimated via maximum likelihood,
% the scores array is a T-by-K matrix.

pDelta = parmhat;
scores = zeros(length(y),numel(parmhat));

for j = 1:numel(parmhat)

    pDelta(j) = parmhat(j)*(1+delta);
    dp = delta*parmhat(j);

    % Trap the case of a zero parameter value, p0(j) = 0:

    if dp == 0
        dp = delta;
        pDelta(j) = dp;
    end

    LogLikelihoods = skewtlike(pDelta, y);

    gDelta = -LogLikelihoods;

    scores(:,j) = (g0-gDelta)/dp;
    pDelta(j) = parmhat(j);

end

% Invert the outer product of the scores matrix to get the approximate
% covariance matrix of the MLE parameters.

try

  % Pre-allocate the output covariance matrix to the full size.
   covarianceMatrix = zeros(numel(parmhat));
   
   j = 1:numel(parmhat);
   covarianceMatrix(j,j) = pinv(scores(:,j)'*scores(:,j));
   
catch

  % If an error occurs in the calculation of the covariance matrix, then
  % assign a matrix of all NaNs to indicate the error condition:

   covarianceMatrix = NaN(numel(parmhat),numel(parmhat));

end

covarianceMatrix = (covarianceMatrix + covarianceMatrix')/2;
