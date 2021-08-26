function [parmhat, se, parmci, output] = sgtfit(X, alpha, flag)
%SGTFIT Parameter estimates and confidence intervals for skewed generalized
%   t data.
%   PARMHAT = SGTFIT(X) returns maximum likelihood estimates of
%   the three-parameters skewed generalized t (SGT) distribution.
%   PARMHAT(1) is the tail index (shape) parameter ETA, PARMHAT(2) is
%   the scale parameter PSI, and PARMHAT(3) is the skew parameter LAMBDA.
%
%   [PARMHAT,SE] = SGTFIT(X) returns approximate standard errors for the
%   parameter estimates.
%
%   [PARMHAT,SE,PARMCI] = SGTFIT(X) returns 95% confidence intervals for
%   the parameter estimates.
%
%   [PARMHAT,SE,PARMCI,OUTPUT] = SGTFIT(X) further returns a structure
%   OUTPUT containing the fitted skewness and kurtosis, and the maximum
%   log-likelihood.
%
%   [...] = SGTFIT(X,ALPHA) returns 100(1-ALPHA) percent
%   confidence intervals for the parameter estimates.
%
%   Pass in [] for ALPHA to use the default values.
%
%   [...] = SGTFIT(X,ALPHA,FLAG), when FLAG = 1 further
%   estimates the mean M, and the standard deviation SIGMA given the
%   data in X. {Default FLAG = 0}.
%   PARMHAT(4) is the mean M, and PARMHAT(5) is the standard deviation SIGMA.
%
%   SGTFIT needs the Optimization Toolboxes.
%
%   See also GHFIT, SGTCDF, SGTINV, SGTPDF, SGTRND.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Copyright (c) 12 December 2013 by Ahmed BenSaïda           %
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

eta0    = 3;
psi0    = 6;
lambda0 = 0;

parmhat = [eta0, psi0, lambda0];

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

LB = [TOLERANCE, 2 + TOLERANCE, -1 + TOLERANCE,...
    -Inf * flag, TOLERANCE * flag]; % set the lower bounds.

UB = [Inf, Inf, 1 - TOLERANCE, Inf * flag, Inf * flag]; % set the upper bounds.

%
% Maximize the log-likelihood with respect to parmhat. Since FMINCON
% minimizes the function, SGTLIKE gives the nagative of the log-likelihood.
%

if existOptim
    [parmhat,logL,err,out] = fmincon(@sgtlike, parmhat, [], [], [], [], LB, ...
        UB, [], Options, X);
    
    % We can also ensure that the estimated parameters satisfy the
    % theoritical skewness and kurtosis of the SGT. But this method is not
    % recommended, since it does not give statisfying results.

    % [parmhat,logL,err,out] = fmincon(@sgtlike, parmhat, [], [], [], [], LB, ...
    %     UB, @SkewnessKurtosis, Options, X);

else
    % Use a private function FMINSEARCHBND.
    [parmhat,logL,err,out] = fminsearchbnd(@sgtlike, parmhat, LB, UB, ...
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

    % Because ETA must be always > 0, compute the CI for ETA using a
    % normal approximation for log(ETA), and transform back to the
    % original scale. se(log(ETA)) is: se(ETA) / ETA.
    etaCI    = exp(sgtinv(probs, [2, Inf, 0], log(parmhat(1)), ...
                se(1)/parmhat(1)));
    
    % Because PSI must be always > 2, compute the CI for PSI using a
    % normal approximation for log(PSI-2), and transform back to the
    % original scale. se(log(PSI-2)) is: se(PSI) / (PSI - 2),
    % (Delta method).
    psiCI    = exp(sgtinv(probs, [2, Inf, 0], log(parmhat(2)-2), ...
                se(2)/(parmhat(2)-2)))+2;
    
    % Because -1 < lambda < 1, compute the CI for lambda using a
    % normal approximation for atanh(lambda), and transform back to the
    % original scale. se(atanh(lambda)) is: se(lambda) / (1 - lambda^2),
    % (Delta method).
    lambdaCI = tanh(sgtinv(probs, [2, Inf, 0], atanh(parmhat(3)), ...
                se(3)/(1-parmhat(3)^2)));
    
    if flag
        % Compute the CI for M using a normal distribution.
        mCI      = sgtinv(probs, [2, Inf, 0], parmhat(4), se(4));

        % Compute the CI for sigma using a normal approximation for
        % log(sigma), and transform back to the original scale.
        % se(log(sigma)) is: se(sigma) / sigma.
        sigmaCI = exp(sgtinv(probs, [2, Inf, 0], log(parmhat(5)), ...
                    se(5)/parmhat(5)));
               
    else
        mCI     = [];
        sigmaCI = [];
    end
    
    parmci = [etaCI; psiCI; lambdaCI; mCI; sigmaCI];

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

eta    = parms(1);
psi    = parms(2);
lambda = parms(3);

% Avoid constarints violations.
eta(eta <= 0) = TOLERANCE;
psi(psi <= 4) = 4 + TOLERANCE;
lambda(lambda >=  1) =  1 - TOLERANCE;
lambda(lambda <= -1) = -1 + TOLERANCE;

try
    
    if psi <= 200
        % Skewness and kurtosis of the SGT.
        Theta = beta(1/eta,psi/eta) / sqrt((1 + 3 * lambda^2) * ...
                beta(1/eta,psi/eta) * beta(3/eta,(psi-2)/eta) - ...
                4 * lambda^2 * beta(2/eta,(psi-1)/eta)^2);
       
        Skew = 4 * Theta^3 * (lambda + lambda^3) * beta(4/eta,(psi-3)/eta) / ...
            beta(1/eta,psi/eta) - 2 * Theta * lambda * beta(2/eta,(psi-1)/eta) / ...
            beta(1/eta,psi/eta) * (3 + 4 * Theta^2 * lambda^2 * ...
            beta(2/eta,(psi-1)/eta)^2 / beta(1/eta,psi/eta)^2);

        Kurt = Theta^4 * (1 + 5 * lambda^2 * (2 + lambda^2)) * ...
            beta(5/eta,(psi-4)/eta) / beta(1/eta,psi/eta) + 24 * ...
            Theta^2 * lambda^2 * beta(2/eta,(psi-1)/eta)^2 / ...
            beta(1/eta,psi/eta)^2 * (1 + 2 * Theta^2 * lambda^2 * ...
            beta(2/eta,(psi-1)/eta)^2 / beta(1/eta,psi/eta)^2) - ...
            32 * Theta^4 * lambda^2 * (1 + lambda^2) * ...
            beta(2/eta,(psi-1)/eta) * beta(4/eta,(psi-3)/eta) / ...
            beta(1/eta,psi/eta)^2;
    else
        % Skewness and kurtosis of the skewed GED.
        Theta = gamma(1/eta) / sqrt((1 + 3 * lambda^2) * gamma(1/eta) * ...
            gamma(3/eta) - 4 * lambda^2 * gamma(2/eta)^2);
        
        Skew = 2 * Theta^3 * (8 * lambda^3 * gamma(2/eta)^3 / gamma(1/eta)^3 ...
            -3 * lambda * (1 + 3 * lambda^2) * gamma(2/eta) * gamma(3/eta) ...
            / gamma(1/eta)^2 + 2 * lambda * (1 + lambda ^2) * gamma(4/eta) / ...
            gamma(1/eta));

        Kurt = Theta^4 * (-48 * lambda^4 * gamma(2/eta)^4 / gamma(1/eta)^4 + ...
            24 * lambda^2 * (1 + 3 * lambda^2) * gamma(2/eta)^2 * gamma(3/eta) / ...
            gamma(1/eta)^3 - 32 * lambda^2 * (1 + lambda^2) * gamma(2/eta) * ...
            gamma(4/eta) / gamma(1/eta)^2 + (1 + 5 * lambda^2 * (2 + lambda^2)) * ...
            gamma(5/eta) / gamma(1/eta));

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
function LogL = sgtlike(parms, x)
% SGT negative log-likelihood.

global TOLERANCE

eta    = parms(1);
psi    = parms(2);
lambda = parms(3);

% Avoid constarints violations.
eta(eta <= 0) = TOLERANCE;
psi(psi <= 4) = 4 + TOLERANCE;
lambda(lambda >=  1) =  1 - TOLERANCE;
lambda(lambda <= -1) = -1 + TOLERANCE;

if length(parms) > 3
    m      = parms(4);
    sigma  = parms(5);
else
    m     = 0;
    sigma = 1;
end

% Note: Although there is no theoretical upper limit to the degree-of-freedom
%       parameter 'PSI', the GAMMA function rapidly approaches infinity as
%       'PSI' increases. To prevent a NaN (i.e., infinity/infinity) condition,
%       the upper limit of the SGT PSI parameter is arbitrarily set
%       to 200, well beyond the point at which SGT and skewed GED
%       distributions are essentially identical.
%

LogL = zeros(size(x));

if psi <= 200
% 
%  SGT.
%
    Theta = gamma(1/eta) * gamma(psi/eta) / sqrt((1 + 3 * lambda^2) * ...
        gamma(1/eta) * gamma(psi/eta) * gamma(3/eta) * ...
        gamma((psi-2)/eta) - 4 * lambda^2 * gamma(2/eta)^2 * ...
        gamma((psi-1)/eta)^2);
    mu = m - 2 * Theta * sigma * lambda * gamma(2/eta) * gamma((psi-1)/eta) / ...
        gamma(1/eta) / gamma(psi/eta);

    LogL(:)  =  log(2/eta) + log(Theta) + log(sigma) + betaln(1/eta,psi/eta) + ...
        ((psi + 1)/eta) * log(1 + abs(x - mu).^eta ./ (1 + lambda * ...
        sign(x - mu)).^eta / (Theta .* sigma).^eta);
    
else
% 
%  Skewed GED.
%
    Theta = sqrt(gamma(1/eta) / gamma(3/eta)) / sqrt(1 + 3 * lambda^2 - 4 * lambda^2 * ...
        gamma(2/eta)^2 / gamma(1/eta) / gamma(3/eta));
    mu = m - 2 * Theta * sigma * lambda * gamma(2/eta) / gamma(1/eta);

    LogL(:) = log(2/eta) + log(Theta) + log(sigma) + gammaln(1/eta) + ...
        abs(x - mu).^eta ./(1 + lambda * sign(x - mu)).^eta / (Theta.*sigma).^eta;

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

LogLikelihoods = sgtlike(parmhat, y);

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

    LogLikelihoods = sgtlike(pDelta, y);

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
