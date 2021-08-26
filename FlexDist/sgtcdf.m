function p = sgtcdf(x,Param,m,sigma)
%SGTCDF Skewed Generalized T cumulative density function (cdf).
%   P = SGTCDF(X,PARAM,M,SIGMA) Returns the Skewed GT cdf
%   with parameters: PARAM = [ETA, PSI, LAMBDA], mean M, and with
%   standard deviation SIGMA, at the values in X.
%
%   The size of P is the common size of the input arguments. A scalar input  
%   functions as a constant matrix of the same size as the other inputs.     
%
%   Default values for PARAM, M and SIGMA are [2, Infinity, 0], 0,
%   and 1 respectively.
%   If (ETA = 2) this function becomes equivalent to the Standardized
%   Student's t-distribution with degree of freedom PSI.
%   If (PSI = Infinity) this function becomes equivalent to skewed GED(ETA).
%   When both conditions are met, the SGT becomes the skewed normal.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Copyright (c) 12 December 2013 by Ahmed BenSaïda           %
%                 LaREMFiQ Laboratory, IHEC Sousse - Tunisia             %
%                       Email: ahmedbensaida@yahoo.com                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin < 1, 
    error('Requires at least one input argument.');
end

if nargin < 2 || isempty(Param)
    Param = [2, Inf, 0];
end

if size(Param,2) ~= 3
    error('Input argument ''Param'' must be a 3-columns parameter.')
end

if nargin < 3 || isempty(m);
    m = 0;
end

if nargin < 4 || isempty(sigma),
    sigma = 1;
end

% Extract distribution parameters from PARAM.
eta    = Param(:,1);
psi    = Param(:,2);
lambda = Param(:,3);

[errorcode, x, eta, psi, lambda, m, sigma] = paramchck(6,x,eta,psi,lambda,m,sigma);

if errorcode > 0
    error('Requires non-scalar arguments to match in size.');
end

%   Initialize P to zero.
if isa(x,'single') || isa(eta,'single') || isa(psi,'single') || ...
        isa(lambda,'single') || isa(m,'single') || isa(sigma,'single')
    p = zeros(size(x),'single');
else
    p = zeros(size(x));
end

%   Set the maximun PSI after which the Skewed GTD becomes equivalent to GED.
PSImax = 200;

% Return NaN for invalid parameters.
p(sigma <= 0 | lambda <= -1 | lambda >= 1 | eta <= 0 | psi <= 2 | ...
    isnan(x) | isnan(sigma) | isnan(lambda) | isnan(eta) | isnan(psi)) = NaN;

h   =   find(sigma > 0 & lambda > -1 & lambda < 1 & eta > 0 & ...
    psi > 2 & psi <= PSImax & ~isnan(x));
if any(h)
    theta = beta(1./eta(h),psi(h)./eta(h)) ./ sqrt((1+3*lambda(h).^2).*...
        beta(1./eta(h),psi(h)./eta(h)).*beta(3./eta(h),(psi(h)-2)./eta(h)) - ...
        4*lambda(h).^2 .*beta(2./eta(h),(psi(h)-1)./eta(h)).^2);
    mu = m(h) - 2*theta.*lambda(h).*sigma(h).*beta(2./eta(h),(psi(h)-1)./eta(h)) ./ ...
        beta(1./eta(h),psi(h)./eta(h));
    
    xx = abs(x(h)-mu).^eta(h) ./ ((theta.*sigma(h).*(1 + sign(x(h)-mu).* ...
        lambda(h))).^eta(h) + abs(x(h)-mu).^eta(h));
    p(h) = 0.5*(1-lambda(h)) + 0.5 * (lambda(h) + sign(x(h)-mu)) .* ...
        betainc(xx , 1./eta(h) , psi(h)./eta(h));
end

% When PSI > PSImax the skewed GTD becomes the skewed GED.
h   =   find(sigma > 0 & lambda > -1 & lambda < 1 & eta > 0 & ...
    psi > 2 & psi > PSImax);
if any(h)
    theta = gamma(1./eta(h)) ./ sqrt((1+3*lambda(h).^2).*...
        gamma(1./eta(h)).*gamma(3./eta(h)) - 4*lambda(h).^2 .*...
        gamma(2./eta(h)).^2);
    mu = m(h) - 2*theta.*lambda(h).*sigma(h).*gamma(2./eta(h)) ./ ...
        gamma(1./eta(h));
    xx = abs(x(h)-mu).^eta(h) ./ (theta.*sigma(h).*(1 + sign(x(h)-mu).* ...
        lambda(h))).^eta(h);
    
%      p(h) = 0.5*(1+sign(x(h)-m)) - 0.5 * (lambda(h) + sign(x(h)-m)) .* ...
%          gammainc(xx , 1./nu(h), 'upper');
      p(h) = 0.5*(1-lambda(h)) + 0.5 * (lambda(h) + sign(x(h)-mu)) .* ...
         gammainc(xx , 1./eta(h), 'lower');
    
end