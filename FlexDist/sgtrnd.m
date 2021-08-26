function r = sgtrnd(Param,m,sigma,varargin)
%SGTRND   Random matrices from Skewed Generalized T distribution.
%   R = SGTRND(PARAM,M,SIGMA) returns a matrix of random numbers
%   chosen from the Skewed generalized T distribution with parameters
%   PARAM = [ETA, PSI, LAMBDA], mean M, and with standard deviation
%   SIGMA.
%
%   The size of R is the common size of ETA, PSI, LAMBDA, M, and SIGMA if
%   all are matrices.
%   If all parameter are scalars, the size of R is the size of the other
%   parameter. Alternatively, R = SGTRND(PARAM,M,SIGMA,A,B,...)
%   or R = SGTRND(PARAM,SIGMA,[A,B,...]) returns an A-by-B-by-... array.
%
%   If (ETA = 2) this function becomes equivalent to the Standardized
%   Student's t-distribution with degree of freedom (PSI).
%   If (PSI = infinity) this function becomes equivalent to skewed GED(ETA).
%   When both conditions are met, the SGT becomes the skewed normal.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Copyright (c) 12 December 2013 by Ahmed BenSaïda           %
%                 LaREMFiQ Laboratory, IHEC Sousse - Tunisia             %
%                       Email: ahmedbensaida@yahoo.com                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin < 1
    error('Requires at least one input argument.');
end

if numel(Param) ~= 3 || size(Param,2) ~= 3
    error('Input argument ''Param'' must be a 3-columns parameter.')
end

if nargin < 2 || isempty(m)
    m = 0;
end

if nargin < 3 || isempty(sigma)
    sigma = 1;
end

% Extract distribution parameters from PARAM.
eta    = Param(:,1);
psi    = Param(:,2);
lambda = Param(:,3);

[err, sizeOut] = sizechk(5,eta,psi,lambda,m,sigma,varargin{:});
if err > 0
    error('Size information is inconsistent.');
end

%Initialize R to zero.
r     = zeros(sizeOut);

if numel(eta) == 1
    eta = eta(ones(sizeOut));
end
if numel(psi) == 1
    psi = psi(ones(sizeOut));
end
if numel(lambda) == 1
    lambda = lambda(ones(sizeOut));
end
if numel(m) == 1
    m = m(ones(sizeOut));
end
if numel(sigma) == 1
    sigma = sigma(ones(sizeOut));
end

% Return NaN for invalid parameters.
r(sigma <= 0 | lambda <= -1 | lambda >= 1 | eta <= 0 | psi <= 2 | ...
    isnan(m) | isnan(sigma) | isnan(lambda) | isnan(eta) | isnan(psi)) = NaN;

% Express SGT random numbers as a function of inverse
% cumulative distribution function.
u = rand(sizeOut);
k   =   find(sigma > 0 & lambda > -1 & lambda < 1 & eta > 0 & psi > 2);
if any(k)    
    r(k) = sgtinv(u(k),[eta(k),psi(k),lambda(k)],m(k),sigma(k));
end

%-------------------------------------------------------------------------%
%                           Helper function                               %
%-------------------------------------------------------------------------%

function [err, commonSize, numElements] = sizechk(nparams,varargin)
%SIZECHK Check for compatible array sizes.
%   [ERR,COMMONSIZE,NUMELEMENTS] = SIZECHK(NPARAMS,A,B,...,M,N,...) or
%   [ERR,COMMONSIZE,NUMELEMENTS] = SIZECHK(NPARAMS,A,B,...,[M,N,...])
%   in effect computes size( A + B + ... + zeros(M,N,...) ), and catches
%   any size mismatches.  NPARAMS is the number of array input arguments.

try
    tmp = 0;
    for argnum = 1:nparams
        tmp = tmp + varargin{argnum};
    end
    if nargin > nparams+1
        tmp = tmp + zeros(varargin{nparams+1:end});
    end
    err = 0;
    commonSize = size(tmp);
    numElements = numel(tmp);

catch
    err = 1;
    commonSize = [];
    numElements = 0;
end