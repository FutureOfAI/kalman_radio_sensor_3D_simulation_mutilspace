function [screwy] = screwt(mu,lambda,sigma,sig_x_r,sig_y_r,n)
    t0 = normrnd(0,sig_x_r,1,n);
    t1 = normrnd(0,sig_y_r,1,n);
    d  = lambda / sqrt(1 + lambda*lambda);
    
    for i=1:n
        screwy(i)  = mu + sigma * (d * abs(t0(i)) + t1(i) * sqrt(1 - d * d));
    end
end