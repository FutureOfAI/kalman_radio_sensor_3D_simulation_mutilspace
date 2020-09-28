function [axm,aym,azm]=transform3d_m(x_a,bx,y_a,by,z_a,bz,m,n,sig_bx_0,sig_by_0,sig_bz_0)
sig_bx=sig_bx_0;
sig_by=sig_by_0;
sig_bz=sig_bz_0;
nx=normrnd(0,sig_bx,1,n);
ny=normrnd(0,sig_by,1,n);
nz=normrnd(0,sig_bz,1,n);
axm=zeros(m);
aym=zeros(m);
azm=zeros(m);
for i=1:n
    axm(i)=x_a(i)+bx(i)+nx(i);
    aym(i)=y_a(i)+by(i)+ny(i);
    azm(i)=z_a(i)+bz(i)+nz(i);
end
end