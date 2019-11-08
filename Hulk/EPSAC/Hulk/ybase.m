function [yb] = ybase(ub,ho,n)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

[~,N] = size(ub);

for k=1:N;
    yb(:,k) = real(modelo(ub(:,k),ho)+n);
    ho = yb(:,k);
end

