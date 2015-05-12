function f=Rsample(e, p, q)
l= length(e);
f=zeros(l*p/q,1);
ii=1;
for count = (1:l/q-1)
        
    for jj = (1:p)
        f(count*p + jj)=e(ii)+(e(ii+q)-e(ii))*(jj-1)/p;
    end
    ii=ii+q;
end