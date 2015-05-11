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








% if r<=1
%     count =1;
%     for ii = (1:l)
%         if count == 1/r
%             f(ii*r)=e(ii);
%             count =1;
%         else
%             count = count+1;
%         end
%     end
% else
%     for ii = (1:l)
%         if ii==l
%             f((ii-1)*r + jj)=e(ii);
%         else
%             for jj = (1:r)
%                 f((ii-1)*r + jj)=e(ii)+(e(ii+1)-e(ii))*(jj-1)/r;
%             end
%         end
%     end
% end