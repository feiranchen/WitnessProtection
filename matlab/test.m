[d,sr]=wavread('long.wav');
if size(d,2)==1
    d=d'
end
r=1.5;
p=3;
q=2;
n = 256;

s = length(d);
% With hann windowing on both input and output, 
% we need 25% window overlap for smooth reconstruction
hop = n/4;
output_hop = 0;
x = zeros(1,s);
x1 = zeros(1,2*s);
bbb=0;
for bb = 0:n:(s-f)
    u = d((bb+1):(bb+f));             %get the next 1024 frame
    temp = fft(u);
    temp1=zeros(1,n);
    temp3=zeros(1,n);
    for ii=1:n
        temp1(ii*2-1)=temp(ii)*16;
    end
    for ii=1:n/2
        temp3(ii)=temp(2*ii)*32;
    end
    temp2 = [0, 0,0,0,0,0,0,0, temp(1:248)];
    px = ifft(temp);
    px1 = ifft(temp1(1:length(temp)));
    px2 = ifft(temp2);
    px3 = ifft(temp3);
    x((bb+1):(bb+n)) = x((bb+1):(bb+n))+real(px3);
    %x1((bbb+1):(bbb+511)) = x1((bbb+1):(bbb+511))+px1;
    %bbb=bbb+511;
end

y=x';

soundsc(y,sr)