[d,sr]=wavread('long.wav');
soundsc(d,sr)
e = pvoc(d, 1.5);
%soundsc(e,sr)
f = Rsample(e,3,2); % NB: 0.8 = 4/5
%soundsc(f,sr) 
f = resample(e,3,2); % NB: 0.8 = 4/5
soundsc(f,sr)