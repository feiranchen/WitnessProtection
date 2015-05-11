[d,sr]=wavread('long.wav');
e = pvoc(d, .75);
soundsc(e,sr)
f = Rsample(e,3,4); % NB: 0.8 = 4/5
%soundsc(f,sr) 
f = resample(e,3,4); % NB: 0.8 = 4/5
soundsc(f,sr)