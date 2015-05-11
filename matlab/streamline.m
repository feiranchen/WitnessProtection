[d,sr]=wavread('long.wav');
r=.75;
p=3;
q=4;
n = 256;
win=getW(n);
% With hann windowing on both input and output, 
% we need 25% window overlap for smooth reconstruction
hop = n/4;
% Effect of hanns at both ends is a cumulated cos^2 window (for
% r = 1 anyway);

% Calculate the basic STFT, magnitude scaled
f = n;
w = n;
s = length(d);

% pre-allocate output array
X = zeros((1+f/2),1+fix((s-f)/hop));
c = 1;
for b = 0:hop:(s-f)
  u = win.*d((b+1):(b+f));
  t = fft(u);
  X(:,c) = t(1:(1+f/2))';
  c = c+1;
end;


% Calculate the new timebase samples
[rows, cols] = size(X);
t = 0:r:(cols-2);
% Have to stay two cols off end because (a) counting from zero, and 
% (b) need col n AND col n+1 to interpolate

% Generate the new spectrogram
X2 = pvsample(X, t, hop);

% Invert to a waveform
y = istft(X2, n, n, hop)';

soundsc(y,sr)
f = Rsample(y,p,q); % NB: 0.8 = 4/5
soundsc(f,sr) 
f = resample(y,p,q); % NB: 0.8 = 4/5
soundsc(f,sr)