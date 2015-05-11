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
% Append a 'safety' column on to the end of b to avoid problems 
% taking *exactly* the last frame (i.e. 1*b(:,cols)+0*b(:,cols+1))
%b = zeros((1+f/2),2+fix((s-f)/hop));
%c = zeros((1+f/2), length(t));
c = 1;
t = 0;
index = 1;
b1 = zeros((1+f/2), 1);
i1 = 0;
ready1 = 0;
b2 = zeros((1+f/2), 1);
i2 = 0;
ready2 = 0;
result = zeros((1+f/2),1)
ph = 0;
while bb <= (s-f)
  % Grab the two columns of b
  ready1 = 1;
  if i1 == floor(t) + 1
      %do nothing
  elseif i2 == floor(t) + 1
      b1 = b2;
      i1 = i2;
  elseif index == floor(t) + 1
  	u = win.*d((bb+1):(bb+f));
  	temp = fft(u);
    b1 = temp(1:(1+f/2))';
    i1 = floor(t) + 1;
  else
      ready1 = 0;
  end
  
  ready2 = 1;
  if i2 == floor(t) + 2
      %do nothing
  elseif index == floor(t) + 2
  	u = win.*d((bb+1):(bb+f));
  	temp = fft(u);
    b2 = temp(1:(1+f/2))';
    i2 = floor(t) + 2;
  else
      ready2 = 0;
  end
  
  index = index + 1;
  bb = bb + hop;
  

  % Phase accumulator
    % Preset to phase of first frame for perfect reconstruction
    % in case of 1:1 time scaling
  if (i1 == 1)  
    ph = angle(b1);
  end
  if (ready1 && ready2)
      tf = t - floor(t);
      bmag = (1-tf)*abs(b1) + tf*(abs(b2));
      % calculate phase advance
      dp = angle(b2) - angle(b1);% - dphi';
      % Reduce to -pi:pi range
      dp = dp - 2 * pi * round(dp/(2*pi));
      % Save the column
      result = bmag .* exp(j*ph);
      % Cumulate phase, ready for next frame
      ph = ph + dp;% + dphi'

      t = t+r;
 
      ft = result';
      ft = [ft, conj(ft([((n/2)):-1:2]))];
      px = real(ifft(ft));
      x((bb+1):(bb+n)) = x((bb+1):(bb+n))+px.*win;
  end
end;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%below is not done%%%%%%%%%%%%%%%%%%%%
% Invert to a waveform
%y = istft(c, n, n, hop)';
s = size(c);
cols = length(t);
xlen = n + (cols-1)*hop;
x = zeros(1,xlen);

for bb = 0:hop:(hop*(cols-1))
  ft = c(:,1+bb/hop)';
  ft = [ft, conj(ft([((n/2)):-1:2]))];
  px = real(ifft(ft));
  x((bb+1):(bb+n)) = x((bb+1):(bb+n))+px.*win;
end;
y=x';

% Calculate the new timebase samples
% Have to stay two cols off end because (a) counting from zero, and 
% (b) need col n AND col n+1 to interpolate

% Generate the new spectrogram
% Empty output array
%c = zeros((1+f/2), length(t));

% Expected phase advance in each bin
%dphi = zeros(1,f/2+1);
%dphi(2:(1 + f/2)) = (2*pi*hop)./(f./(1:(f/2)));

% Phase accumulator
% Preset to phase of first frame for perfect reconstruction
% in case of 1:1 time scaling
ph = angle(b(:,1));


ocol = 1;
for tt = t
  % Grab the two columns of b
  bcols = b(:,floor(tt)+[1 2]);
  tf = tt - floor(tt);
  bmag = (1-tf)*abs(bcols(:,1)) + tf*(abs(bcols(:,2)));
  % calculate phase advance
  dp = angle(bcols(:,2)) - angle(bcols(:,1));% - dphi';
  % Reduce to -pi:pi range
  dp = dp - 2 * pi * round(dp/(2*pi));
  % Save the column
  c(:,ocol) = bmag .* exp(j*ph);
  % Cumulate phase, ready for next frame
  ph = ph + dp;% + dphi'
  ocol = ocol+1;
end

% Invert to a waveform
%y = istft(c, n, n, hop)';
s = size(c);
cols = length(t);
xlen = n + (cols-1)*hop;
x = zeros(1,xlen);

for bb = 0:hop:(hop*(cols-1))
  ft = c(:,1+bb/hop)';
  ft = [ft, conj(ft([((n/2)):-1:2]))];
  px = real(ifft(ft));
  x((bb+1):(bb+n)) = x((bb+1):(bb+n))+px.*win;
end;
y=x';


%soundsc(y,sr)
f = Rsample(y,p,q); % NB: 0.8 = 4/5
soundsc(f,sr) 
%f = resample(y,p,q); % NB: 0.8 = 4/5
%soundsc(f,sr)