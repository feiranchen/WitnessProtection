function [Real, Imag, Abs, Theta]=disassemble(complex)

Real = real(complex);  %take half the fft output
Imag = imag(complex);
Abs = sqrt(Real.^2 + Imag.^2);

%Theta = acos(Real./Abs).*sign(Imag);
Theta = acos_table(Real./Abs).*sign(Imag);
