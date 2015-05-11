function [Real, Imag, Abs, Theta]=disassemble(complex)

Real = real(complex);  %take half the fft output
Imag = imag(complex);
Abs = sqrt(Real.^2 + Imag.^2);
Theta = atan(Imag./Real);