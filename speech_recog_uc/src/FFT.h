/* MIT License

Copyright (c) 2018 Universidade de Coimbra

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef CLASS_FFT_H
#define CLASS_FFT_H
//#endif // HEADER_FLAG

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <memory>



#define PI    (3.141592653589793)
#define TWOPI (6.283185307179586)

typedef   double    Real; //    <<-----------------<<   float OR double

//==================================
struct Complex {
	Real real;
	Real imag;
};
//==================================


//======================
class FFT
//======================
{
private:

    int    *tabBR;  //bit-reversing table for N/2 bits.
    Real   *w;      //exponencial vector, WN(-k), in bit reversed order.
    int     Nfft;   //actual length of the DFT/FFT
	//Private methods:
    void    cfftr2_dit(Real x[], int N);
    void    exps2fft(int N);
    void    tab4bitrev(int N);
	void	init(int N); //creation/changing of exps and tabBR

public:
    FFT()
    {
        tabBR = NULL;
        w     = NULL;
        Nfft  = 0;
    }
   ~FFT()
    {
		if (Nfft != 0 && w != NULL) {
			free(w); free(tabBR);
	   }
    }
	//Public methods:
	void  fft_real(Real x[], int N);	//DFT of a real sequence, x, in-place. Result in x is Complex.
	void  powfft(Real  x[], int N);		//Square Magnitude of Complex X in-place. Result in x is Real.
	void  ifft_cs(Real  x[], int N);	//IDFT of a Complex Symetric sequence x, in-place. Result in x is Real.
	//void  ifft_real(Real x[], int N); //IDFT of the real and even sequence x. Result in x is Real.

	//Auto- and Cross-correlation of real sequences x[n] and y[n] of length M.
	void  xcorr(Real x[], Real y[], int N, int M, int mode=3);
	void  gcc_phat(Real x[], Real y[], int N, int M);

	void shift_pn(Real x[], int N, int M); //shift values: positive then negative indexes.
	void shift_np(Real x[], int N, int M); //shift values: negative then positive indexes.
};


#endif // HEADER_FLAG
