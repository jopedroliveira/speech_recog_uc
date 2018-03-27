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

/*
FFT.cpp  :  DFT routines of real signals, DFT magnitude and correlations.
*/

#include "FFT.h"
#include <ros/ros.h>
//#include <memory.h>

//===============================================
void FFT::cfftr2_dit(Real x[], int N)
//===============================================
//---- Basic DFT routine of a complex signal ----
//Texas Instruments FFT routine (with ASM version)
//w: tweedles WN(-k) in bit reversed order (member; previous computation).
//   (w is the same for DFTs or lower orders)
//x: complex signal, normal order. Output in bit-reversed order.
//N: DFT length: must be a power of 2.
//Ref: figure 6.12 in Oppenheim, 1975, page 300.
{

int   N2, ie, ia, i, j, k, p,q;
Real  rtemp, itemp, c, s;
		
    N2 = N;
    ie = 1;
    for(k=N; k > 1; k >>= 1)
     {
       N2 >>= 1;
       ia = 0;
       for(j=0; j < ie; j++)
        { p= j<<1;
          c = w[p]; s = w[p+1];
          for(i=0; i < N2; i++)
           {
             q = (ia+N2)<<1; p=ia<<1;
             rtemp  = c * x[q]   + s * x[q+1];
             itemp  = c * x[q+1] - s * x[q];
             x[q]   = x[p] - rtemp; x[q+1] = x[p+1] - itemp;
             x[p]   = x[p] + rtemp; x[p+1] = x[p+1] + itemp;
             ia++;
           }
          ia += N2;
        }
       ie <<= 1;
      }
 } //cfftr2_dit
//-----------------------------------------------------------------------------------------

//==============================================
void FFT::exps2fft(int N)
//==============================================
//Computs N/2 exponentials, WN(-k), for DFT/FFT of length N.
//WN(-k) = exp(+j*2*pi*k/N) = c +j*s, k=0...N/2-1.
//Memory for w (member) already allocated: N Real elements.
//Computs vector w in bit-reversed order; 
//Uses tabBR, the bit-reverse table of N/2 bits.
//N *must* be a power of 2.
{
int    j,k,N2;
double TwoPiN;

    N2 = N >> 1; // N2 = N/2
    TwoPiN = 2*PI/N;

    // table test:
    if (tabBR[1] != (N2>>1)) 
      ROS_ERROR("exps2fft: wrong bit reversing table.");

    w[0] = 1.0; w[1] = 0.0;   // WN0 = 1
    for (k=1; k<N2; k++)
     {
        j = tabBR[k] << 1;
        w[j]  = (Real) cos(TwoPiN*k); // WNk = W_N^k
        w[j+1]= (Real) sin(TwoPiN*k);
     }

}//exps2fft
//------------------------------------------------------------------------------

//=======================================
void FFT::tab4bitrev(int N)
//=======================================
// Computs the bit-reversal table for N indexes, tabBR (member).
//   N must be a power of 2 and greater than 4.
//   Memory for tabBR already allocated.
{
int i,j,k,N2;

	//Test for N:
	if (N<8 || ((N & (N-1))!=0))
		ROS_ERROR("N is not a power of 2 and/or not greater than 4.");

    N2 = N >> 1;
    *tabBR = 0;					// 0
    tabBR[1] = N2;				// N/2
    tabBR[2] = N2>>1;			// N/4
    tabBR[3] = j = tabBR[2]+N2;	// N/4+N/2

    for (i=4; i<N; i++)
     {
        k = N2;
        while (k <= j) 
        {
            j-=k; 
            k >>= 1; 
        }
        j+=k;  //##### i: normal order; j: bit-reversed order
        tabBR[i]=j;
     }
} //tab4bitrev
//------------------------------------------------------------------------------

//==================================================
void FFT::init(int N)
//==================================================
//Prepars for DFT/FFT computation of length N,
//where N is a power of 2.
{
    if (N > Nfft) //new exponentials ?
    {
        if (w!=NULL) // is w already allocated?
        {
            free(w);
            free(tabBR);
        }
		int N2 = N >> 1; //N/2
        w     = (Real *) malloc(N*sizeof(Real));
        tabBR = (int *)  malloc(N2*sizeof(int));
        Nfft = N;
        tab4bitrev(N2);
        exps2fft(N);
    }
    else if (N<Nfft)
    {
        tab4bitrev(N >> 1);   //only computs a new bit-reversal table. Exponentials are the same.
    }
}//testBed

//==================================================
void FFT::fft_real(Real x[], int N)
//==================================================
//FFT of real vector, x[n], n=0..N-1;
//Nfft=N must be a power of 2.
//Algorithm:
// 1. Takes real x[n] as a complex signal z[n]=x[2*n]+j*x[2*n+1];
// 2. Computs Z(k).
// 3. Do trigonometric recombination for computing X(k).
//
//X(k), complex, is computed in-place, so x is changed. The result is:
//x[0]=X(0); x[1]=X(N/2) and
//x[2*k]=real(X(k)); x[2*k+1]=imag(X(k)), k=1...N/2-1
//
{
int     i,k,ik,Nk,iNk,N2,N4;
Real    aR,aI,bR,bI,c,s,dR,dI;
Complex *Z,Zk,ZNk;

	init(N);
    N2 = N>>1;      // N/2
    N4 = N2>>1;     // N/4

    //DFT de z[n] = x[2*n]+j*x[2*n+1] (of length N/2):

    cfftr2_dit(x,N2);

    //Bit-Reversing and trigonometric recombination:

    Z =(Complex *)x;

    for (k=1; k<N4; k++)
    {
        Nk = N2-k;
        ik = tabBR[k];
        iNk= tabBR[Nk];

        //bit reversing of the output, at pairs (k,Nk)

        if (k<ik && ik<Nk)      // Note: If ik>Nk, the exchange is already done: iNk was already equal to actual k.
        {
            Zk = Z[ik]; Z[ik] = Z[k]; // exchange Z(k) with Z(ik). Z(k) will be changed
        }
        else Zk = Z[k];

        if (k<iNk && iNk<Nk)
        {
            ZNk = Z[iNk]; Z[iNk] = Z[Nk];
        }
        else ZNk = Z[Nk];

        // In-place trigoonometric recombination:
        //   -j*WNk = -(s+j*c)
        //
        // Zk  >------.------>(+)---------------.------>(+)--------->---> Xk
        //             \     /   a               \     /   a+d     0.5
        //               \ /                       \ / 
        //               / \                       / \ 
        //        conj / -1  \   b   -(s+j*c)  d / -1  \   a-d     0.5
        // ZNk >--(*)-´--->-->(+)------->-------´--->-->(+)----(*)-->---> XNk


        aR = Zk.real+ZNk.real;  aI = Zk.imag-ZNk.imag;  // a  = Zk + conj(ZNk)
        bR = Zk.real-ZNk.real;  bI = Zk.imag+ZNk.imag;  // b  = Zk - conj(ZNk)

        i = ik<<1;   // index of WNk = c+j*s, within w
        c = w[i++];
        s = w[i];

        dR = c*bI - s*bR;       dI = -c*bR - s*bI;      // d  =-b*(s+j*c)

        Z[k ].real = (Real)0.5*(aR+dR); Z[k ].imag = (Real)0.5*(dI+aI);   // Xk = (a+d)/2
        Z[Nk].real = (Real)0.5*(aR-dR); Z[Nk].imag = (Real)0.5*(dI-aI);   // XNk= conj(a-d)/2
    }
    // X(0):
    c = Z->real;
    Z->real += Z->imag;         //X(0) = Re{X(0)} = Re{Z(0)} + Im{Z(0)}
    Z->imag = c - Z->imag;      //X(N/2) = Re{X(N/2)} = Re{Z(0)} - Im{Z(0)} ---> put in imag part of X[0]

    // X(N/4):
    Z[N4].imag = -Z[N4].imag;   // X(N/4) = conj(Z(N/4))


}//fft_real
//--------------------------------------------------------------------------------------------------------------------

//===============================================================
void FFT::powfft(Real X[],int N)
//===============================================================
//Computs the square magnitude of DFT X(k), in-place: abs(X(k))^2.
//Put in X a real sequence of N/2+1 Real values of abs(X(k))^2, k=0:N/2.
//where the input X has:
//X[0]=real(X(0)); X[1]=real(X(N/2)) and
//X[2*k]=real(X(k)); X[2*k+1]=imag(X(k)).
//
//Changes X -> |X(k)|^2, k=0...N/2, in its N/2+1 first indexes.
//The remaining X are left unchanged.
{
int k,N2;
Real XN2,Xr,Xi,*pX;

    N2 = N>>1;
    X[0] *= X[0];
    XN2 = X[1];

    pX = X+2;
    for (k=1;  k<N2;  k++)
    {
        Xr = *pX++;
        Xi = *pX++;
        X[k] = Xr*Xr + Xi*Xi; //=X[N-k]
    }
    X[N2] = XN2*XN2;
}//powfft
//------------------------------------------------------------------------------

//=================================================================
void FFT::ifft_cs(Real x[], int N)
//=================================================================
//====== IDFT/IFFT routine of Complex Simetric (CS) sequence ======
//IFFT of CS (complex simetric) vector X(k), k=0..N/2; N is a power of 2.
//With x=X, X(k) is the Complex CS sequence defined with only N/2+1 values, where
//X(0) = Xr(0)+j*Xr(N/2);
//X(1) = Xr(1)+j*Xi(1);
//...
//X(N/2-1) = Xr(N/2-1)+j*Xi(N/2-1).
//That is, X has the same format as the result of calling realfft(x).
//
//Computs x[n], real, (in-place), n=0..N-1.
//
//This implies that calling in sequence
// fft_real(x,N);
// ifft_cs(x,N);
//produces the same initial x, unless the numeric precision.

{
int k,ik,Nk,N2,N4,n,r;
Real aR,aI,bR,bI,c,s,dR,dI,invN;

	init(N);
    N2 = N>>1;      // N/2
    N4 = N2>>1;     // N/4
	invN=(Real)1.0/N;

	//--------------------------------------
	//Algorithm:
	//We want to compute z[n]=x[2n]+j*x[2n+1] from X(k), so that the samples of
	//x[n] be contiguous.
	//We difine a[n]=x[2n]; b[n]=x[2n+1], and z[n]=a[n]+j*b[n]. Then Z(k)=A(k)+jB(k).
	//The iDFT if X(k), being CS, gives:
	//x[2n]   = iDFT{ X(k)+X*(N/2-k) }           = iDFT{A(k)}
	//x[2n+1] = iDFT{ (X(k)-X*(N/2-k))*WN^(-k) } = iDFT{B(k)}
	//But z[n] =(2/N)*DFT{Z(N/2-k)}, n,k=0...N/2-1.
	//
	//Passes
	//1.Compute:
	//    C(k)=(2/N)*Z(N/2-k) = { [X*(k)+X(N/2-k)] +jWN^k)*[X*(k)-X(N/2-k)]  }*(1/N) = (a+d)/N
	//                            <----- a ------>         <----- b ------>
	//                                              <--------- d --------->
	// and, at the same time,  C(N/2-k) = conj(a-d)/N. 
	// Calculate separately C(0)=X(0)+X(N/2) +j*( X(0)-X(N/2)) and C(N/4)=conj(X(N/4)).
	//
	//2. Compute  DFT{C(k)} = z[n]
	//
	// Trigonometric Recombination, in-place, with +j*WN^k = +j(c-js)= (s+j*c)
	//
	//  X(k)   o--(*)-o-------(+)---------------o-------(+)--------->---> C(k)
	//           conj  \     /    a              \     /    a+d    1/N
	//                   \ /                       \ / 
	//                   / \                       / \ 
	//                 /     \    b  (s+j*c)  d  /     \    a-d    1/N
	//X(N/2-k) o------o-(-1)--(+)------->-------o-(-1)--(+)----(*)-->---> C(N/2-k)

	for (k=1; k<N4; k++)
	{ 
		ik=k<<1;
		Nk=(N2-k)<<1;    // Nk = 2*(N/2-k).
		r =tabBR[k]<<1;  // índice de WN(-k), em w
		c=w[r]; s=w[r+1];// c = cos(2*pi*k/N); s = sin(2*pi*k/N)
		aR=x[ik]+x[Nk]; aI=x[Nk+1]-x[ik+1];       // a  = X*(k) + X(N/2-k)
		bR=x[ik]-x[Nk]; bI=-x[ik+1]-x[Nk+1];      // b  = X*(k) - X(N/2-k)
		dR=s*bR-c*bI;   dI= c*bR+s*bI;            // d  = b*(s+j*c) =(bR+j*bI)*(s+j*c)
		x[ik]=invN*(aR+dR); x[ik+1]=invN*(aI+dI); // C(k)    = (a+d)/N
		x[Nk]=invN*(aR-dR); x[Nk+1]=invN*(dI-aI); // C(N/2-k)= (a* - d*)/N
	}
	// The remaining indexes 0 and N/4:
	aR=x[0]; aI=x[1];  // X(N/2) put as imag(X(0))
	x[0]=(aR+aI)*invN; // Cr(0)
	x[1]=(aR-aI)*invN; // C(0)= [X(0)+X(N/2) +j*( X(0)-X(N/2))]/N
	invN *= 2;         // invN=2/N */
	x[N2] *= invN;
	x[N2+1] *= -invN;   // C(N/4) = conj{X(N/4)}*(2/N)

	cfftr2_dit(x,N2);	// z[n] = DFT{C(k)}

	// bit reversing
	for (k=1; k<N2; k++)
	{ 
		ik =tabBR[k];
		if (k<ik) // exchange x[k] with x[ik], x is complex
		{	n   = k<<1; r=ik<<1; // troca x(n) com x(r) e x(n+1) com x(r+1), x real
			aR  = x[n]; aI=x[n+1];
			x[n]=x[r];  x[n+1]= x[r+1];
			x[r]=aR;    x[r+1]= aI;
		}
	}

}//ifft_cs
//------------------------------------------------------------------------------
//
////==================================================
//void FFT::ifft_real(Real x[], int N)
////==================================================
////IDFT/IFFT of real and even vector x=X[k].
////The same routine as csifft(), but where the sequence X(k) is real (besides being CS).
////Useful to compute autocorrelation, if X(k) = Y(k)*conj(Y(k)) = |Y(k)|^2 (real and even DFT).
//{
//int k,ik,Nk,N2,N4,n,r;
//Real aR,bR,c,s,dR,dI,invN;
//
//	init(N);
//    N2 = N>>1;      // N/2
//    N4 = N2>>1;     // N/4
//	invN=(Real)1.0/N;
//
//	for (k=1; k<N4; k++)
//	{
//		ik=k<<1;
//		Nk=(N2-k)<<1;    // Nk = 2*(N/2-k).
//		r =tabBR[k]<<1;  // index of WN(-k), in w
//		c=w[r]; s=w[r+1];// c = cos(2*pi*k/N); s = sin(2*pi*k/N)
//		aR=x[ik]+x[Nk];  // a  = X*(k) + X(N/2-k) !!!!!!!!!! Do not compute aI !!!!!!!!!!!!
//		bR=x[ik]-x[Nk];  // b  = X*(k) - X(N/2-k) !!!!!!!!!! Do not compute bI !!!!!!!!!!!!
//		dR=s*bR;   dI= c*bR;   // d  = b*(s+j*c) =(bR+j*bI)*(s+j*c)
//		x[ik]=invN*(aR+dR); x[ik+1]=x[Nk+1]=invN*(dI); // C(k)    = (a+d)/N
//		x[Nk]=invN*(aR-dR); //x[Nk+1]= invN*(dI);      // C(N/2-k)= (a* - d*)/N
//	}
//	dR=x[0]; dI=x[1];  // X(N/2) in imag(X(0)=x[1])
//	x[0]=(dR+dI)*invN;
//	x[1]=(dR-dI)*invN; // C(0)= [X(0)+X(N/2) +j*( X(0)-X(N/2))]/N
//	x[N2] *= (2*invN);
//
//	cfftr2_dit(x,N2);	// z[n] = DFT{C(k)}
//
//	// bit reversing
//	for (k=1; k<N2; k++)
//	{
//		ik =tabBR[k];
//		if (k<ik)
//		{
//			n   = k<<1; r=ik<<1;
//			dR  = x[n]; dI=x[n+1];
//			x[n]=x[r];  x[n+1]= x[r+1];
//			x[r]=dR;    x[r+1]= dI;
//		}
//	}
//}//ifft_real
////------------------------------------------------------------------------------

//=================================================================
void FFT::xcorr(Real x[], Real y[], int N, int M, int mode)
//=================================================================
//Computes the auto-correlation of x[n], Rxx[i], and the cross-correlation between x[n] and y[n], Rxy[i].
// Both x[n] and y[n] are real with length M (but alloc for N values).
//Computs Rxx and/or Rxy according to <mode>:
//	mode=1: only autocorrelation   (Rxx[i] -> x, y unchanged)
//	mode=2: only cross-correlation (Rxy[i] -> y, X[k] -> x)
//	mode=3: both auto- and cross-correlation (Rxx[i] -> x and Rxy[i] -> y).
//No normalization.
//The length of x[n] and y[n] is M (M<N but with memory allocation for N values);
//The other samples, from M to N-1, are zeroed here.
//Equations:
// rxx[i] = Sum{x[n+i]*x[n], n=0...M-1-i}, i=0...M-1; rxx[-i]=Rxx[i], i = 1...M-1.
// rxy[i] = Sum{x[n+i]*y[n], n=0...M-1-i}, i=0...M-1; rxy[-i]=Ryx[i], i = 1...M-1.
//With M<N<2M-1, there is temporal aliasing: 
//ONLY N-M+1 values of the auto- or cross-correlation are correct (in non-negative indexes).
//Algorithm: Computes X(k) and Y(k) and the IDFTs of |X(k)|^2  and of X(k)*conj(Y(k)):
// rxx[n] = conv(x[n],x[-n]) (has 2M-1 values, from n=-(M-1)...M-1) <--> X(w)*conj(X(w)) = |X(w)|^2.
// rxy[n] = conv(x[n],y[-n]) (has 2M-1 values, from n=-(M-1)...M-1) <--> X(w)*conj(Y(w)) 
// Rxx[n]=IDFT{|X[k]|^2} equals rxx[n] (placed in x) for n=0...N-M and Rxx[N-n]=rxx[-n] for n=1...N-M.
// Rxy[n]=IDFT{X(k)Y*(k)}equals rxy[n] (placed in y) for n=0...N-M and Rxy[N-n]=rxy[-n] for n=1...N-M.
// (Rxx[n] is one period of the periodic overlapping of rxx[n] (2M-1 samples) every N samples)
//If N>=2M all samples are correct.
//Example:
//If M=1000 and N=1024, the auto- and cross-correlations are correct in the indexes corresponding to -24:24:
// n=0...24 (25=N-M+1 values) and n=1000...1023 (24=N-M values) corresponding to negative indexes.
//If M=24 and N=1024, all 2M-1=49 values are correct for n=0...24 and n=1000...1023.
//If M=512 and N=1024, all the 1023 correlation values are correct and there is one zero at Rxx[N/2].
{
	int k,k1;
	Real Yr, Yi, Xr, Xi;

	if (mode < 1 || mode>3)
		ROS_ERROR("mode must be 1(=01b), 2(=10b) or 3=(11b).");
	init(N);
	if (M>=N) ROS_ERROR("M>=N");

	// Force zeros in x (x has only M samples):
	for (k=M; k<N; k++) x[k] = (Real)0.0;
	fft_real(x,N);
	if (mode > 1) //mode=2 or mode=3
	{
		for (k = M; k<N; k++) y[k] = (Real)0.0;
		fft_real(y, N);
		//Computation of X*conj(Y)
		y[0] *= x[0]; y[1] *= x[1];
		for (k = 2; k < N; k += 2)
		{
			k1 = k + 1;
			Yr = y[k]; Yi = y[k1];
			Xr = x[k]; Xi = x[k1];
			y[k]  = Xr*Yr + Xi*Yi; // [Xr+j*Xi]*[Yr-j*Yi]
			y[k1] = Xi*Yr - Xr*Yi;
		}
		ifft_cs(y, N);   //cross-correlation
	}
	if (mode==1 || mode==3) {
		//Computation of |X(k)|^2 in-place
		x[0] *= x[0]; x[1] *= x[1];
		for (k = 2; k < N; k += 2)
		{
			k1 = k + 1;
			Xr = x[k]; Xi = x[k1];
			x[k] = Xr*Xr + Xi*Xi;
			x[k1] = 0;
		}
		//ifft_of_real(x, N); //autocorrelation
		ifft_cs(x, N);   //cross-correlation
	}
}//xcorr

////======================================================================
//void FFT::autocorr(Real x[], int N, int M)
////======================================================================
//{
//	xcorr(x, NULL, N, M, 1);
//}
////===========================================================

//===================================================================
void FFT::gcc_phat(Real x[], Real y[], int N, int M)
//===================================================================
//Computs in y the so-called GCC-PHAT method by computing Z1[k] = X[k]*conj(Y[k]);
//Z2[k] = Z1[k]/abs(Z1[k]) and IFFT of this result (put in y).
//Uses a slightly different method:
//Z2[k] = Z1[k]/abs(Z1[k]) if abs(Z1[k]) > Threshold; else Z2[k] = 0;
//Threshold = min(min(abs(Z1(k))*1000,max(abs(Z1[k]))/1e4);
//NOTE: see xcorr().
{
	int k, k1;
	Real Yr, Yi, Xr, Xi;
	Real Threshold;

	init(N);
	if (M >= N) ROS_ERROR("M>=N");

	// Force zeros in x (x has only M samples):
	for (k = M; k<N; k++) x[k] = y[k] = (Real)0.0;
	
	fft_real(x, N); //X[k] in x
	fft_real(y, N); //Y[k]
	//Computation of X*conj(Y)
	y[0] *= x[0]; y[1] *= x[1];
	for (k = 2; k < N; k += 2)
	{
		k1 = k + 1;
		Yr = y[k]; Yi = y[k1];
		Xr = x[k]; Xi = x[k1];
		y[k] = Xr*Yr + Xi*Yi; // [Xr+j*Xi]*[Yr-j*Yi] = Z1[k] in y
		y[k1] = Xi*Yr - Xr*Yi; // imag part
		//y has Z1[k]=X[k]*conj(Y[k]). Compute abs(Z1[k]) in x[k]:
		x[k] = sqrt(y[k] * y[k] + y[k1] * y[k1]);
	}
	//Normalization:
	Yr = fabs(y[0]); Yi = fabs(y[1]); //Y1 = abs(Z1[N/2])
	Xr = Xi = Yr; 

	for (k = 2; k < N; k += 2) {
		if (x[k] > Xr) Xr = x[k];  // max(abs(Z1[k])) in Xr
		if (x[k] < Xi) Xi = x[k];  // min(abs(Z1[k])) in Xi
	} //Now Xr has  max(abs(y))
	if (Yi > Xr) Xr = Yi; //Z1[N/2] is max?
	if (Yi < Xi) Xi = Yi; //Z1[N/2] is min?
	Xr *= 1e-4;
	Xi *= 1e+3;
	Threshold = (Xr > Xi ? Xi : Xr); //min(Xr,Xi); Threshold = min(min(abs(Z1(k))*1e3,max(abs(Z1[k]))*1e-4);
	//------------------------------
	//Z1[0]
	if (Yr > Threshold) {
		y[0] = (y[0] > 0 ? 1.0 : -1.0);
	} else
		y[0] = 0;
	//Z1[N/2]
	if (Yi > Threshold) {
		y[1] = (y[1] > 0 ? 1.0 : -1.0);
	}
	else
		y[1] = 0;
	for (k = 2; k < N; k += 2)
	{
		k1 = k + 1;
		Xr = x[k];
		if (Xr > Threshold) {
			y[k] /= Xr;	//Normalization by abs
			y[k1] /= Xr;
		} else
			y[k] = y[k1] = 0;
	}

	ifft_cs(y, N);   //PHAT cross-correlation

}//gcc_phat

//======================================================================
void FFT::shift_pn(Real x[], int N, int M)
//======================================================================
//Puts the last M-1 values of x[n] after the first M, M<=N/2, resulting in:
// {x[0], x[1],...,x[M-1],x[N-M+1],...,x[N-1]} (2M-1 values)
// {rxx[0],...,rxx[M-1],rxx[-(M-1)]),...,rxx[-1]}; rxx[0] in x[0]
//If x[n] represents a sequence with negative indexes, the values with negative indexes are after: p_n
//See shift_np.
//Note: to use after call of xcorr() or autocor().
{
	if (N < 2 * M) ROS_ERROR("M>N/2");
	if (N == 2 * M) return; //nothing to do.
	int M1 = M - 1;
	Real *p1 = x + M;
	Real *p2 = x + N - M1;
	for (int n = 0; n < M1; n++) {
		*p1++ = *p2++;
	}
}
//======================================================================
void FFT::shift_np(Real x[], int N, int M)
//======================================================================
//Puts the last M-1 values of x[n] before the first M, M<=N/2, resulting in:
//  {x[N-M+1],...,x[N-1],x[0], x[1],...,x[M-1]} (2M-1 values)
//  rxx[-(M-1)],...,rxx[M-1]); rxx[0] in x[M-1]; rxx[i] in x[M-1+i]
//If x[n] represents a sequence with negative indexes, the values with negative indexes are first: n_p
//See shift_pn.
//Note: to use after call of xcorr() or autocor().
{
	if (N < 2 * M) ROS_ERROR("M>N/2");
	if (N == 2 * M) { //as in fftshift: {X[N/2],X[N-k],X[0],X[k]}, k=1...N/2-1
		Real xx;
		for (int k = 0; k < M; k++) { //exchange x[k] with x[M+k]
			xx = x[k];
			x[k] = x[k + M];
			x[k + M] = xx;
		}
	} else {
		Real *x1 = (Real *)malloc(M * sizeof(Real));
		int M1 = M - 1;
		for (int n = 0; n < M1; n++) {
			x1[n] = x[n];
			x[n] = x[n + N - M1];
		}
		x1[M1] = x[M1];
		for (int n=0; n < M; n++) {
			x[n + M1] = x1[n];
		}
		free(x1);
	}
}
