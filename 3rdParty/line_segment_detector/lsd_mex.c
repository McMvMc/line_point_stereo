#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "lsd.h"
#include "mex.h"



void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  double * img = mxGetPr(prhs[0]);
  double * out;
  double * out_ptr;
  int i,j,n;
  int X = (int)mxGetM(prhs[0]);
  int Y = (int)mxGetN(prhs[0]);
  
  
  // default parameters
  double scale = 0.8;
  double sigma_scale = 0.8; /* Sigma for Gaussian filter is computed as
                                sigma = sigma_scale/scale.                    */
  double quant = 0.0;       /* Bound to the quantization error on the
                                gradient norm.                                */
  double ang_th = 22.5;     /* Gradient angle tolerance in degrees.           */
  double log_eps = 0.0;     /* Detection threshold: -log10(NFA) > log_eps     */
  double density_th = 0.7;  /* Minimal density of region points in rectangle. */
  int n_bins = 1024;        /* Number of bins in pseudo-ordering of gradient
                               modulus.  */
  
  int offset = 1;
  if(nrhs > offset + 1)
      scale = (double)mxGetScalar(prhs[offset + 1]);
  if(nrhs > offset + 2)
      sigma_scale = (double)mxGetScalar(prhs[offset + 2]);
  if(nrhs > offset + 3)
      quant = (double)mxGetScalar(prhs[offset + 3]);
  if(nrhs > offset + 4)
      ang_th = (double)mxGetScalar(prhs[offset + 4]);
  if(nrhs > offset + 5)
      log_eps = (double)mxGetScalar(prhs[offset + 5]);
  if(nrhs > offset + 6)
      density_th = (double)mxGetScalar(prhs[offset + 6]);
  if(nrhs > offset + 7)
      n_bins = (int)mxGetScalar(prhs[offset + 7]);
  
  
  // process detection mask
  struct coorlist * mask;
  int useWholeImage = 1;
  if(nrhs > 1)
  {
      int N = (int) ceil( Y * scale );
      int M = (int) ceil( X * scale );
      
      double * loc = mxGetPr(prhs[1]);
      int locM = (int)mxGetM(prhs[1]);
      int locN = (int)mxGetN(prhs[1]);
      if(locM==3 && locN>0)
      {
          useWholeImage = 0;
          // create mask from input
          int idx, x, y, r, L, count;
          L = locM*locN;
          count = 0;
          for(idx=2;idx<L;idx+=3)
          {
              r = (int) ceil( loc[idx] * scale );
              count += (2*r+1)*(2*r+1);
          }
          
          if(count > (N-1)*(M-1))
              useWholeImage = 1;
          else
          {
              mask = (struct coorlist *) calloc( (size_t) (count), sizeof(struct coorlist) );
              int mask_count = 0, rx, ry;
              for(idx=0;idx<L;idx+=3)
              {
                  x = (int) ceil( loc[idx] * scale );
                  y = (int) ceil( loc[idx+1] * scale );
                  r = (int) ceil( loc[idx+2] * scale );
                  for(ry=y-r;ry<=y+r;ry++)
                      for(rx=x-r;rx<=x+r;rx++)
                      {
                          if(ry<0 || ry>=N-1 || rx<0 || rx>=M-1)
                              continue;
                          (mask+mask_count)->x = rx;
                          (mask+mask_count)->y = ry;
                          (mask+mask_count)->next = mask+mask_count+1;
                          mask_count ++;
                      }
              }
              (mask+mask_count)->next = NULL;
          }
      }
  }
  
  if(useWholeImage==1)
  {
      // calculate frame size
      int N = (int) ceil( Y * scale );
      int M = (int) ceil( X * scale );

      mask = (struct coorlist *) calloc( (size_t) ((N-1)*(M-1)), sizeof(struct coorlist) );
      int x, y, adr;
      for(y=0;y<N-1;y++)
          for(x=0;x<M-1;x++)
          {
              adr = y*(M-1) + x;
              (mask+adr)->x = (int) x;
              (mask+adr)->y = (int) y;
              (mask+adr)->next = mask+adr+1;
          }
      (mask+adr)->next = NULL;
  }
      
  
  /* LSD call */

  out = MaskedLineSegmentDetection( &n, img, X, Y, mask, scale, sigma_scale, quant,
                               ang_th, log_eps, density_th, n_bins,
                               NULL, NULL, NULL );
  /*out = lsd(&n,img,X,Y);*/
  


  plhs[0] = mxCreateDoubleMatrix( 7, n, mxREAL ); 
  out_ptr = mxGetPr(plhs[0]);
  
  for (i = 0; i < n; i++)
	for (j = 0; j < 7; j++)
		out_ptr[7*i + j] = out[7*i + j];
  
  // clear memory
  free( (void *) mask );
  free( (void *) out );
  
  return;
}
