/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: G32_parsing_emxAPI.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 29-Jul-2024 12:55:51
 */

/* Include Files */
#include "G32_parsing_emxAPI.h"
#include "G32_parsing_emxutil.h"
#include "G32_parsing_types.h"
#include "rt_nonfinite.h"
#include <stdlib.h>

/* Function Definitions */
/*
 * Arguments    : int numDimensions
 *                const int *size
 * Return Type  : emxArray_real32_T *
 */
emxArray_real32_T *emxCreateND_real32_T(int numDimensions, const int *size)
{
  emxArray_real32_T *emx;
  int i;
  int numEl;
  emxInit_real32_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }
  emx->data = (float *)malloc((unsigned int)numEl * sizeof(float));
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

/*
 * Arguments    : float *data
 *                int numDimensions
 *                const int *size
 * Return Type  : emxArray_real32_T *
 */
emxArray_real32_T *emxCreateWrapperND_real32_T(float *data, int numDimensions,
                                               const int *size)
{
  emxArray_real32_T *emx;
  int i;
  int numEl;
  emxInit_real32_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }
  emx->data = data;
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

/*
 * Arguments    : float *data
 *                int rows
 *                int cols
 * Return Type  : emxArray_real32_T *
 */
emxArray_real32_T *emxCreateWrapper_real32_T(float *data, int rows, int cols)
{
  emxArray_real32_T *emx;
  emxInit_real32_T(&emx, 2);
  emx->size[0] = rows;
  emx->size[1] = cols;
  emx->data = data;
  emx->numDimensions = 2;
  emx->allocatedSize = rows * cols;
  emx->canFreeData = false;
  return emx;
}

/*
 * Arguments    : int rows
 *                int cols
 * Return Type  : emxArray_real32_T *
 */
emxArray_real32_T *emxCreate_real32_T(int rows, int cols)
{
  emxArray_real32_T *emx;
  int numEl;
  emxInit_real32_T(&emx, 2);
  emx->size[0] = rows;
  numEl = rows * cols;
  emx->size[1] = cols;
  emx->data = (float *)malloc((unsigned int)numEl * sizeof(float));
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  return emx;
}

/*
 * Arguments    : emxArray_real32_T *emxArray
 * Return Type  : void
 */
void emxDestroyArray_real32_T(emxArray_real32_T *emxArray)
{
  emxFree_real32_T(&emxArray);
}

/*
 * Arguments    : emxArray_real32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInitArray_real32_T(emxArray_real32_T **pEmxArray, int numDimensions)
{
  emxInit_real32_T(pEmxArray, numDimensions);
}

/*
 * File trailer for G32_parsing_emxAPI.c
 *
 * [EOF]
 */
