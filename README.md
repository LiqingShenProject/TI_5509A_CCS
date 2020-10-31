# TI_5509A_CCS
This example utilize the DSP core of TMS320VC5509A to generate error corrected xz_x, xz_y.
Due to the non-linearity of Lens, the xy point on the focal plane suffered pin-cushion and barrel distortion
A 128X128 error correction table is generated from FPGA and sent to DSP via CE_0# external memory interface
The original x,y line buffer is also filled up by FPGA via CE_0# external memory interface
The example here use a linear interpolation method the error correct the original x,y.
