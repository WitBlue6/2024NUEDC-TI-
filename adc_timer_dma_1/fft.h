#ifndef FFT_H_
#define FFT_H_

void GetMag(void);

struct compx {float real,imag;};                        //定义一个复数结构struct compx s[128];
void FFT(struct compx *);

#endif /* FFT_H_ */


