/*
 * Logic for writing opencv matrices to a framebuffer 
 * Author: Christopher Rooney
 *
 */

#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace LibSeek {

class Framebuffer
{
public:
	Framebuffer(const char* filename);
	~Framebuffer(); //destructor
	void writeFrame(cv::Mat& frame);
	float getScale(cv::Mat& frame);
	void setUncopiedRows(uint numrows);
	void closefb();

protected:
	int m_fbfd;
	struct fb_var_screeninfo m_vinfo;
	struct fb_fix_screeninfo m_finfo;
	long int m_screensize;
	char* m_fbp;
	uint m_uncopiedrows;
};

}/*libseek namespace*/


#endif //FRAMEBUFFER_H