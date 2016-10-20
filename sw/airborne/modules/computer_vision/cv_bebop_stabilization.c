/*
 * Copyright (C) W Vlenterie
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_bebop_stabilization.c"
 * @author W Vlenterie
 * Image stabilization for bebop front camera
 */

#include "modules/computer_vision/cv_bebop_stabilization.h"
#include "modules/computer_vision/lib/opengl/opengl.h"
#include "modules/computer_vision/cv.h"
#include <stdio.h>

#ifndef BS_CAMERA
#define BS_CAMERA front_camera
#endif

static const GLfloat vVertices[] = {
		-1.0f, -1.0f,
		1.0f, -1.0f,
		-1.0f,  1.0f,
		1.0f,  1.0f,
};
static const GLfloat textureVertices[] = {
		0.0f, 0.0f,
		1.0f, 0.0f,
		0.0f, 1.0f,
		1.0f, 1.0f,
};

static GLfloat lens[] = {
		 1.0, // a
		  1.0, // b
		  1.0,  // F
		  1.0     // scale
};

static GLfloat fov[] = {
		1.0, // x
		1.0  // y
};

struct image_t* cv_bebop_stabilization_func(struct image_t * img);
struct image_t* cv_bebop_stabilization_func (struct image_t *img)
{
	printf("Passing to GPU\n");
	glUseProgram(opengl.programObject);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, mt9f002.output_width, mt9f002.output_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img->buf);
	// Set the vertices
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, vVertices);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, 0, 0, textureVertices);
	glEnableVertexAttribArray(1);
	// Pass variables
	glUniform4fv(2, 4, &lens[0]);
	glUniform2fv(3, 2, &fov[0]);
	// Draw the square
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	// Read the image back
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glReadPixels(0, 0, mt9f002.output_width, mt9f002.output_height, GL_RGBA, GL_UNSIGNED_BYTE, img->buf);
	return img;
}


void cv_bebop_stabilization_init ()
{
	printf("Initializing front camera stabilization\n");
	opengl_init();
	cv_add_to_device(&BS_CAMERA, cv_bebop_stabilization_func);
	return;
}

