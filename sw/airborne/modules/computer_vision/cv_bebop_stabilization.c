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
#include <state.h>
#include <stdio.h>

#include BOARD_CONFIG

#ifndef BEBOP_STAB_PERSPECTIVE_CORRECTION
#define BEBOP_STAB_PERSPECTIVE_CORRECTION 1
#endif


#ifndef BS_CAMERA
#define BS_CAMERA front_camera
#endif

static const GLfloat vVertices[] = {
		-1.0f, -1.0f,
		1.0f, -1.0f,
		-1.0f,  1.0f,
		1.0f,  1.0f,
};

//indices
const ushort indices[] = {
		0, 1, 2, 1, 2, 3
};

static const GLfloat textureVertices[] = {
		0.0f, 0.0f,
		1.0f, 0.0f,
		0.0f, 1.0f,
		1.0f, 1.0f,
};

// Lens correction parameter k
int   noroll 		= 0;
int   nopitch 		= 0;
int   noprojection 	= 0;
float k_fisheye 	= 1.053;
float scale_x 		= 0.52;
float scale_y 		= 0.52;
float focalLength 	= 1 / ( 4 * 0.70711 ); // = sind(180/4) 0.70711
float near 			= 0.1;
float far 			= 100.0;
float angleOfView 	= 150; // 160 degrees
float imgRot 		= 0.0;
float frameRot 		= 0.0;

GLuint ULMVPMat, ULscalar, ULlensCentre, ULaspectRatio, ULfocalLength, ULk;

struct image_t* cv_bebop_stabilization_func (struct image_t *img)
{
	glUseProgram(opengl.programObject);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mt9f002.output_width/2, mt9f002.output_height, GL_RGBA, GL_UNSIGNED_BYTE, img->buf);
	// Set the vertices
	glVertexAttribPointer((GLuint) 0, 2, GL_FLOAT, GL_FALSE, 0, vVertices);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer((GLuint) 1, 2, GL_FLOAT, 0, 0, textureVertices);
	glEnableVertexAttribArray(1);
	mat4 modelviewProjection, modelMat, viewMat, modelviewMat, projectionMat;
	vec4 eye, forward, up;
	eye[0] 		= 0.0;  eye[1] 		=  0.0; eye[2] 		= -1.0; 	eye[3] 		= 1.0;
	forward[0]  = 0.0;  forward[1] 	=  0.0; forward[2]  =  1.0; 	forward[3] 	= 1.0;
	up[0] 		= 0.0;  up[1] 		=  1.0; up[2] 		=  0.0; 	up[3] 		= 1.0;
	// Create The model matrix
	mat4 sensorRotation, eyeTranslationF, headingRotation, eyeTranslationR;
	setRotationMat(M_PI, 0.0, - 0.5 * M_PI, sensorRotation); // The ISP is 90 degrees rotated wrt the camera and the pinhole camera model flips the image
	setTranslationMat(-eye[0], -eye[1], -eye[2], eyeTranslationF);
	setRotationMat(0.0, img->eulerAngles->psi, 0.0, headingRotation);
	setTranslationMat( eye[0],  eye[1],  eye[2], eyeTranslationR);
	mat4 modelMat_tmp1, modelMat_tmp2;
	matrixMultiply(eyeTranslationF, sensorRotation, modelMat_tmp1);
	matrixMultiply(headingRotation, modelMat_tmp1, modelMat_tmp2);
	matrixMultiply(eyeTranslationR, modelMat_tmp2, modelMat);
	// Create the view matrix
	//rotateVector(, 0.0, 0.0, forward);
	//rotateVector(MT9F002_THETA_OFFSET, 0.0, 0.0, up);
	rotateVector(MT9F002_THETA_OFFSET + img->eulerAngles->theta, img->eulerAngles->psi, img->eulerAngles->phi, forward);
	rotateVector(MT9F002_THETA_OFFSET + img->eulerAngles->theta, img->eulerAngles->psi, img->eulerAngles->phi, up);
	vec4 center;
	center[0] = eye[0] + forward[0];
	center[1] = eye[1] + forward[1];
	center[2] = eye[2] + forward[2];
	center[3] = 1.0;
	view_set_lookat(viewMat, eye,  center,  up);
	if(noprojection == 1)
	{
		setIdentityMatrix(projectionMat);
	}else{
		setPerspectiveMat(projectionMat);
	}
	matrixMultiply(viewMat, modelMat, modelviewMat);
	matrixMultiply(projectionMat, modelviewMat, modelviewProjection);

	/*
	printf("          Projection                          View                              Model\n");
	printf("| %5.2f\t%5.2f\t%5.2f\t%5.2f |   | %5.2f\t%5.2f\t%5.2f\t%5.2f |   | %5.2f\t%5.2f\t%5.2f\t%5.2f |\n", projectionMat[0], projectionMat[4], projectionMat[8],  projectionMat[12], viewMat[0], viewMat[4], viewMat[8],  viewMat[12], modelMat[0], modelMat[4], modelMat[8],  modelMat[12]);
	printf("| %5.2f\t%5.2f\t%5.2f\t%5.2f |   | %5.2f\t%5.2f\t%5.2f\t%5.2f |   | %5.2f\t%5.2f\t%5.2f\t%5.2f |\n", projectionMat[1], projectionMat[5], projectionMat[9],  projectionMat[13], viewMat[1], viewMat[5], viewMat[9],  viewMat[13], modelMat[1], modelMat[5], modelMat[9],  modelMat[13]);
	printf("| %5.2f\t%5.2f\t%5.2f\t%5.2f | * | %5.2f\t%5.2f\t%5.2f\t%5.2f | * | %5.2f\t%5.2f\t%5.2f\t%5.2f |\n", projectionMat[2], projectionMat[6], projectionMat[10], projectionMat[14], viewMat[2], viewMat[6], viewMat[10], viewMat[14], modelMat[2], modelMat[6], modelMat[10], modelMat[14]);
	printf("| %5.2f\t%5.2f\t%5.2f\t%5.2f |   | %5.2f\t%5.2f\t%5.2f\t%5.2f |   | %5.2f\t%5.2f\t%5.2f\t%5.2f |\n", projectionMat[3], projectionMat[7], projectionMat[11], projectionMat[15], viewMat[3], viewMat[7], viewMat[11], viewMat[15], modelMat[3], modelMat[7], modelMat[11], modelMat[15]);
	*/
	float aspectRatio 			= mt9f002.output_width /  mt9f002.output_height;
	// Calulate parameters
	const GLfloat lensCentre[] 	= { 0, 0 };
	// Calculate scalar
	float opw = focalLength * tan( asin( sin( atan( 1.0 * aspectRatio / focalLength ) ) * k_fisheye ) ); 	// What is the input pixel at the edge?
	float oph = focalLength * tan( asin( sin( atan( 1.0 / focalLength ) ) * k_fisheye ) ); 					// What is the input pixel at the edge
	const GLfloat scalar[] 		= { 1 / opw, 1 / oph }; 	// Use that to scale the image
	// Pass variables
	glUniformMatrix4fv(ULMVPMat, 1, GL_FALSE, &modelviewProjection[0]);
	glUniform2fv(ULscalar, 		1, &scalar[0]);
	glUniform2fv(ULlensCentre, 	1, &lensCentre[0]);
	glUniform1fv(ULaspectRatio, 1, &aspectRatio);
	glUniform1fv(ULfocalLength, 1, &focalLength);
	glUniform1fv(ULk,			1, &k_fisheye);
	// Draw the square
	//glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glDrawElements(GL_TRIANGLE_STRIP, 6, GL_UNSIGNED_SHORT,  &indices[0]);
	// Read the image back
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glReadPixels(0, 0, mt9f002.output_width/2, mt9f002.output_height, GL_RGBA, GL_UNSIGNED_BYTE, img->buf);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glFinish();
	return img;
}


void cv_bebop_stabilization_init ()
{
	printf("Initializing front camera stabilization\n");
	opengl_init();

	char vShaderStr[] = STRINGIFY(
	uniform   mat4 modelviewProjection;
	attribute vec4 pos;
	attribute vec4 textCoord;
	varying   vec4 v_textCoord;
	void main() {
		gl_Position = modelviewProjection * vec4(pos.x, pos.y, 0.0, 1.0);
		v_textCoord = textCoord;
	}
	);

	char fShaderStr[] = STRINGIFY(
	precision highp float;

	uniform sampler2D 		videoFrame;
	varying vec4 			v_textCoord;

	uniform vec2 			scalarVector;
	uniform vec2 			lensCentre;
	uniform float 			aspectRatio;
	uniform float 			focalLength;
	uniform float 			k;

	vec2 GLCoord2TextureCoord(vec2 glCoord) {
		return glCoord  * vec2(1.0, 1.0)/ 2.0 + vec2(0.5, 0.5);
	}

	vec2 TextureCoord2GLCoord(vec2 glCoord) {
		return (glCoord - vec2(0.5, 0.5))  * vec2(1.0, 1.0) * 2.0 ;
	}

	void main(void)
		{
			// in vec4 gl_FragCoord
			// in bool gl_FrontFacing
			// in vec2 gl_PointCoord
			vec2 outPos     = TextureCoord2GLCoord(v_textCoord.xy);
			//gl_FragColor = texture2D(videoFrame, v_textCoord.xy);
			if(outPos.y > 1.0 || outPos.y < -1.0 || outPos.x > 1.0 || outPos.x < -1.0)
			{
				gl_FragColor   = vec4(0.5, 0.0, 0.5, 0.0);
			}else
			{
				// Calculate relative position
				// Apply a scale to both x and y in order to scale the input image
				vec2 relPos;
				relPos.x 		= (outPos.x - lensCentre.x) / scalarVector.x * aspectRatio;
				relPos.y 		= (outPos.y - lensCentre.y) / scalarVector.y;
				// Apply perspective and rotation
				//relPos     		= VPMatrix * relPos;
				// Fake perspective?
				//relPos.y 		= relPos.y * (1.0 + 0.4 * relPos.x);
				// Convert to radial
				//outPos.y 	= outPos.y / 1.13208;
				vec2 relPosRad;
				relPosRad.x		= sqrt(relPos.y * relPos.y + relPos.x * relPos.x);	// r
				relPosRad.y 	= atan(relPos.y, relPos.x);															// Theta
				// Calculate from which pixel to pull the texture

				// Generic non-linear method for fisheye correction
				vec2 inPosRad;
				inPosRad.x    	= focalLength * tan( asin( sin( atan( relPosRad.x / focalLength ) ) / k ) );
				inPosRad.y 		= relPosRad.y;
				// Convert back to cartesian
				vec2 inPos;
				inPos.x 		= inPosRad.x * cos( relPosRad.y );
				inPos.y 		= inPosRad.x * sin( relPosRad.y );
				// 6-th order barrel distortion
			//vec4 inPos;
			//inPos.xy 		= relPos.xy * (1.0 - 0.2619 * relPosRad.x * relPosRad.x + 0.0582 * relPosRad.x * relPosRad.x * relPosRad.x * relPosRad.x - 0.0402 * relPosRad.x * relPosRad.x * relPosRad.x * relPosRad.x * relPosRad.x * relPosRad.x);
			//inPos.z 		= 1.0; // v_textCoord.z;
			//inPos.w 		= 1.0; //v_textCoord.w;
				// Get that pixel location's texutre location
				vec4 inTexCoord;
				inTexCoord.xy = GLCoord2TextureCoord(inPos.xy);
				if(inTexCoord.x > 0.0 && inTexCoord.x < 1.0 && inTexCoord.y > 0.0 && inTexCoord.y < 1.0)
				{
					gl_FragColor   = texture2D(videoFrame, inTexCoord.xy);
				}else{
					gl_FragColor   = vec4(0.5, 0.0, 0.5, 0.0);
				}
			}
			// out float gl_FrageDepth
		}
	);

	GLuint vertexShader   = opengl_shader_load(vShaderStr, GL_VERTEX_SHADER);
	GLuint fragmentShader = opengl_shader_load(fShaderStr, GL_FRAGMENT_SHADER);
	// Vertex
	glAttachShader(opengl.programObject, vertexShader);
	glBindAttribLocation(opengl.programObject, 0, "pos");
	glBindAttribLocation(opengl.programObject, 1, "textCoord");
	// Fragment
	glAttachShader(opengl.programObject, fragmentShader);
	if(opengl_linkProgram())
	{
		ULMVPMat 		= glGetUniformLocation(opengl.programObject, "modelviewProjection");
		ULscalar    	= glGetUniformLocation(opengl.programObject, "scalarVector");
		ULlensCentre    = glGetUniformLocation(opengl.programObject, "lensCentre");
		ULaspectRatio   = glGetUniformLocation(opengl.programObject, "aspectRatio");
		ULfocalLength   = glGetUniformLocation(opengl.programObject, "focalLength");
		ULk     		= glGetUniformLocation(opengl.programObject, "k");
		printf("[opengl] Uniform locations: %i %i %i %i %i %i\n", ULMVPMat, ULscalar, ULlensCentre, ULaspectRatio, ULfocalLength, ULk);
		glUseProgram(opengl.programObject);

		char fake_buf[mt9f002.output_width * mt9f002.output_height * 2];
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, mt9f002.output_width/2, mt9f002.output_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, &fake_buf);
		cv_add_to_device(&BS_CAMERA, cv_bebop_stabilization_func);
	}
	return;
}

void matrixMultiply(mat4 m1, mat4 m2, mat4 result)
{
	// Fisrt Column
	result[0]  = m1[0]*m2[0] + m1[4]*m2[1] + m1[8]*m2[2] + m1[12]*m2[3];
	result[1]  = m1[1]*m2[0] + m1[5]*m2[1] + m1[9]*m2[2] + m1[13]*m2[3];
	result[2]  = m1[2]*m2[0] + m1[6]*m2[1] + m1[10]*m2[2] + m1[14]*m2[3];
	result[3]  = m1[3]*m2[0] + m1[7]*m2[1] + m1[11]*m2[2] + m1[15]*m2[3];
	// Second Column
	result[4]  = m1[0]*m2[4] + m1[4]*m2[5] + m1[8]*m2[6] + m1[12]*m2[7];
	result[5]  = m1[1]*m2[4] + m1[5]*m2[5] + m1[9]*m2[6] + m1[13]*m2[7];
	result[6]  = m1[2]*m2[4] + m1[6]*m2[5] + m1[10]*m2[6] + m1[14]*m2[7];
	result[7]  = m1[3]*m2[4] + m1[7]*m2[5] + m1[11]*m2[6] + m1[15]*m2[7];
	// Third Column
	result[8]  = m1[0]*m2[8] + m1[4]*m2[9] + m1[8]*m2[10] + m1[12]*m2[11];
	result[9]  = m1[1]*m2[8] + m1[5]*m2[9] + m1[9]*m2[10] + m1[13]*m2[11];
	result[10] = m1[2]*m2[8] + m1[6]*m2[9] + m1[10]*m2[10] + m1[14]*m2[11];
	result[11] = m1[3]*m2[8] + m1[7]*m2[9] + m1[11]*m2[10] + m1[15]*m2[11];
	// Fourth Column
	result[12] = m1[0]*m2[12] + m1[4]*m2[13] + m1[8]*m2[14] + m1[12]*m2[15];
	result[13] = m1[1]*m2[12] + m1[5]*m2[13] + m1[9]*m2[14] + m1[13]*m2[15];
	result[14] = m1[2]*m2[12] + m1[6]*m2[13] + m1[10]*m2[14] + m1[14]*m2[15];
	result[15] = m1[3]*m2[12] + m1[7]*m2[13] + m1[11]*m2[14] + m1[15]*m2[15];
}

void matvecMultiply(mat4 m1, vec4 v1, vec4 result)
{
	// Fisrt Column
	result[0]  = m1[0]*v1[0] + m1[4]*v1[1] + m1[8]*v1[2] + m1[12]*v1[3];
	result[1]  = m1[1]*v1[0] + m1[5]*v1[1] + m1[9]*v1[2] + m1[13]*v1[3];
	result[2]  = m1[2]*v1[0] + m1[6]*v1[1] + m1[10]*v1[2] + m1[14]*v1[3];
	result[3]  = m1[3]*v1[0] + m1[7]*v1[1] + m1[11]*v1[2] + m1[15]*v1[3];
}

void setIdentityMatrix(mat4 m)
{
	m[0]  = m[5]  = m[10] = m[15] = 1.0;
	m[1]  = m[2]  = m[3]  = m[4]  = 0.0;
	m[6]  = m[7]  = m[8]  = m[9]  = 0.0;
	m[11] = m[12] = m[13] = m[14] = 0.0;
}

void setRotationMat(float x, float y, float z, mat4 outputMat)
{
	mat4 rotX;
	mat4 rotY;
	mat4 rotZ;
	setIdentityMatrix(outputMat);
	setIdentityMatrix(rotX);
	setIdentityMatrix(rotY);
	setIdentityMatrix(rotZ);
	// Set z rotation
	rotZ[0]  =  cosf(z);
	rotZ[1]  =  sinf(z);
	rotZ[4]  = -rotZ[1];
	rotZ[5]  =  rotZ[0];
	// Set y rotation
	rotY[0]  =  cosf(y);
	rotY[2]  =  sinf(y);
	rotY[8]  = -rotY[2];
	rotY[10] =  rotY[0];
	// Set x rotation
	rotX[5] = cosf(x);
	rotX[6] = -sinf(x);
	rotX[9] = -rotX[6];
	rotX[10] = rotX[5];

	// Multiply
	mat4 rotXY;
	matrixMultiply(rotY, rotX, rotXY);
	matrixMultiply(rotZ, rotXY, outputMat);
}

void setTranslationMat(float x, float y, float z, mat4 outputMat)
{
	setIdentityMatrix(outputMat);
	outputMat[12] = x;
	outputMat[13] = y;
	outputMat[14] = z;
}

void setPerspectiveMat(mat4 perspectiveMat)
{
	// Create perspective matrix
	// These paramaters are about lens properties.
	// The "near" and "far" create the Depth of Field.
	// The "angleOfView", as the name suggests, is the angle of view.
	// The "aspectRatio" is the cool thing about this matrix. OpenGL doesn't
	// has any information about the screen you are rendering for. So the
	// results could seem stretched. But this variable puts the thing into the
	// right path. The aspect ratio is your device screen (or desired area) width divided
	// by its height. This will give you a number < 1.0 the the area has more vertical
	// space and a number > 1.0 is the area has more horizontal space.
	// Aspect Ratio of 1.0 represents a square area.
	float aspectRatio 	= ((float) mt9f002.output_width) / ((float) mt9f002.output_height);
	// Some calculus before the formula.
	float size 		=  near * tanf((angleOfView / 180.0 * M_PI) / 2.0);
	float left 		= -size;
	float right 	=  size;
	float bottom 	= -size / aspectRatio;
	float top 		=  size / aspectRatio;
	// First Column
	perspectiveMat[0]  = 2 * near / (right - left);
	perspectiveMat[1]  = 0.0;
	perspectiveMat[2]  = 0.0;
	perspectiveMat[3]  = 0.0;
	// Second Column
	perspectiveMat[4]  = 0.0;
	perspectiveMat[5]  = 2 * near / (top - bottom);
	perspectiveMat[6]  = 0.0;
	perspectiveMat[7]  = 0.0;
	// Third Column
	perspectiveMat[8]  = (right + left) / (right - left);
	perspectiveMat[9]  = (top + bottom) / (top - bottom);
	perspectiveMat[10] = -(far + near) / (far - near);
	perspectiveMat[11] = -1;
	// Fourth Column
	perspectiveMat[12] = 0.0;
	perspectiveMat[13] = 0.0;
	perspectiveMat[14] = -(2 * far * near) / (far - near);
	perspectiveMat[15] = 0.0;
}

void translate_xyz(float* result, const float translatex,
		const float translatey, const float translatez) {
	result[12] += result[0] * translatex + result[4] * translatey
			+ result[8] * translatez;
	result[13] += result[1] * translatex + result[5] * translatey
			+ result[9] * translatez;
	result[14] += result[2] * translatex + result[6] * translatey
			+ result[10] * translatez;
	result[15] += result[3] * translatex + result[7] * translatey
			+ result[11] * translatez;
}

float vector_length(const float x, const float y, const float z) {
	return (float) sqrt(x*x + y*y +z*z);
}

void view_set_lookat(mat4 result, vec4 eye, vec4 center, vec4 up) {
	float fx = center[0] - eye[0];
	float fy = center[1] - eye[1];
	float fz = center[2] - eye[2];

	// normalize f
	float rlf = 1.0f / vector_length(fx, fy, fz);
	fx *= rlf;
	fy *= rlf;
	fz *= rlf;

	// compute s = f x up (x means "cross product")
	float sx = fy * up[2] - fz * up[1];
	float sy = fz * up[0] - fx * up[2];
	float sz = fx * up[1] - fy * up[0];

	// and normalize s
	float rls = 1.0f / vector_length(sx, sy, sz);
	sx *= rls;
	sy *= rls;
	sz *= rls;

	//The up vector must not be parallel to the line of sight from the eye point to the reference point.
	if((0 == sx)&&(0 == sy)&&(0 == sz))
		return;

	// compute u = s x f
	float ux = sy * fz - sz * fy;
	float uy = sz * fx - sx * fz;
	float uz = sx * fy - sy * fx;

	result[0] = sx;
	result[1] = ux;
	result[2] = -fx;
	result[3] = 0.0f;

	result[4] = sy;
	result[5] = uy;
	result[6] = -fy;
	result[7] = 0.0f;

	result[8] = sz;
	result[9] = uz;
	result[10] = -fz;
	result[11] = 0.0f;

	result[12] = 0.0f;
	result[13] = 0.0f;
	result[14] = 0.0f;
	result[15] = 1.0f;

	translate_xyz(result, -eye[0], -eye[1], -eye[2]);
}

void rotateVector(float x, float y, float z, vec4 vector)
{
	mat4 rotX;
	mat4 rotY;
	mat4 rotZ;
	setRotationMat(x, 0.0, 0.0, rotX);
	setRotationMat(0.0, y, 0.0, rotY);
	setRotationMat(0.0, 0.0, z, rotZ);
	// Multiply
	vec4 rotXvec, rotYXvec, rotZYXvec;
	matvecMultiply(rotZ, vector,  rotXvec);
	matvecMultiply(rotX, rotXvec, rotYXvec);
	matvecMultiply(rotY, rotYXvec, rotZYXvec);
	vector[0] = rotZYXvec[0];
	vector[1] = rotZYXvec[1];
	vector[2] = rotZYXvec[2];
	vector[3] = rotZYXvec[3];
}
