/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/lib/opengl/opengl.c
 *
 * Handles all the OpenGL ES and EGL setup and shared
 * functions. This is developped for the Mali GPU
 * especially for vision processing.
 */

#include "opengl.h"
#include <stdio.h>
#include <stdlib.h>
#include BOARD_CONFIG

#ifndef STRINGIFY
#define STRINGIFY(s) #s
#endif

/* The main OpenGL structure */
struct opengl_t opengl;

/* OpenGL Config attributes */
static EGLint opengl_config_att[] = {
  EGL_SAMPLES,             4,
  EGL_RED_SIZE,            8,                   /* For UYVY this is U */
  EGL_GREEN_SIZE,          8,                   /* For UYVY this is Y */
  EGL_BLUE_SIZE,           8,                   /* For UYVY this is V */
  EGL_ALPHA_SIZE,          8,                   /* For UYVY this is Y */
  EGL_BUFFER_SIZE,         32,                  /* The total of RGBA 8*4=32 */
  EGL_STENCIL_SIZE,        0,
  EGL_RENDERABLE_TYPE,     EGL_OPENGL_ES2_BIT,  /* Request version 2 of opengl */
  EGL_SURFACE_TYPE,        EGL_PBUFFER_BIT,     /* We are creating a pixel buffer */
  EGL_DEPTH_SIZE,          16,
  EGL_NONE
};

/* OpenGL Context attributes */
static EGLint opengl_context_att[] = {
  EGL_CONTEXT_CLIENT_VERSION, 2, /* Request opengl version 2 */
  EGL_NONE
};

/* OpenGL Pixel Buffer attributes */
static EGLint opengl_pbuffer_att[] = {
  EGL_WIDTH, MT9F002_OUTPUT_WIDTH,
  EGL_HEIGHT, MT9F002_OUTPUT_HEIGHT,
/*  EGL_COLORSPACE, GL_RGB, */
  EGL_TEXTURE_FORMAT, EGL_TEXTURE_RGBA,
  EGL_TEXTURE_TARGET, EGL_TEXTURE_2D,
  EGL_NONE
};

/**
 * Initialize OpenGL and EGL for vision processing
 */
bool opengl_init(void)
{
  EGLBoolean ret;
  EGLint config_nb = 0;

  /* Get the current display */
  opengl.display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  if (opengl.display == EGL_NO_DISPLAY) {
    printf("[opengl] Could not get the default display.\n");
    return FALSE;
  }

  /* Initialize the display (ignore the version number) */
  ret = eglInitialize(opengl.display, NULL, NULL);
  if (!ret) {
    printf("[opengl] Could not initialize the EGL display.\n");
    return FALSE;
  }

  /* Choose an EGL config */
  ret = eglChooseConfig(opengl.display, opengl_config_att, &opengl.config, 1, &config_nb);
  if (!ret || config_nb != 1) {
    printf("[opengl] Could not choose a config.\n");
    eglTerminate(opengl.display);
    return FALSE;
  }

  /* Create a window surface */
  opengl.surface = eglCreatePbufferSurface(opengl.display, opengl.config, opengl_pbuffer_att);
  if (opengl.surface == EGL_NO_SURFACE) {
    printf("[opengl] Could not create a pixel buffer surface.\n");
    eglTerminate(opengl.display);
    return FALSE;
  }

  /* Create a new context */
  opengl.context = eglCreateContext(opengl.display, opengl.config, EGL_NO_CONTEXT, opengl_context_att);
  if (opengl.context == EGL_NO_CONTEXT) {
    printf("[opengl] Could not create context.\n");
    eglTerminate(opengl.display);
    return FALSE;
  }

  /* Set the display, surface and context as current */
  ret = eglMakeCurrent(opengl.display, opengl.surface, opengl.surface, opengl.context);
  if(!ret) {
    printf("[opengl] Could not set the display, surface and context as current.\n");
    eglTerminate(opengl.display);
    return FALSE;
  }

  char vShaderStr[] = STRINGIFY(
	attribute vec4 aVertexPosition;
	attribute vec4 inputTextureCoordinate;
    varying vec4 vPosition;
    varying vec2 vTextureCoord;
    void main() {
      vPosition = aVertexPosition;
      vTextureCoord = inputTextureCoordinate.xy;
      gl_Position = vPosition;
    }
  );

  printf("Shader string:\n");
  char fShaderStr[] = STRINGIFY(
  precision highp float;
  uniform sampler2D videoFrame;

  uniform vec4 uLens;
  uniform vec2 uFov;

  varying vec4 vPosition;
  varying vec2 vTextureCoord;

  vec2 GLCoord2TextureCoord(vec2 glCoord) {
	  return glCoord  * vec2(1.0, -1.0)/ 2.0 + vec2(0.5, 0.5);
  }

  void main(){
	  float scale = uLens.w;
	  float F = uLens.z;

	  float L = length(vec3(vPosition.xy/scale, F));

	  vec2 vMapping = vPosition.xy * F / L;
	  vMapping = vMapping * uLens.xy;

	  vMapping = GLCoord2TextureCoord(vMapping/scale);

	  vec4 texture = texture2D(videoFrame, vMapping);
	  if(vMapping.x > 0.99 || vMapping.x < 0.01 || vMapping.y > 0.99 || vMapping.y < 0.01){
		  texture = vec4(0.0, 0.0, 0.0, 1.0);
	  }
	  gl_FragColor = texture;
  });

  GLuint vertexShader = opengl_shader_load(vShaderStr, GL_VERTEX_SHADER);
  GLuint fragmentShader = opengl_shader_load(fShaderStr, GL_FRAGMENT_SHADER);
  opengl.programObject = glCreateProgram();

  printf("Linking!\n");
  glAttachShader(opengl.programObject, vertexShader);
  glBindAttribLocation(opengl.programObject, 0, "aVertexPosition");
  glBindAttribLocation(opengl.programObject, 1, "inputTextureCoordinate");

  glAttachShader(opengl.programObject, fragmentShader);
  glBindAttribLocation(opengl.programObject, 2, "uLens");
  glBindAttribLocation(opengl.programObject, 3, "uFov");


  glLinkProgram(opengl.programObject);

  GLint linked;
  glGetProgramiv(opengl.programObject, GL_LINK_STATUS, &linked);
  if (!linked)
  {
    printf("[opengl] Could not link program\n");
  }
  printf("Linking done!\n");


  // Clear the color buffer (UYVY)
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  //Create a texture
  glEnable(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE0);
  GLint texUnitLoc = glGetUniformLocation(opengl.programObject, "videoFrame");
  glUniform1i(texUnitLoc, 1);
  glBindTexture(GL_TEXTURE_2D, 1);

  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

  // Set the viewport
  glViewport(0, 0, MT9F002_OUTPUT_WIDTH/2, MT9F002_OUTPUT_HEIGHT);
  glClear(GL_COLOR_BUFFER_BIT);

  return TRUE;
}

/**
 * Clean up all the initialized OpenGL stuff
 */
void opengl_free(void) {
  eglTerminate(opengl.display);
}

/**
 * Create a new shader of a certain type based on the shader source.
 */
GLuint opengl_shader_load(const char *shaderSrc, GLenum type)
{
  GLuint shader;
  GLint compiled;

  // Create the shader object
  shader = glCreateShader(type);
  if (shader == 0) {
    printf("[opengl] Failed to create shader of type %d\n", type);
    return 0;
  }

  // Load the shader source
  glShaderSource(shader, 1, &shaderSrc, NULL);

  // Compile the shader
  glCompileShader(shader);

  // Check the compile status
  glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
  if (!compiled) {
    GLint infoLen = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);

    if (infoLen > 1) {
      char *infoLog = malloc(sizeof(char) * infoLen);
      glGetShaderInfoLog(shader, infoLen, NULL, infoLog);
      printf("[opengl] Error compiling shader:\n%s\n", infoLog);
      free(infoLog);
    }
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}
