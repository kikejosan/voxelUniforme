/* raytracing.cpp
 *
 * Interaccion y Visualizacion de la Informacion.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */

/****************/
/* Header files */
/****************/

/* General */
#include <iostream> 
#include <stdio.h>
#include <stdlib.h>

/* Microsoft Visual C++ */
#include <windows.h>

/* OpenGL */
#include "GL/glew.h"  /* OpenGL Extension Wrangler */
#include "GL/glut.h"  /* OpenGL Utility Toolkit */

/* GLM */
#include "glm/vec3.hpp" // glm::vec3
#include "glm/mat4x4.hpp" // glm::mat4
#include "glm/gtc/matrix_transform.hpp" // glm::translate, glm::rotate, glm::scale, glm::lookAt, glm::perspective

#include "matrices.h"

#include "shadinfo.h"
#include "object.h"
#include "objectlist.h"
#include "light.h"
#include "lightlist.h"
#include "world.h"
#include "view.h"
#include "frame.h"
#include "raytracer.h"
#include <math.h>
#include "BBox.h"

/*******************/
/* Data structures */
/*******************/

/* Variables related to Ray Tracing method */
RayTracer raytracer;

bool raytracing = false;

/* Window size */
int width;   // Width
int height;  // Height



/* Array of characters */
char buffer[256];


/***********************/
/* Function prototypes */
/***********************/

void init(void);
void output(GLfloat x, GLfloat y, char* text);
void drawAxes(GLfloat length);
void drawSpheres(void);
void drawHUD(void);
void RayTracing(void);
void redraw(void);
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);


//*******************************************************************

void
init(void)
{  
  glm::mat4 lookatMat;

  glClearColor(raytracer.world.bgcolor.x, raytracer.world.bgcolor.y, raytracer.world.bgcolor.z, 0.0);

  glEnable(GL_DEPTH_TEST); // Enable depth buffering

  glMatrixMode(GL_MODELVIEW);
  //glLoadIdentity();
  //gluLookAt(eye.x, eye.y, eye.z,   
  //          center.x, center.y, center.z,   
  //          up.x, up.y, up.z);
  lookatMat = glm::lookAt(raytracer.world.eye, raytracer.world.center, raytracer.world.up);
  glLoadMatrixf(&lookatMat[0][0]);
}

//*******************************************************************

void
output(GLfloat x, GLfloat y, char* text)
{
  char *p;

  glRasterPos2f(x, y);  
  for (p = text; *p; p++)
    glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *p);
}

//*******************************************************************

void
drawAxes(GLfloat length)
{
  /* Draw three reference axes */
  //glLineWidth(1.0);
  glBegin(GL_LINES);
	
  /* X axis, red */
  glColor3f(1.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(length, 0.0, 0.0);

  /* Y axis, green */
  glColor3f(0.0, 1.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, length, 0.0);

  /* Z axis, blue */
  glColor3f(0.0, 0.0, 1.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, length);
  glEnd(); 
}

//*******************************************************************

void
drawSpheres(void)
{
  Object *optr;

  if (raytracer.world.objects.Length() > 0) {

	  /* Access and draw the objects in the list in sequence */
	  optr = raytracer.world.objects.First();
	  while (optr != NULL) {
		  optr->Draw();
		  optr = raytracer.world.objects.Next();
	  }
  }
}

//*******************************************************************

void
drawHUD(void)
{
  /* Set projection and modelview matrices to draw 2D text */
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, width-1, 0, height-1);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  
  glColor3f(1.0, 1.0, 1.0);
  sprintf_s(buffer, "Press space bar to execute Ray Tracing (and wait!)");
  output(10, 15, buffer);

  /* Restore previous projection and modelview matrices */
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

//*******************************************************************
BBox boundingBoxes(BBox bounding1, BBox bounding2) {
	BBox result;

	for (int i = 0; i < bounding1.Cmax.length(); i++) {
		//CALCULAMOS el vector MAXIMO DEL ENVOLVENTE

		if (bounding1.Cmax[i] >= bounding2.Cmax[i])
			result.Cmax[i] = bounding1.Cmax[i];
		else
			result.Cmax[i] = bounding2.Cmax[i];

		//CALCULAMOS  el vector MINIMO DEL ENVOLVENTE

		if (bounding1.Hmin[i] <= bounding2.Hmin[i])
			result.Hmin[i] = bounding1.Hmin[i];
		else 
			result.Hmin[i] = bounding2.Hmin[i];
	}
	
	////CALCULAMOS el vector MAXIMO DEL ENVOLVENTE
	////Componente x de maximo
	//
	//if (bounding1.Cmax.x >= bounding2.Cmax.x) {
	//	result.Cmax.x = bounding1.Cmax.x;
	//} else {
	//	result.Cmax.x = bounding2.Cmax.x;
	//}
	////fprintf(stdout, "%f %f %f \n", bounding1.Cmax.x, bounding2.Cmax.x, result.Cmax.x);
	////Componente y de maximo
	//if (bounding1.Cmax.y >= bounding2.Cmax.y) {
	//	result.Cmax.y = bounding1.Cmax.y;
	//}
	//else {
	//	result.Cmax.y = bounding2.Cmax.y;
	//}
	////Componente de z de maximo
	//if (bounding1.Cmax.z >= bounding2.Cmax.z) {
	//	result.Cmax.z = bounding1.Cmax.z;
	//}
	//else {
	//	result.Cmax.z = bounding2.Cmax.z;
	//}

	////CALCULAMOS  el vector MINIMO DEL ENVOLVENTE
	////Componente x del minimo
	//if (bounding1.Hmin.x <= bounding2.Hmin.x) {
	//	result.Hmin.x = bounding1.Hmin.x;
	//}else {
	//	result.Hmin.x = bounding2.Hmin.x;
	//}
	////Componente y del minimo
	//if (bounding1.Hmin.y <= bounding2.Hmin.y) {
	//	result.Hmin.y = bounding1.Hmin.y;
	//}else {
	//	result.Hmin.y = bounding2.Hmin.y;
	//}
	////Componente z del minimo
	//if (bounding1.Hmin.z <= bounding2.Hmin.z) {
	//	result.Hmin.z = bounding1.Hmin.z;
	//}else {
	//	result.Hmin.z = bounding2.Hmin.z;
	//}

	/*fprintf(stdout, "bounding1Maximo %f %f %f \n", bounding1.Cmax.x, bounding1.Cmax.y, bounding1.Cmax.z);
	fprintf(stdout, "bounding2Maximo %f %f %f \n", bounding2.Cmax.x, bounding2.Hmin.y, bounding2.Hmin.z);
	fprintf(stdout, "resultante Maximo %f %f %f \n\n", result.Cmax.x, result.Cmax.y, result.Cmax.z);
	*/
	return result;
}
/* Encuentro el voxel que contiene el punto sin recorrerlos todos */
glm::vec3 getCoordVoxel(BBox escena[num][num][num], glm::vec3 punto) {
	glm::vec3 minimo;
	glm::vec3 maximo;
	for (int i = 0; i < num; i++) {
		for (int j = 0; j < num; j++) {
			for (int k = 0; k < num; k++) {
				minimo = escena[i][j][k].Hmin;
				maximo = escena[i][j][k].Cmax;
				if (minimo.x<=punto.x && maximo.x>punto.x &&
					minimo.y <= punto.y && maximo.y > punto.x &&
					minimo.z <= punto.x && maximo.x > punto.z ) {
					return glm::vec3(i,j,k);
				}
			}
		}
	}
	return glm::vec3();

}
/* Encuentro el voxel que contiene un cierto punto sin recorrer todos */
glm::vec3 getCoordVoxel1(BBox escena, glm::vec3 punto) {
	float numerador;
	float denominador;
	glm::vec3 indices;
	for (int i = 0; i < punto.length(); i++) {
		numerador = punto[i] - escena.Hmin[i];
		denominador = (escena.Cmax.x - escena.Hmin.x) / num; 
		indices[i] = floor(numerador / denominador);
	}
	
	return indices;
}
boolean testVoxelEsfera(Object* esfera, BBox caja) {
	float dmin = 0;
	float dmax = 0;
	float a = 0;
	float b = 0;
	int n = 3;
	//Sphere esferita = new Sphere(esfera);
	glm::vec3 center = esfera->getCenter();
	float radio = esfera->getRadius();
	fprintf(stdout, "HOLA TESTING");
	float radioCuadrado = radio * radio;
	
	for (int i = 0; i < caja.Cmax.length(); i++) {
		if (center[i] < caja.Hmin[i])
			dmin += sqrt(center[i] - caja.Hmin[i]);
		else
			if (center[i] > caja.Cmax[i])
				dmin += sqrt(center[i] - caja.Cmax[i]);
	}
	
	return (dmin <= radioCuadrado);
	/*else {
		for (int i = 0; i < caja.Cmax.length(); i++) {
			a = sqrt(center[i] - caja.Hmin[i]);
			b = sqrt(center[i] - caja.Cmax[i]);
			if (a >= b)  dmax += a;
			else dmax += b;

			if (center[i] < caja.Hmin[i]) dmin += a;
			else
				if (center[i] > caja.Cmax[i]) dmin += b;
		}
		return (dmin<= radioCuadrado && radioCuadrado<=dmax);
	}*/
}

void
RayTracing(void)
{
	int numDiv = 3;
	BBox boundingUnitario;
	BBox mainBox;
	
	Object *objeto;
	objeto = raytracer.world.objects.First();

	/*variables de div*/
	float divX;
	float divY;
	float divZ;
	
	BBox matriz3D[num][num][num];
	int n;
	
	
	/* BOUNDING BOX de TODA LA ESCENA */
	
	while (objeto != NULL) {
		boundingUnitario = objeto->GetBox();
		
		fprintf(stdout, "unitarioMAX %f %f %f \n", boundingUnitario.Cmax.x, boundingUnitario.Cmax.y, boundingUnitario.Cmax.z);
		fprintf(stdout, "mainMAX %f %f %f \n", mainBox.Cmax.x, mainBox.Cmax.y, mainBox.Cmax.z);
		fprintf(stdout, "unitarioMIN %f %f %f \n", boundingUnitario.Hmin.x, boundingUnitario.Hmin.y, boundingUnitario.Hmin.z);
		fprintf(stdout, "mainMIN %f %f %f \n", mainBox.Hmin.x, mainBox.Hmin.y, mainBox.Hmin.z);
		mainBox = boundingBoxes(mainBox, boundingUnitario);
		
		fprintf(stdout, "ResultanteMAX %f %f %f \n", mainBox.Cmax.x, mainBox.Cmax.y, mainBox.Cmax.z);
		fprintf(stdout, "ResultanteMIN %f %f %f \n\n", mainBox.Hmin.x, mainBox.Hmin.y, mainBox.Hmin.z);
		objeto = raytracer.world.objects.Next();
	}

	//Dividimos el Bounding Box en partes iguales
	divX = (float)abs(mainBox.Cmax.x-mainBox.Hmin.x) / (float)numDiv;
	divY = (float)abs(mainBox.Cmax.y - mainBox.Hmin.y) / (float)numDiv;
	divZ = (float)abs(mainBox.Cmax.z - mainBox.Hmin.z) / (float)numDiv;
	
	BBox voxel;
	glm::vec3 origenMin(mainBox.Hmin.x, mainBox.Hmin.y, mainBox.Hmin.z);
	glm::vec3 origenMax(mainBox.Cmax.x, mainBox.Cmax.y, mainBox.Cmax.z);
	for (int i = 0; i < numDiv; i++) {
		voxel.Hmin.x = origenMin.x + (divX * (float)i);
		voxel.Cmax.x = origenMax.x + (divX * (float)i);
		for (int j = 0; j < numDiv ; j++) {
			voxel.Hmin.y = voxel.Hmin.y + (divY * (float)j);
			voxel.Cmax.y = origenMax.y + (divY * (float)j);
			for (int z = 0; z < numDiv; z++) {
				voxel.Hmin.z = voxel.Hmin.z + (divZ * (float)z);
				voxel.Cmax.z = origenMax.z + (divZ * (float)z);
				matriz3D[i][j][z] = voxel;
				
			}
		} 
	}
	/* Ubicacion de los objetos en los diferentes voxels */
	
	objeto = raytracer.world.objects.First();
	glm::vec3 coordVoxelMin;
	glm::vec3 coordVoxelMax;
	while (objeto != NULL) {
		boundingUnitario = objeto->GetBox();
		/*coordVoxelMin = getCoordVoxel1(mainBox,boundingUnitario.Hmin);
		coordVoxelMax = getCoordVoxel1(mainBox, boundingUnitario.Cmax)*/;
		coordVoxelMin = getCoordVoxel(matriz3D,boundingUnitario.Hmin);
		coordVoxelMax = getCoordVoxel(matriz3D, boundingUnitario.Cmax);
		for (int x = coordVoxelMin.x; x <= coordVoxelMax.x; x++) {
			for (int y = coordVoxelMin.y; y <= coordVoxelMax.y; y++) {
				for (int z = coordVoxelMin.z; z <= coordVoxelMax.z; z++) {
					fprintf(stdout, "ESTOY EN %d %d %d \n",x,y,z);
					fprintf(stdout, "%f %f %f", coordVoxelMax.x, coordVoxelMax.y, coordVoxelMax.z);
					if (testVoxelEsfera(objeto, matriz3D[x][y][z])) {
						matriz3D[x][y][z].objects2.Add(objeto);
					}
				}
			}
		}
		objeto = raytracer.world.objects.Next();
	}
	

	/* Inicio de la práctica 2 */
  long numObjs = 0, numRays = 0, numTests = 0;
  float pixelWidth, pixelHeight;

  fprintf(stdout, "Executing Ray Tracing\n");
  raytracer.world.ClearStats();

  /* Pixel width and height */
  pixelWidth = (raytracer.view.umax - raytracer.view.umin)/(float)width;
  pixelHeight = (raytracer.view.vmax - raytracer.view.vmin)/(float)height;

  for (int y=0; y<height; y++) {

    for (int x=0; x<width; x++) {

      glm::vec3 dirN, dirU, dirV, dir;
	  glm::vec3 color;

      /* Direction towards the pixel */
      dirN = raytracer.view.n*(raytracer.view.d);
	  dirU = raytracer.view.u*(raytracer.view.umin + x*pixelWidth + pixelWidth/2.0f);
      dirV = raytracer.view.v*(raytracer.view.vmin + y*pixelHeight + pixelHeight/2.0f);
	  dir =  glm::normalize(dirN + dirU + dirV);  // Normalized direction

	  /* Ray trace to compute color for this pixel */
	  color = raytracer.world.Trace(raytracer.world.eye, dir, 1);
	  raytracer.frame.SetPixel(x, y, color);

	  raytracer.world.numPrimRays++;
	}

    fprintf(stdout, "." );
  }

  /* Set projection and modelview matrices to draw using OpenGL */
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, width-1, 0, height-1);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  raytracer.frame.Draw();  // Draw the image in the OpenGL context

  /* Restore previous projection and modelview matrices */
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  /* Print statistics */
  fprintf(stdout, "\n");
  fprintf(stdout, "Number of objects: %d\n", raytracer.world.objects.Length());
  fprintf(stdout, "Number of lights: %d\n", raytracer.world.lights.Length());
  fprintf(stdout, "Maximum depth: %d\n", raytracer.world.maxDepth);
  fprintf(stdout, "Number of primary rays: %d\n", raytracer.world.numPrimRays);
  fprintf(stdout, "Number of shadow rays: %d\n", raytracer.world.numShadRays);
  fprintf(stdout, "Number of reflection rays: %d\n", raytracer.world.numReflRays);
  fprintf(stdout, "Number of refraction rays: %d\n", raytracer.world.numRefrRays);

  fprintf(stdout, "Press space bar to execute the Z-buffer method\n");
}

//*******************************************************************

void
redraw(void)
{
  if (!raytracing) {

    /* Clear frame and depth buffer bits */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* Draw reference axes */
    drawAxes(5.0);  

    /* Draw model */
	drawSpheres();

    /* Draw on-screen information */
    drawHUD();

  } else {

	/* Execute Ray Tracing */
	RayTracing();
  }

  /* Swap back and front buffers */
  glutSwapBuffers();
}

//*******************************************************************

void
reshape(int w, int h)
{
	glm::mat4 projectionMat;

	/* OpenGL */

	/* Viewport is same size as the application window */
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	/* Set the projection matrix as the active one */
	glMatrixMode(GL_PROJECTION);

	/* Set perspective projection parameters. Use the
	perspective function from the GLM library */
	//glLoadIdentity();
	//gluPerspective(raytracer.view.angle,         /* FOV, field of view
	//                                (vertical), in degrees */
	//               (float) w/h,  /* Aspect ratio width:height */
	//               0.1, 10000.0  /* Front face (Z near, hither) and rear face
	//                                (Z far, yon) of the viewing volume */
	//              );
	projectionMat = glm::perspective(
		glm::radians(raytracer.view.angle),  /* FOV, field of view (vertical), in radians */
		(float)w / h,		  /* Aspect ratio width:height */
		0.1f, 10000.0f		  /* Front face (Z near, hither) and rear face
							  (Z far, yon) of the viewing volume */
	);
	glLoadMatrixf(&projectionMat[0][0]);

	/* Restore modelview matrix as the active one */
	glMatrixMode(GL_MODELVIEW);

	/* Ray Casting */

	/* The image is also same size as the application window */
	raytracer.frame.SetSize(w, h);

	/* Perspective projection parameters are the same too */
	raytracer.view.SetPerspective(raytracer.view.angle, (float)w/h);

	/* Save current window size */
	width = w;
	height = h;
}

//*******************************************************************

void 
keyboard(unsigned char key, int x, int y)
{
  switch (key) {
    case ' ':  // Switch between Z-buffer and Ray Casting methods
	  if (!raytracing) {
		glDisable(GL_DEPTH_TEST);
		raytracing = true;
	  } else {
		raytracing = false;
		glEnable(GL_DEPTH_TEST);
	  }
      glutPostRedisplay();
      break;
    case 27:  // Escape key
      exit(0);
  }
}

//*******************************************************************

void
main(int argc, char* argv[])
{
  /* Process the command line and negotiates the start
	 of an OpenGL session with the window system */
  glutInit(&argc, argv);
  
  /* Set the OpenGL display mode for the window that will be created
     next: doble buffer, RGBA colour model, and depth buffer */
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

  /* Load the scene from a NFF file */
  if (argc >= 2) {

	  if (raytracer.LoadFileNFF(argv[1]) == -1) {
		  std::cerr << "Error al cargar fichero NFF" << std::endl;
		  exit(-1);
	  }
  }

  /* Create a window for OpenGL rendering: selects the appropriate visual
     and colourmap, and create an OpenGL context associated to this window */
  glutInitWindowSize(raytracer.frame.GetWidth(), raytracer.frame.GetHeight());
  glutCreateWindow(argv[1]);

  /* Initialize GLEW */
  GLenum err = glewInit();
  if (GLEW_OK != err) {
	  std::cerr << "Error: " << glewGetString(err) << std::endl;
  }

  /* Initialize the OpenGL context */
  init();

  /* Register event handler functions */

  /* Called each time the window needs to be redisplayed */ 
  glutDisplayFunc(redraw);

  /* Called each time the window is moved or resized */
  glutReshapeFunc(reshape);

  /* Called when a key that generates an ASCII code is pressed */
  glutKeyboardFunc(keyboard);

  /* Event loop */
  glutMainLoop();
}
