/* object.h
 *
 * Interaccion y Visualizacion de la Informacion.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */

#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "glm/vec3.hpp" // glm::vec3

#include "shadinfo.h"
#include "material.h"
//#include "BBox.h"

class BBox;

/****************/
/* Object class */
/****************/

class Object {

public:

  /* Computes the nearest intersection point between a ray and the object, 
     in the direction of the ray. Returns the t value of that point if it
     is positive, otherwise returns 0. */
  virtual float NearestInt(const glm::vec3& pos, const glm::vec3& dir) = 0;

  /* Returns the color of the object at the intersection point with the ray */
  virtual glm::vec3 Shade(ShadingInfo &shadInfo) = 0;

  /* Draw the object with OpenGL */
  virtual void Draw() = 0;

  /* Get Bounding Box from the object selected  */
  virtual BBox GetBox()= 0;
  virtual glm::vec3 getCenter() = 0;
  virtual float getRadius() = 0;

  Material *pMaterial;
};


/****************/
/* Sphere class */
/****************/

class Sphere : public Object {

public:

  /* Constructor */
  Sphere(const glm::vec3& Center, float Radius, Material *pMat);

  /* Inherited functions that are implemented in this class */
  virtual float NearestInt(const glm::vec3& pos, const glm::vec3& dir);
  virtual glm::vec3 Shade(ShadingInfo &shadInfo);
  virtual void Draw();
  virtual BBox GetBox();
  virtual glm::vec3 getCenter();
  virtual float getRadius();


//private:

  glm::vec3 center;  // Center coordinates
  float radius;      // Radius
};


#endif  // !defined _OBJECT_H_
