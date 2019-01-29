#ifndef _BBOX_H_

#define _BBOX_H_
#include "glm/vec3.hpp" 
#include "objectlist.h"

class BBox{
public:
	glm::vec3 id;
	glm::vec3 A;
	glm::vec3 B;
	glm::vec3 Cmax;
	glm::vec3 D;
	glm::vec3 E;
	glm::vec3 F;
	glm::vec3 G;
	glm::vec3 Hmin;
	BBox();
	ObjectList objects2;
	//BBox(glm::vec3 myMin, glm::vec3 myMax, ObjectList objects);
	BBox(glm::vec3 myMin, glm::vec3 myMax);
};


#endif  