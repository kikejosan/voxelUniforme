#include "BBox.h"
#include "glm/vec3.hpp" 

BBox::BBox() : objects2()
{
	this->Cmax = glm::vec3(-9999999.9,-9999999.9,-9999999.9);
	this->Hmin = glm::vec3(9999999.9, 9999999.9, 9999999.9);

}
//BBox::BBox(glm::vec3 myMin, glm::vec3 myMax, ObjectList objects){
//	this->Cmax = myMax;
//	this->Hmin = myMin;
//	this->objects = objects;
//	La formula de la diagonal de un cubo es la siguiente: D = sqrt(3)*a
//}
BBox::BBox(glm::vec3 myMin, glm::vec3 myMax) : objects2(){
	this->Cmax = myMax;
	this->Hmin = myMin;
	//La formula de la diagonal de un cubo es la siguiente: D = sqrt(3)*a

}
