/* Material.cpp
*
* Interaccion y Visualizacion de la Informacion.
*
* Practice 2.
* Ray tracing.
*
* Jose Pascual Molina Masso.
* Escuela Superior de Ingenieria Informatica de Albacete.
*/


#include "glm/glm.hpp" // glm::vec3, glm::dot

#include "Material.h"
#include "light.h"
#include "lightlist.h"
#include "world.h"
#include "limits.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include "list.h"


/* Constructors */
Material::Material(const glm::vec3 &diff) {

	Ka = glm::vec3(0.0, 0.0, 0.0);
	Kd = diff;
	Kdt = glm::vec3(0.0, 0.0, 0.0);
	Ks = glm::vec3(0.0, 0.0, 0.0);
	Kst = glm::vec3(0.0, 0.0, 0.0);
	n = 0;
	Ie = glm::vec3(0.0, 0.0, 0.0);
	Kr = glm::vec3(0.0, 0.0, 0.0);
	Kt = glm::vec3(0.0, 0.0, 0.0);
	ior = 0.0;
}

bool Material::isTrans()
{
	return (Kt != glm::vec3(0.0, 0.0, 0.0));
}

Material::Material(const glm::vec3 &amb, const glm::vec3 &diff, const glm::vec3 &diffTrans,
	const glm::vec3 &spec, const glm::vec3 &specTrans, int shine, const glm::vec3 &emis,
	const glm::vec3 &refl, const glm::vec3 &trans, float index) {

	Ka = amb;
	Kd = diff;
	Kdt = diffTrans;
	Ks = spec;
	Kst = specTrans;
	n = shine;
	Ie = emis;
	Kr = refl;
	Kt = trans;
	ior = index;
}

/* Implements the global illumination model */
glm::vec3 Material::Shade(ShadingInfo &shadInfo)
{
	glm::vec3 color(0.0, 0.0, 0.0), V;
	float VdotN, ratio;
	bool isTrans;

	V = -shadInfo.rayDir;
	VdotN = glm::dot(V, shadInfo.normal);
	isTrans = (Kt != glm::vec3(0.0, 0.0, 0.0));
	if (VdotN < 0) {

		// The viewer stares at an interior or back face of the object,
		// we will only illuminate it if material is transparent
		if (isTrans) {
			shadInfo.normal = -shadInfo.normal;  // Reverse normal
			VdotN = -VdotN;
			ratio = 1.0 / ior; // Ray always comes from vacuum (hollow objects)
			//ratio = ior;  // Use this instead for solid objects
		}
		else
			return color;
	}
	else {

		// The viewer stares at a front face of the object
		if (isTrans)
			ratio = 1.0 / ior; // Ray comes from vacuum
	}

	// To do ...
	/*Variable de shadInfo:
		- Normal
		- Point
		- Puntero A WORLD (Contiene las luces)
		- rayDir
		- rayPos
	Las variables de los coeficientes estan en MATERIAL.
	*/



	// Componentes LOCALES
	glm::vec3 IdR(0, 0, 0); //difusa reflexion
	glm::vec3 IdT(0, 0, 0); //difusa transmision
	glm::vec3 IsR(0, 0, 0); //especular reflexion
	glm::vec3 IsT(0, 0, 0); //especular transmision
	glm::vec3 R(0, 0, 0); //R obtención vector refractado
	glm::vec3  li(0, 0, 0); //Rayo incidente de la object list
	glm::vec3 T(0, 0, 0); // Vector T al chocar con el rayo incidente contra el cuerpo en local
	float ln = 0.0; // producto punto L . N
	float RiV = 0.0; //Componente del sumatorio R*V en local
	float b = 0.0; // Auxiliar para calculo de T 
	float cosFi = 0.0;
	float radical = 0.0;

	// Componentes GLOBALES
	glm::vec3 IaDRT(0, 0, 0); //difusa rflexion y transmision
	glm::vec3 IaER(0, 0, 0); // especular reflexion
	glm::vec3 IaET(0, 0, 0); // especular transmision
	glm::vec3 RGlobal(0, 0, 0); // R producida en la global
	float RiVG = 0.0; //Componente del sumatorio R*V en global
	float TiV = 0.0; // Componente del sumatorio R*V en global
	glm::vec3 TGlobal(0, 0, 0); // Vector T al chocar con el rayo incidente contra el cuerpo en local
	glm::vec3  Ilia(0, 0, 0); // Componente intensidad de la luz ambiental
	glm::vec3 Ir(0, 0, 0); // Ir procedente de la reflexión global al chocar entre cuerpos
	glm::vec3 It(0, 0, 0); // It procedente de la transmisión de luz entre cuerpos
	float bGlobal = 0.0;
	float cosFiGlobal = 0.0;
	float radicalGlobal = 0.0;


	//Coeficiente de sombra y Intensidades finales ponderadas a cada rayo
	glm::vec3 shadowCoef(1., 1., 1.);
	glm::vec3 intensityId(0, 0, 0);
	glm::vec3 intensityIs(0, 0, 0);
	glm::vec3 intensityIa(0, 0, 0);

	// Inicialización de objetos para el tratamiento especifico de la luz
	Object *objetoPEsfera;
	Light *myLightOpt = shadInfo.pWorld->lights.First();

	std::vector<glm::vec3> puntosInicioFin;
	std::vector<glm::vec3> listaIndices;
	glm::vec3 indiceX;
	BBox voxel;
	glm::vec3 inicio;
	glm::vec3 fin;
	// Recorremos cada rayo de luz
	while (myLightOpt != NULL) {
		// Obtenemos la proyeccion del rayo con respecto la normal y la diferencia del vector posicion desde 
		//esta  el punto observador
		li = glm::normalize(myLightOpt->position - shadInfo.point);
		ln = glm::dot(li, shadInfo.normal);

		
		//Para proyectar los rayos de sombra, se recorren todas las esferas y se sacan las intersecciones del rayo hasta el punto
		shadowCoef = glm::vec3(1., 1., 1.);

		/* AMANATIDES :::::: Lista de voxels e iterarlo con este bucle */
		/* Obtenemos el punto de inicio del rayo en la escena y el fin, comparando las distancias t con respecto los 6 planos del boundingbox de la escena */
		puntosInicioFin = sacarInicioFin(li, myLightOpt->position,shadInfo.pWorld->matriz3D[0][0][0].Hmin, shadInfo.pWorld->matriz3D[num-1][num-1][num-1].Cmax);
		
		/* Obtenemos la lista de indices de los voxeles por donde pasa el rayo */
		inicio = puntosInicioFin[0];
		fin = puntosInicioFin[1];
		fprintf(stdout, "INICIO FIN %f  %f \n", inicio.x, fin.x);
		listaIndices = this->getListaIndices(inicio,fin);
		
		/* Iteramos sobre la lista de indices para comprobar los puntos t minimos por donde pasa el rayo interseccion */
		
		for (int i = 0; i < listaIndices.size(); i++) {
			/* Eligimos el voxel correspondiente */
			voxel = shadInfo.pWorld->matriz3D[(int)listaIndices[i].x][(int)listaIndices[i].y][(int)listaIndices[i].z];
			/* Iteramos sobre los objetos de ese voxel */
			objetoPEsfera = voxel.objects2.First();
			while (objetoPEsfera != NULL) {
				//Miramos desde donde nace el rayo (li) y lo que se encuentra en los puntos de detrás (-li) del rayo 
				if (objetoPEsfera->NearestInt(shadInfo.point, li) > TMIN && objetoPEsfera->NearestInt(myLightOpt->position, -li) > TMIN) {
					shadowCoef = shadowCoef * objetoPEsfera->pMaterial->Kt;
					i = INT_MAX;
				}
				objetoPEsfera = voxel.objects2.Next();
			}
		}
		
		
		/* Multiplico el coeficiente de sombra por las intensidades de la luz*/
		intensityId = myLightOpt->Id * shadowCoef;
		intensityIs = myLightOpt->Is * shadowCoef;
		intensityIa = myLightOpt->Ia * shadowCoef;
		

		// Si la luz nace desde fuera de las esferas 
		if (ln > 0) {
			//Difusa reflexión
			//Sumatorio
			IdR = IdR + (intensityId*ln);

			//Especular reflexión
			R = glm::normalize(((2 * ln)*shadInfo.normal) - li);
			RiV = glm::dot(R,glm::normalize(V));
			RiV = pow(RiV,n);

			//Sumatorio
			IsR = IsR +(RiV * intensityIs);
			
		}else{	
			//La esfera es transparente y por tanto actuan varios fenomenos que 
			//hacen que la luz varie en su salida del cuerpo 
				//Difusa transmisión
			if (ln < 0 && isTrans) {
				//Sumatorio
				IdT = IdT + intensityId*(-ln);

				//Especular transmisión
				cosFi = glm::dot(-shadInfo.normal, li);
				radical = 1 + (pow(ratio, 2)*(pow(cosFi, 2) - 1));
				// Así evitamos que la raiz cuadrada se pueda hacer con éxito
				if (radical >= 0) {
					//b = (ratio * cosFi) - sqrt(1 + (pow(ratio, 2)*(pow(cosFi, 2) - 1)));
					b = (ratio * cosFi) - sqrt(radical);
					T = glm::normalize((ratio * -li) + (b * shadInfo.normal));
					TiV = pow(glm::dot(T, V), n);
					IsT = IsT + TiV * intensityIs;
				}
			}
		}
		//Calculo de la ambiental
		Ilia = Ilia + intensityIa;
		//Pasamos al siguiente rayo incidente
		myLightOpt = shadInfo.pWorld->lights.Next();
	}

	//Calculo de las componentes locales con respecto sus coeficientes correspondientes
	//locales
	IdR = Kd * IdR;
	IdT = Kdt * IdT;
	IsR = Ks * IsR;
	IsT = Kst * IsT;
	//globales
	IaDRT = Ka * Ilia;


	/*GLOBAL*/
	// Mientras que el rayo no haya llegado al limite establecido
	if (shadInfo.pWorld->maxDepth > shadInfo.depth) {
		// ¿Es posible que haya REFLEXION?
		if (Kr != glm::vec3(0.,0.,0.) ) {
			// Especular Reflexion global
			RGlobal = glm::normalize(2 * VdotN * shadInfo.normal - V);
			Ir = shadInfo.pWorld->Trace(shadInfo.point, RGlobal, shadInfo.depth + 1);
			IaER = Kr * Ir;
		}
		// En caso de haber un material transparente HABRA TRANSMISION DE LUZ TRAS LOS CUERPOS
		if (isTrans) {
			// Especular Transmision global
			cosFiGlobal = VdotN;
			radicalGlobal = 1 + (pow(ratio, 2)*(pow(cosFiGlobal, 2) - 1));
			// Así evitamos que la raiz cuadrada se pueda hacer con éxito
			if (radical >= 0) {
				bGlobal = (ratio * cosFiGlobal) - sqrt(radicalGlobal);
				TGlobal = ratio * -V + bGlobal * shadInfo.normal;
				It = shadInfo.pWorld->Trace(shadInfo.point, TGlobal, shadInfo.depth + 1);
				IaET = Kt * It;
			}
		}
	}

	//El color final será la suma de las diferentes elementos de las
	// componentes locales y finales
	color = color + IaDRT + IaER + IaET + IdR + IdT + IsR + IsT;;

	return color;
}

std::vector<glm::vec3> Material::sacarInicioFin(glm::vec3 direccion, glm::vec3 posicion, glm::vec3 minimoMundo, glm::vec3 maximoMundo) {
	glm::vec3 rayPos = posicion;
	glm::vec3 rayDir = direccion;
	
	glm::vec3 minimo = minimoMundo;
	glm::vec3 maximo = maximoMundo;

	//Obtenemos 3 puntos de cada plano para sacar las ecuaciones del plano y las normales
	glm::vec3 normales[6];

	glm::vec3 puntos[18];
	glm::vec3 punto1min = glm::vec3(minimo.x, minimo.y, maximo.z);
	glm::vec3 punto1max = maximo;
	glm::vec3 punto1otro = glm::vec3(minimo.x, maximo.y, maximo.z);
	
	
	glm::vec3 punto2min = glm::vec3(maximo.x, minimo.y, minimo.z);
	glm::vec3 punto2max = maximo;
	glm::vec3 punto2otro = glm::vec3(maximo.x, minimo.y, maximo.z);
	
	
	glm::vec3 punto3min = glm::vec3(minimo.x, maximo.y, minimo.z);
	glm::vec3 punto3max = maximo;
	glm::vec3 punto3otro = glm::vec3(maximo.x, maximo.y, minimo.z);
	
	
	glm::vec3 punto4min = minimo;
	glm::vec3 punto4max(maximo.x, minimo.y, maximo.z);
	glm::vec3 punto4otro(minimo.x, maximo.y, maximo.z);
	
	
	glm::vec3 punto5min = minimo;
	glm::vec3 punto5max(maximo.x, maximo.y, minimo.z);
	glm::vec3 punto5otro(minimo.x,maximo.y,minimo.z);
	
	glm::vec3 punto6min = minimo;
	glm::vec3 punto6max(minimo.x, maximo.y, maximo.z);
	glm::vec3 punto6otro(minimo.x,minimo.y,maximo.z);
	
	puntos[0] = punto1min;
	puntos[1] = punto2min;
	puntos[2] = punto3min;
	puntos[3] = punto4min;
	puntos[4] = punto5min;
	puntos[5] = punto6min;

	normales[0] = glm::cross(punto1otro - punto1min, punto1max - punto1min);
	normales[1] = glm::cross(punto2otro - punto2min, punto2max - punto2min);
	normales[2] = glm::cross(punto3otro - punto3min, punto3max - punto3min);
	normales[3] = glm::cross(punto4otro-punto4min, punto4max-punto4min);
	normales[4] = glm::cross(punto5otro - punto5min, punto5max - punto5min);
	normales[5] = glm::cross(punto6otro - punto6min, punto6max - punto6min);

	//Con las normales y los puntos obtenemos las ts a través de las ecuaciones parametricas de la recta y la ecuacion generald el planoç
	// ¡Ojo! Las ecuaciones de los planos son de planos infinitos, luego controlaremos las cotas con un "if"
	std::vector<glm::vec3> lista;

	std::vector<float> ts;
	float numerador = 0.0;
	float denominador = 0.0;
	glm::vec3 puntico;
	double t;
	for (int i = 0; i < normales->length();i++) {
		numerador = ((normales[i].x*(puntos[i].x-rayPos.x))+
			(normales[i].y*(puntos[i].y*rayPos.y)) +
			(normales[i].z*(puntos[i].z*rayPos.z))
			);
		denominador = (normales[i].x*rayDir.x + normales[i].y*rayDir.y + normales[i].z*rayDir.z);
		fprintf(stdout, "NUMERADOR y DENOMINADOR %f  %f \n",numerador,denominador);
		t = (double)numerador / (double)denominador;
		fprintf(stdout, "t %f \n", t);
		puntico.x = rayPos.x + t * rayDir.x;
		puntico.y = rayPos.y + t * rayDir.y;
		puntico.z = rayPos.z + t * rayDir.z;
		//lista.push_back(puntico);
		// Comprobamos que el punto está en uno de los planos ACOTADOS del mainbox
		if ((puntico.x <= maximo.x && puntico.x >= minimo.x) && (puntico.z == minimo.z || puntico.z == maximo.z) &&
			((puntico.y <= maximo.y && puntico.z >= minimo.z) && (puntico.x == minimo.x || puntico.x == maximo.x)) &&
			(puntico.x <= maximo.x && puntico.z >= minimo.z) && (puntico.y == minimo.y || puntico.y == maximo.y)
			){
		
			lista.push_back(puntico);
		}
	}
	
	return lista;
}
std::vector<glm::vec3> Material::getListaIndices(glm::vec3 origen, glm::vec3 final) {
	std::vector<glm::vec3> indices;
	glm::vec3 voxel_actual(floor(origen[0] / 1),floor(origen[1] / 1), floor(origen[2] / 1));
	glm::vec3 voxel_final(floor(final[0] / 1), floor(final[1] / 1), floor(final[2] / 1));
	
	glm::vec3 rayo = origen - final;
	
	double stepX = (rayo[0] >= 0) ? 1 : -1; 
	double stepY = (rayo[1] >= 0) ? 1 : -1; 
	double stepZ = (rayo[2] >= 0) ? 1 : -1; 

	double nextVoxelVecino_x = (voxel_actual[0] + stepX) * 1; 
	double nextVoxelVecino_y = (voxel_actual[1] + stepY) * 1;
	double nextVoxelVecino_z = (voxel_actual[2] + stepZ) * 1;

	double tMaxX = (rayo[0] != 0) ? (nextVoxelVecino_x - origen[0]) / rayo[0] : DBL_MAX;
	double tMaxY = (rayo[1] != 0) ? (nextVoxelVecino_y - origen[1]) / rayo[1] : DBL_MAX;
	double tMaxZ = (rayo[2] != 0) ? (nextVoxelVecino_z - origen[2]) / rayo[2] : DBL_MAX;

	double tDeltaX = (rayo[0] != 0) ? 1 / rayo[0] * stepX : DBL_MAX;
	double tDeltaY = (rayo[1] != 0) ? 1 / rayo[1] * stepY : DBL_MAX;
	double tDeltaZ = (rayo[2] != 0) ? 1 / rayo[2] * stepZ : DBL_MAX;

	glm::vec3 diferencia;

	bool neg_ray = false;

	if (voxel_actual[0] != voxel_final[0] && rayo[0] < 0) { diferencia[0]--; neg_ray = true; }
	if (voxel_actual[1] != voxel_final[1] && rayo[1] < 0) { diferencia[1]--; neg_ray = true; }
	if (voxel_actual[2] != voxel_final[2] && rayo[2] < 0) { diferencia[2]--; neg_ray = true; }

	if (neg_ray) {
		voxel_actual += diferencia;
		indices.push_back(voxel_actual);
	}

	while (voxel_final != voxel_actual) {
		if (tMaxX < tMaxY) {
			if (tMaxX < tMaxZ) {
				voxel_actual[0] += stepX;
				tMaxX += tDeltaX;
			}
			else {
				voxel_actual[2] += stepZ;
				tMaxZ += tDeltaZ;
			}
		}
		else {
			if (tMaxY < tMaxZ) {
				voxel_actual[1] += stepY;
				tMaxY += tDeltaY;
			}
			else {
				voxel_actual[2] += stepZ;
				tMaxZ += tDeltaZ;
			}
		}
		indices.push_back(voxel_actual);
	}
	return indices;
}


