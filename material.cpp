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


	// Recorremos cada rayo de luz
	while (myLightOpt != NULL) {
		// Obtenemos la proyeccion del rayo con respecto la normal y la diferencia del vector posicion desde 
		//esta  el punto observador
		li = glm::normalize(myLightOpt->position - shadInfo.point);
		ln = glm::dot(li, shadInfo.normal);

		//Para proyectar los rayos de sombra, se recorren todas las esferas y se sacan las intersecciones del rayo hasta el punto
		objetoPEsfera = shadInfo.pWorld->objects.First();
		shadowCoef = glm::vec3(1., 1., 1.);
		while (objetoPEsfera != NULL) {
			//Miramos desde donde nace el rayo (li) y lo que se encuentra en los puntos de detrás (-li) del rayo 
			if (objetoPEsfera->NearestInt(shadInfo.point,li)>TMIN && objetoPEsfera->NearestInt(myLightOpt->position,-li)>TMIN) {
				shadowCoef = shadowCoef * objetoPEsfera->pMaterial->Kt;
			}
			objetoPEsfera = shadInfo.pWorld->objects.Next();
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
