 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "rayAccelerator.h"
#include "maths.h"
#include "macros.h"

//#define DEBUG

//Enable OpenGL drawing.  
bool drawModeEnabled = false;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene

#define MAX_DEPTH 4  //number of bounces

#define CAPTION "Whitted Ray-Tracer"
#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

unsigned int FrameCount = 0;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];


// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;

Grid* grid_ptr = NULL;
BVH* bvh_ptr = NULL;
accelerator Accel_Struct = NONE;

int RES_X, RES_Y;

int WindowHandle = 0;

bool skybox_flg = false;



/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");
	
	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs

void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);
	
// unbind the VAO
	glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;
	glClear(GL_COLOR_BUFFER_BIT);

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}


// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top, 
			float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'r':
			camX = Eye.x;
			camY = Eye.y;
			camZ = Eye.z;
			r = Eye.length();
			beta = asinf(camY / r) * 180.0f / 3.14f;
			alpha = atanf(camX / camZ) * 180.0f / 3.14f;
			break;

		case 'c':
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
			printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
			break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}

bool getNearestIntersection(Ray ray, float& nearestHit, Object*& nearestObject)
{
	nearestHit = FLT_MAX;
	nearestObject = NULL;

	for (int i = 0; i < scene->getNumObjects(); i++) {
		Object* object = scene->getObject(i);
		float dist;

		if (object->intercepts(ray, dist)) {
			if (dist < nearestHit) {
				nearestHit = dist;
				nearestObject = object;
			}
		}
	}

	if (nearestObject == NULL) {
		return false;
	}

	return true;
}

/////////////////////////////////////////////////////YOUR CODE HERE///////////////////////////////////////////////////////////////////////////////////////

bool getShadowHit(Ray ray, Object* hitObj, float t) {
	bool shadowHit = false;
	Vector hitPoint = Vector(0.0f, 0.0f, 0.0f);

	switch (scene->GetAccelStruct()) 
	{
	case (NONE): {
			ray.direction.normalize();
			shadowHit = getNearestIntersection(ray, t, hitObj);
			break;
		};
	case (GRID_ACC): {
			ray.direction.normalize();
			shadowHit = grid_ptr->Traverse(ray);
			break;
		};
	case (BVH_ACC): {
			shadowHit = bvh_ptr->Traverse(ray);
			break;
		};
	}

	return shadowHit;
}

float getShadowCoeff(int lightNum, Vector hitPoint, Vector bias)
{
	std::vector<Ray> shadowRays;
	for (float k = -0.2; k <= 0.2; k += 0.1)
		for (float j = -0.2; j <= 0.2; j += 0.1)
		{
			Vector part_light = scene->getLight(lightNum)->position + Vector(k, j, 0.);
			Vector new_hitpoint = hitPoint + bias;
			Ray shadowRay = Ray(new_hitpoint, (part_light - hitPoint).normalize());

			shadowRays.push_back(shadowRay);
		}

	float numShadowRays = shadowRays.size();
	float litCounter = 0.;

	for (const Ray& shadowRay : shadowRays)
	{
		float nearestShadowHit = FLT_MAX;
		Object* nearestShadowObject = NULL;
		if (!getShadowHit(shadowRay, nearestShadowObject, nearestShadowHit))
			litCounter++;
	}

	float proportion = litCounter / numShadowRays;
	return proportion;
}

Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	Color color = Color();

	float nearestHit = FLT_MAX;
	Object* nearestObject = NULL;
	Vector hitPoint = Vector(0.0f, 0.0f, 0.0f);

	switch (scene->GetAccelStruct()) 
	{
		case (NONE): {
			if (!getNearestIntersection(ray, nearestHit, nearestObject)) {
				return scene->GetBackgroundColor();
			}
			hitPoint = ray.origin + ray.direction * nearestHit;
			break;
		};
		case (GRID_ACC): {
			if (!grid_ptr->Traverse(ray, &nearestObject, hitPoint)) {
				if (skybox_flg) {
					return color = scene->GetSkyboxColor(ray);
				}
				else {
					return color = (scene->GetBackgroundColor());
				}
			}
			break;
		};
		case (BVH_ACC): {
			bool hit = bvh_ptr->Traverse(ray, &nearestObject, hitPoint);
			if (!hit) {
				if (skybox_flg) {
					return color = scene->GetSkyboxColor(ray);
				}
				else {
					return color = (scene->GetBackgroundColor());
				}
			}
			break;
		};
	}

	if (nearestObject == NULL) {
		return scene->GetBackgroundColor();
	}

	Vector geomNormal = nearestObject->getNormal(hitPoint);

	bool isInside = false;
	Vector normal = geomNormal;

	if (ray.direction * geomNormal > 0)
	{
		isInside = true;
		normal = geomNormal * (-1);
	}

	Vector bias = normal * EPSILON; 

	//Calculation of direct illumination only if outside
	if (!isInside) {

		for (int i = 0; i < scene->getNumLights(); i++) 
		{
			
			int spp = scene->GetSamplesPerPixel();

			Vector lightVector = Vector(0.0, 0.0, 0.0);
			float shadowCoeff = 0.0;;
			float intensity = 0.0;

			if (spp != 0)
			{
				float lightSize = 1.0;
				Vector r = Vector(rand_float() - lightSize / 2.0, rand_float() - lightSize / 2.0, 0.0);
				lightVector = scene->getLight(i)->position - hitPoint + r;
				intensity = lightVector * normal;

				shadowCoeff = 1.0;
			}
			else
			{
				lightVector = scene->getLight(i)->position - hitPoint;
				intensity = lightVector * normal;

				shadowCoeff = getShadowCoeff(i, hitPoint, bias);
			}

			if (intensity > 0.0)
			{
				bool shadowHit = false;
				if (spp != 0)
				{
					Ray shadowRay(hitPoint + bias, lightVector);
					float nearestShadowHit = FLT_MAX;
					Object* nearestShadowObject = NULL;
					shadowHit = getShadowHit(shadowRay, nearestShadowObject, nearestShadowHit);
				}

				if (!shadowHit && shadowCoeff > EPSILON) {
					// no object between the hit point and the light 

					lightVector.normalize();
					Material* material = nearestObject->GetMaterial();
					Vector hitpointToEye = ray.direction * -1;
					Vector halfway = (lightVector + hitpointToEye).normalize(); 

					Color diffuseColor = material->GetDiffColor();
					float diffuse = material->GetDiffuse() * std::fmax(lightVector * normal, 0.0f);

					Color specColor = material->GetSpecColor();
					float specular = material->GetSpecular() *
						std::pow(std::fmax(normal * halfway, 0.0f), material->GetShine());

					color += (diffuseColor * diffuse + specColor * specular) * shadowCoeff;
				}
			}
		}
	}

	if (depth >= MAX_DEPTH) return color;

	//reflection with a reflective non-dielectric
	if (nearestObject->GetMaterial()->GetReflection() > EPSILON && nearestObject->GetMaterial()->GetTransmittance() == 0.0) {
		Vector reflectionDir = (ray.direction - normal * 2 * (ray.direction * normal)).normalize();

		float roughness = 0.1f;

		reflectionDir = reflectionDir + rnd_unit_sphere() * roughness;
		Ray refRay = Ray(hitPoint + bias, reflectionDir);
		Color reflectionColor = rayTracing(refRay, depth + 1, ior_1) * nearestObject->GetMaterial()->GetSpecColor();

		color += reflectionColor * nearestObject->GetMaterial()->GetReflection();
	}

	//dielectric
	if (nearestObject->GetMaterial()->GetTransmittance() == 1.0 ) {

		float kr = 1;  //in case of total reflection

		Vector vt = normal * (ray.direction * (-1) * normal) - ray.direction * (-1);
		float sin_i = vt.length();
		float cos_i = sqrtf(1 - (sin_i * sin_i));

		Vector t = vt.normalize();
		float eta_i = ior_1;
		float eta_t = nearestObject->GetMaterial()->GetRefrIndex();

		if (isInside) {
			eta_t = 1.0;
		};

		float sin_t = eta_i / eta_t * sin_i;
		float cos_t = sqrtf(1 - (sin_t * sin_t));

		if (sin_t * sin_t < 1.0) {   //not total reflection
			float R0 = powf((eta_i - eta_t) / (eta_i + eta_t), 2.0);

			if (eta_i > eta_t) {
				kr = R0 + (1 - R0) * powf((1 - cos_t), 5.0);
			}
			kr = R0 + (1 - R0) * powf((1 - cos_i), 5.0);
		}
		float kt = 1 - kr;

		Color  refractedColor = Color();
		if (kt > 0.0) {
			Vector refractedDir = t * sin_t + normal * ((-1) * cos_t); 
			refractedDir.normalize();
			Ray refrRay = Ray(hitPoint - bias, refractedDir);
			refractedColor = rayTracing(refrRay, depth + 1, eta_t);
		}

		Vector reflectionDir = (ray.direction - normal * 2 * (ray.direction * normal)).normalize();

		float roughness = 0.1f;
		
		reflectionDir = reflectionDir + rnd_unit_sphere() * roughness;
		reflectionDir.normalize();
		Ray reflRay = Ray(hitPoint + bias, reflectionDir);
		Color reflectionColor = rayTracing(reflRay, depth + 1, ior_1) * nearestObject->GetMaterial()->GetSpecColor();

		color += reflectionColor * kr + refractedColor * kt;
	}

	return color;
}

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	set_rand_seed(time(NULL) * time(NULL));

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}
	int total = RES_X * RES_Y;

	int n = scene->GetSamplesPerPixel();

#ifdef DEBUG
	int total_steps = RES_X * RES_Y * n * n;
	int total_no_spp = RES_X * RES_Y;
#endif
	int current = 0;
	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color;

			int n = scene->GetSamplesPerPixel();
			if(n!=0)
			{
			for (int p = 0; p < n; p++) {
				for (int q = 0; q < n; q++) {

						Vector subpixel;  //viewport coordinates
						subpixel.x = x + (p+ rand_float()) /n;
						subpixel.y = y + (q + rand_float()) / n;
						subpixel.z = -1 * scene->GetCamera()->GetPlaneDist();

						Vector lens_sample = rnd_unit_disk() * scene->GetCamera()->GetAperture() / 2.0f;

						Ray subray = scene->GetCamera()->PrimaryRay(lens_sample, subpixel);
						color += rayTracing(subray, 1, 1.0).clamp();

				}
			}
			
			color = color / (float)pow(n, 2);
			}
			else
			{
				Vector pixel;  //viewport coordinates
				pixel.x = x + 0.5f;
				pixel.y = y + 0.5f;
				// Without jittering antialiasing

				Ray ray = scene->GetCamera()->PrimaryRay(pixel);   //function from camera.h
				color = rayTracing(ray, 1, 1.0).clamp();
			}

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();
			}

			int current = y * RES_X + x * y;
			if (current != 0 && current % 250000 == 0) {
				//std::cout << "color(" << color.r() << ", " << color.g() << ", " << color.b() << ")" << std::endl;
				//std::cout << "[DEBUG] Progress: " << current << "/" << total << std::endl;
			}
		}

	}

	//std::cout << "[DEBUG] finished" << std::endl;
	if (drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}


///////////////////////////////////////////////////////////////////////  SETUP     ///////////////////////////////////////////////////////

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}

void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
}

void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			cout << "Input the Scene Name: ";
			cin >> input_user;
			strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
			strcat_s(scene_name, sizeof(scene_name), input_user);

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
		printf("Scene loaded.\n\n");
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}


	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);

	Accel_Struct = scene->GetAccelStruct();   //Type of acceleration data structure

	if (Accel_Struct == GRID_ACC) {
		grid_ptr = new Grid();
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		grid_ptr->Build(objs);
		printf("Grid built.\n\n");
	}
	else if (Accel_Struct == BVH_ACC) {
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();
		bvh_ptr = new BVH();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		bvh_ptr->Build(objs);
		printf("BVH built.\n\n");
		printf("num of objects: %d\n", bvh_ptr->getNumObjects());
	}
	else
		printf("No acceleration data structure.\n\n");

	unsigned int spp = scene->GetSamplesPerPixel();
	if (spp == 0)
		printf("Whitted Ray-Tracing\n");
	else
		printf("Distribution Ray-Tracing\n");

}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int 
		ch;
	if (!drawModeEnabled) {

		do {
			init_scene();

			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X*RES_Y * sizeof(float);
		size_colors = 3 * RES_X*RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
		memset(colors, 0, size_colors);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////