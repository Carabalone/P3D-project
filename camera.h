#ifndef CAMERA_H
#define CAMERA_H

#include <cmath>
#include <stdio.h>
using namespace std;

#include "vector.h"
#include "ray.h"
#include "maths.h"

class Camera
{

private:
	Vector eye, at, up; 
	float fovy, vnear, vfar, plane_dist, focal_ratio, aperture;
	float w, h;
	int res_x, res_y;
	Vector u, v, n;

public:
	Vector GetEye() { return eye; }
	int GetResX()  { return res_x; }
    int GetResY()  { return res_y; }
	float GetFov() { return fovy; }
	float GetPlaneDist() { return plane_dist; }
	float GetFar() {return vfar; }
	float GetAperture() { return aperture; }

    Camera( Vector from, Vector At, Vector Up, float angle, float hither, float yon, int ResX, int ResY, float Aperture_ratio, float Focal_ratio) {
	    eye = from;
	    at = At;
	    up = Up;
	    fovy = angle;
	    vnear = hither;
	    vfar = yon;
	    res_x = ResX;
	    res_y = ResY;
		focal_ratio = Focal_ratio;

        // set the camera frame uvn
        n = ( eye - at );
        plane_dist = n.length();
	    n = n / plane_dist;

	    u = up % n;
	    u = u / u.length();

	    v = n % u;

        //Dimensions of the vis window
	    h = 2 * plane_dist * tan( (PI * angle / 180) / 2.0f );
        w = ( (float) res_x / res_y ) * h;  

		aperture = Aperture_ratio * (w / res_x); //Lens aperture = aperture_ratio * pixel_size

		printf("\nwidth=%f height=%f fov=%f, viewplane distance=%f, pixel size=%.3f\n", w,h, fovy,plane_dist, w/res_x);
		if (Aperture_ratio != 0) printf("\nDepth-Of-Field effect enabled with a lens aperture = %.1f\n", Aperture_ratio);
    }

	void SetEye(Vector from) {
		eye = from;
		// set the camera frame uvn
		n = (eye - at);
		plane_dist = n.length();
		n = n / plane_dist;
		u = up % n;
		u = u / u.length();
		v = n % u;
	}

	Ray PrimaryRay(const Vector& pixel_sample) //  Rays cast from the Eye to a pixel sample which is in Viewport coordinates
	{
		Vector ray_dir;
		
		ray_dir = (
			u * (w * (((pixel_sample.x + 0.5f) / res_x) - (0.5f))) +
			v * (h * (((pixel_sample.y + 0.5f) / res_y) - (0.5f))) +
			n * (- 1 * plane_dist)
		).normalize();

		return Ray(eye, ray_dir);  
	}

	Ray RandomDiskPrimaryRay(const Vector& pixel_sample) // DOF: Rays cast from a random disk sample to a pixel sample
	{
		Vector ray_dir;
		Vector eye_offset;

		Vector lens_sample = rnd_unit_disk() * aperture / 2.0f;

		eye_offset = eye + u * lens_sample.x + v * lens_sample.y;

		ray_dir = (
			u * ((pixel_sample.x * focal_ratio) - lens_sample.x) +
			v * ((pixel_sample.y * focal_ratio) - lens_sample.y) +
			n * (-1 * plane_dist)
		).normalize();


		return Ray(eye_offset, ray_dir);
	}

	Ray PrimaryRay(const Vector& lens_sample, const Vector& pixel_sample) // DOF: Rays cast from  a thin lens sample to a pixel sample
	{
		
		Vector ray_dir;
		Vector eye_offset;

		return Ray(eye_offset, ray_dir);
	}
};

#endif