#include <vector>
#include <iostream>
#include <limits>
#include "tgaimage.h"
#include "model.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);

Model *model = NULL;
const int width = 800;
const int height = 800;

//Structure declaration
struct Vect2
{
	int x;
	int y;

	Vect2() {
		x = 0;
		y = 0;
	}

	Vect2(int a, int b)
	{
		x = a;
		y = b;
	}
};

struct Vect3
{
	union
	{
		struct
		{
			double x, y, z;
		};
		double raw[3];
	};

	double &operator[](int i) { return raw[i]; }

	Vect3(double a, double b, double c)
	{
		x = a;
		y = b;
		z = c;
	}
};

//Functions
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
	bool steep = false;
	if (std::abs(x0 - x1) < std::abs(y0 - y1))
	{
		std::swap(x0, y0);
		std::swap(x1, y1);
		steep = true;
	}
	if (x0 > x1)
	{
		std::swap(x0, x1);
		std::swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = y1 - y0;
	int derror2 = std::abs(dy) * 2;
	int error2 = 0;
	int y = y0;
	for (int x = x0; x <= x1; x++)
	{
		if (steep)
		{
			image.set(y, x, color);
		}
		else
		{
			image.set(x, y, color);
		}
		error2 += derror2;
		if (error2 > dx)
		{
			y += (y1 > y0 ? 1 : -1);
			error2 -= dx * 2;
		}
	}
}

void transposerMatrice(Vect3 mat[3], Vect3 result[3])
{
	result[0].x = mat[0].x;
	result[0].y = mat[1].x;
	result[0].z = mat[2].x;

	result[1].x = mat[0].y;
	result[1].y = mat[1].y;
	result[1].z = mat[2].y;

	result[2].x = mat[0].z;
	result[2].y = mat[1].z;
	result[2].z = mat[2].z;
}

void inverseMatrice(Vect3 mat[3], Vect3 result[3])
{
	//Déterminant de la matrice
	double det;

	det = mat[0].x * mat[1].y * mat[2].z + mat[0].y * mat[1].z * mat[2].x + mat[1].x * mat[2].y * mat[0].z - (mat[0].z * mat[1].y * mat[2].x + mat[0].y * mat[1].x * mat[2].z + mat[0].x * mat[1].z * mat[2].y);

	Vect3 inv[3] = {Vect3(0, 0, 0), Vect3(0, 0, 0), Vect3(0, 0, 0)};

	//Matrice adjointe
	inv[0].x = mat[1].y * mat[2].z - mat[2].y * mat[1].z;
	inv[0].y = mat[1].x * mat[2].z - mat[2].x * mat[1].z;
	inv[0].z = mat[1].x * mat[2].y - mat[2].x * mat[1].y;

	inv[1].x = mat[0].y * mat[2].z - mat[2].y * mat[0].z;
	inv[1].y = mat[0].x * mat[2].z - mat[2].x * mat[0].z;
	inv[1].z = mat[0].x * mat[2].y - mat[2].x * mat[0].y;

	inv[2].x = mat[0].y * mat[1].z - mat[1].y * mat[0].z;
	inv[2].y = mat[0].x * mat[1].z - mat[1].x * mat[0].z;
	inv[2].z = mat[0].x * mat[1].y - mat[1].x * mat[0].y;

	//Matrice des co-facteurs
	inv[0].y = inv[0].y * (-1);
	inv[1].x = inv[1].x * (-1);
	inv[1].z = inv[1].z * (-1);
	inv[2].y = inv[2].y * (-1);

	for (int i = 0; i < 3; i++)
	{
		inv[i].x = inv[i].x / det;
		inv[i].y = inv[i].y / det;
		inv[i].z = inv[i].z / det;
	}

	//Transposition de la matrice
	transposerMatrice(inv, result);
}

Vect3 baricentricCoordinates(Vect3 tri[3], Vect3 p)
{
	double x, y, z;

	//Inversion de la matrice du triangle
	Vect3 inv[3] = {Vect3(0, 0, 0), Vect3(0, 0, 0), Vect3(0, 0, 0)};
	inverseMatrice(tri, inv);

	//Produit scalaire de l'inverse avec le point P
	x = inv[0].x * p.x + inv[0].y * p.y + inv[0].z * p.z;
	y = inv[1].x * p.x + inv[1].y * p.y + inv[1].z * p.z;
	z = inv[2].x * p.x + inv[2].y * p.y + inv[2].z * p.z;

	return Vect3(x, y, z);
}

void triangle(Vect3 a, Vect3 b, Vect3 c, TGAImage &image, TGAColor color, float *zbuffer)
{
	//Max x
	int maxx = std::max(a.x, std::max(b.x, c.x));

	//Max y
	int maxy = std::max(a.y, std::max(b.y, c.y));

	//Min x
	int minx = std::min(a.x, std::min(b.x, c.x));

	//Min y
	int miny = std::min(a.y, std::min(b.y, c.y));

	//Matrices
	Vect3 tri[3] = {{a.x, b.x, c.x}, {a.y, b.y, c.y}, {1, 1, 1}};

	//Remplissage du triangle
	for (int i = maxy; i > miny; i--)
	{
		for (int j = minx; j < maxx; j++)
		{
			Vect3 P = Vect3(j, i, 1);
			Vect3 baricentre = baricentricCoordinates(tri, P);
			if (baricentre.x < 0 || baricentre.y < 0 || baricentre.z < 0)
			{
				continue;
			}

			P.z = 0;
			P.z += a.z * baricentre.x;
			P.z += b.z * baricentre.y;
			P.z += c.z * baricentre.z;

			if (zbuffer[int(P.x + P.y * width)] < P.z)
			{
				zbuffer[int(P.x + P.y * width)] = P.z;
				image.set(P.x, P.y, color);
			}
		}
	}
}


void triangle(Vect3 a, Vect3 b, Vect3 c, Vect2 vt[3], TGAImage &image, TGAImage &texture, float *zbuffer)
{
	//Max x
	int maxx = std::max(a.x, std::max(b.x, c.x));

	//Max y
	int maxy = std::max(a.y, std::max(b.y, c.y));

	//Min x
	int minx = std::min(a.x, std::min(b.x, c.x));

	//Min y
	int miny = std::min(a.y, std::min(b.y, c.y));

	//Matrices
	Vect3 tri[3] = {{a.x, b.x, c.x}, {a.y, b.y, c.y}, {1, 1, 1}};

	//Remplissage du triangle
	for (int i = maxy; i > miny; i--)
	{
		for (int j = minx; j < maxx; j++)
		{
			Vect3 P = Vect3(j, i, 1);
			Vect3 baricentre = baricentricCoordinates(tri, P);
			if (baricentre.x < 0 || baricentre.y < 0 || baricentre.z < 0)
			{
				continue;
			}

			P.z = 0;
			P.z += a.z * baricentre.x;
			P.z += b.z * baricentre.y;
			P.z += c.z * baricentre.z;

			if (zbuffer[int(P.x + P.y * width)] < P.z)
			{
				zbuffer[int(P.x + P.y * width)] = P.z;

				int u = vt[0].x * baricentre.x + vt[1].x * baricentre.y + vt[2].x * baricentre.z;
				int v = vt[0].y * baricentre.x + vt[1].y * baricentre.y + vt[2].y * baricentre.z;
				
				TGAColor color = texture.get(u,v);
				image.set(P.x, P.y, color);
			}
		}
	}
}
int main(int argc, char **argv)
{
	/*TGAImage image(100, 100, TGAImage::RGB);

	triangle(Vect2(50, 20), Vect2(70, 30), Vect2(60, 80), image);

	image.set(52, 41, red);
	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");

	return 0;*/

	model = new Model("obj/african_head.obj");

	TGAImage texture;
	texture.read_tga_file("./obj/african_head_diffuse.tga");
	texture.flip_vertically();


	TGAImage image(width, height, TGAImage::RGB);
	float zbuffer[width * height];
	for (int i = 0; i < width * height; i++)
	{
		zbuffer[i] = -std::numeric_limits<int>::max();
	}

	for (int i = 0; i < model->nfaces(); i++)
	{
		std::vector<int> face = model->face(i);

		Vec3f v0 = model->vert(face[0]);
		Vec3f v1 = model->vert(face[(2) % 6]);
		Vec3f v2 = model->vert(face[(4) % 6]);

		int x0 = (v0.x + 1.) * width / 2.;
		int y0 = (v0.y + 1.) * height / 2.;
		int x1 = (v1.x + 1.) * width / 2.;
		int y1 = (v1.y + 1.) * height / 2.;
		int x2 = (v2.x + 1.) * width / 2;
		int y2 = (v2.y + 1.) * height / 2;

		Vec3f tmp1 = model->vt(face[1]);
		Vec3f tmp2 = model->vt(face[3]);
		Vec3f tmp3 = model->vt(face[5]);

		Vect2 vt[3];
		vt[0] = {tmp1.x * texture.get_width(), tmp1.y * texture.get_height()};
		vt[1] = {tmp2.x * texture.get_width(), tmp2.y * texture.get_height()};
		vt[2] = {tmp3.x * texture.get_width(), tmp3.y * texture.get_height()};

		//triangle(Vect3(x0, y0, v0.z * 1000), Vect3(x1, y1, v1.z * 1000), Vect3(x2, y2, v2.z * 1000), image, TGAColor(rand() % 255, rand() % 255, rand() % 255, 255), zbuffer);
		triangle(Vect3(x0, y0, v0.z * 1000), Vect3(x1, y1, v1.z * 1000), Vect3(x2, y2, v2.z * 1000), vt, image, texture, zbuffer);
	}

	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	delete model;
	return 0;
}