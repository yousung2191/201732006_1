#include<iostream>
#include<vector>
using namespace std;

struct Vector
{
	float x;
	float y;
	float z;
};
//외적
Vector ou(Vector a, Vector b) {
	Vector vt;
	vt.x = a.y * b.z - a.z * b.y;
	vt.y = a.z * b.x - a.x * b.z;
	vt.z = a.x * b.y - a.y * b.x;

	return vt;
}
//내적
float iu(Vector a, Vector b) {
	return a.x * b.x + a.y + b.y + a.z * b.z;
}

int main() {
	Vector v1, v2;

	return 0;
}