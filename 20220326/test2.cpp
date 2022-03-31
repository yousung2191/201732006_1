#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

float sum;
float datax[16];
float datay[16];

float dis(float x, float y, float a, float b) {
	return abs(a * x - y + b) / sqrt(a * a + 1);
}

float f(float a, float b) {
	sum = 0.0;
	for (int i = 0; i < 16; i++) {
		sum += dis(datax[i], datay[i], a, b) / 16;
	}
	return sum;
}

float dfabda(float a, float b, float da) {
	return (f(a + da, b) - f(a, b)) / da;
}
float dfabdb(float a, float b, float db) {
	return (f(a, b + db) - f(a, b)) / db;
}

float EE(float x0, float x1, float y0, float y1) {
	return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

int main() {
	srand((unsigned int)time(NULL));
	ifstream out("simple.txt");
	for (int i = 0; i < 16; i++) {
		out >> datax[i] >> datay[i];
		cout << datax[i] + ((float)(rand() % 100)/100 - 0.5) << " " << datay[i] + ((float)(rand() % 100)/100 - 0.5)<< endl;
	}
	float a0 = 0, b0 = 0;
	int iteration = 0;
	float da = 0.01;
	float db = 0.01;
	float a1 = 3, b1 = 4;
	while (EE(a0, b0, a1, b1) > 0.0001 && iteration < 1000000) {
		a0 = a1;
		b0 = b1;
		a1 -= 0.005 * dfabda(a0, b0, da);
		b1 -= 0.005 * dfabdb(a0, b0, db);
		iteration++;
	}
	cout << "y = " << a1 << "x + " << b1 << endl;
	cout << iteration << "-th  E = " << EE(a0, b0, a1, b1) << endl;
	return 0;
}
