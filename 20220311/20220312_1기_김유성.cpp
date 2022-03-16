#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(void) {
	FILE* fp = fopen("test.txt", "r");
	unsigned char data[100] = { 0, };

	for (int i = 0; i < 4; i++) {
		fscanf(fp, "%x", &data[i]);
	}

	fclose(fp);
	printf("MSB -> LSB: 0x%x 0x%x 0x%x 0x%x\n", data[0], data[1], data[2], data[3]);
	printf("LSB -> MSB: 0x%x 0x%x 0x%x 0x%x\n", data[3], data[2], data[1], data[0]);


	return 0;
}