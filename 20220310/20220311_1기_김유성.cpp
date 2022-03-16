#include <stdio.h>
#include <string.h>

int main(void) {
	FILE* fp = fopen("test.txt", "w");
	unsigned int num;
	unsigned char data[100] = { 0, };

	printf("���� �Է�: ");
	scanf("%d", &num);

	memcpy(data, &num, sizeof(int));

	printf("MSB -> LSB byte list = ");

	fprintf(fp, "�Է¹��� ����: %d\n\n", num);
	fprintf(fp, "MSB -> LSB byte list = ");

	for (int i = 0; i < 4; i++) {
		printf("%#x ", data[i]);

		fprintf(fp, "%#x ", data[i]);
	}
	printf("\n\n");
	printf("LSB -> MSB byte list = ");

	fprintf(fp, "\n\n");
	fprintf(fp, "LSB -> MSB byte list = ");

	for (int i = 3; i >= 0; i--) {
		printf("%#x ", data[i]);

		fprintf(fp, "%#x ", data[i]);
	}
	printf("\n\n");
	fprintf(fp, "\n\n");

	fclose(fp);

	return 0;
}