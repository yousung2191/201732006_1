#include <stdio.h>

void findcolor(long long a);

int main(void) {
	long long resist, max, min, per;
	int count = 0;
	printf("���� ���� �Է��ϼ���: ");
	scanf("%lld", &resist);
	printf("���� ���� ���� �ۼ�Ʈ �Է�(%%): ");
	scanf("%lld", &per);
	per *= 2;

	for (int i = 0; resist > 1000; i++) {
		resist /= 10;
		count++;
	}

	if (resist < 100) {
		findcolor((resist / 10) % 10);
		findcolor(resist % 10);
		findcolor(count);
		findcolor(per);
	}
	else if (resist >= 100 && resist < 1000 && resist % 10 != 0) {
		printf("5band�� �÷� �ڵ� -> ");
		findcolor((resist / 100) % 10);
		findcolor((resist / 10) % 10);
		findcolor(resist % 10);
		findcolor(count);
		findcolor(per);
		printf("\n");
	}
	else {
		printf("5band�� �÷� �ڵ� -> ");
		findcolor((resist / 100) % 10);
		findcolor((resist / 10) % 10);
		findcolor(resist % 10);
		findcolor(count);
		findcolor(per);
		printf("\n");

		printf("4band�� �÷� �ڵ� -> ");
		findcolor((resist / 100) % 10);
		findcolor((resist / 10) % 10);
		findcolor(count + 1);
		findcolor(per);
		printf("\n");
	}
	return 0;
}

void findcolor(long long a) {
	switch (a)
	{
	case 0:
		printf("������ ");
		break;
	case 1:
		printf("���� ");
		break;
	case 2:
		printf("������ ");
		break;
	case 3:
		printf("��Ȳ�� ");
		break;
	case 4:
		printf("����� ");
		break;
	case 5:
		printf("�ʷϻ� ");
		break;
	case 6:
		printf("�Ķ��� ");
		break;
	case 7:
		printf("����� ");
		break;
	case 8:
		printf("ȸ�� ");
		break;
	case 9:
		printf("�Ͼ�� ");
		break;
	case 10:
		printf("�ݻ�");
		break;
	case 20:
		printf("����");
		break;
	case 40:
		printf("���� ");
		break;
	}
}