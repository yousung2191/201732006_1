#include <stdio.h>

void findcolor(long long a);

int main(void) {
	long long resist, max, min, per;
	int count = 0;
	printf("저항 값을 입력하세요: ");
	scanf("%lld", &resist);
	printf("저항 값의 오차 퍼센트 입력(%%): ");
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
		printf("5band의 컬러 코드 -> ");
		findcolor((resist / 100) % 10);
		findcolor((resist / 10) % 10);
		findcolor(resist % 10);
		findcolor(count);
		findcolor(per);
		printf("\n");
	}
	else {
		printf("5band의 컬러 코드 -> ");
		findcolor((resist / 100) % 10);
		findcolor((resist / 10) % 10);
		findcolor(resist % 10);
		findcolor(count);
		findcolor(per);
		printf("\n");

		printf("4band의 컬러 코드 -> ");
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
		printf("검정색 ");
		break;
	case 1:
		printf("갈색 ");
		break;
	case 2:
		printf("빨강색 ");
		break;
	case 3:
		printf("주황색 ");
		break;
	case 4:
		printf("노란색 ");
		break;
	case 5:
		printf("초록색 ");
		break;
	case 6:
		printf("파란색 ");
		break;
	case 7:
		printf("보라색 ");
		break;
	case 8:
		printf("회색 ");
		break;
	case 9:
		printf("하얀색 ");
		break;
	case 10:
		printf("금색");
		break;
	case 20:
		printf("은색");
		break;
	case 40:
		printf("없음 ");
		break;
	}
}