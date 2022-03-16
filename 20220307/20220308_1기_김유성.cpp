#include <stdio.h>
#include <math.h>

int select;
long double value = 0;
int findcolor();
char color[5] = { 0, 0, 0, 0, 0 };
void ohm();

int main(void) {
        printf("4색코드는 4, 5색코드는 5를 입력해주세요: ");
        scanf("%d", &select);
        if (select == 4) {
            findcolor();
            value = color[0] * 10 + color[1];

            value *= pow(10, color[2]);
            ohm();
            return 0;
        }
        else if (select == 5) {
            findcolor();
            value = color[0] * 100 + color[1] * 10 + color[2];

            value *= pow(10, color[3]);
            ohm();
            return 0;
        }
    }

int findcolor() {

    for (int i = 0; i < select; i++) {
        printf("색깔을 입력하세요 (검=B,갈=b,빨=r,주=o,노=y,녹=g,청=u,보=v,회=s,흰=w,골드=G,은=S,없음=N): ");
        scanf(" %c", &color[i]);
        if (color[i] == 'B')
            color[i] = 0;
        else if (color[i] == 'b')
            color[i] = 1;
        else if (color[i] == 'r')
            color[i] = 2;
        else if (color[i] == 'o')
            color[i] = 3;
        else if (color[i] == 'y')
            color[i] = 4;
        else if (color[i] == 'g')
            color[i] = 5;
        else if (color[i] == 'u')
            color[i] = 6;
        else if (color[i] == 'v')
            color[i] = 7;
        else if (color[i] == 's')
            color[i] = 8;
        else if (color[i] == 'w')
            color[i] = 9;
    }
}

void ohm() {
    if (select == 4) {
        if (color[3] == 'G')
            printf("저항은 %lf ~ %lf입니다.", value * 0.95, value * 1.05);
        else if (color[3] == 'S')
            printf("저항은 %lf ~ %lf입니다.", value * 0.9, value * 1.1);
        else if (color[3] == 'N')
            printf("저항은 %lf ~ %lf입니다.", value * 0.8, value * 1.2);
    }
    else if (select == 5) {
        if (color[4] == 'G')
            printf("저항은 %lf ~ %lf입니다.", value * 0.95, value * 1.05);
        else if (color[4] == 'S')
            printf("저항은 %lf ~ %lf입니다.", value * 0.9, value * 1.1);
        else if (color[4] == 'N')
            printf("저항은 %lf ~ %lf입니다.", value * 0.8, value * 1.2);
    }
}