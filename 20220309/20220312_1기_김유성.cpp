#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define N 15
#define M 60

int main()
{
    int i, j, k = 0;
    char* arr = (char*)calloc(N, sizeof(char));
    char arr_2[M] = { 0, };

    for (i = 0; i < M; i++)
    {
        scanf(" %c", &arr[i % N]);

        for (j = 0; j < N; j++)
        {
            printf("%c", arr[j]);
        }
        printf("\n");

        arr_2[i] = arr[i % N];

        char* ptr = strstr(arr_2, "#S5D#*");

        if (ptr != NULL) {
            for (k = 0; k < 6; k++) {
                int arr_s = i % N - k;
                if (arr_s < 0) {
                    arr_s += N;
                }
                arr[arr_s] = ' ';
            }
            i -= 6;
        }
    }


    return 0;
}