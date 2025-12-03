#define _CRT_SECURE_NO_WARNINGS
#include<stdio.h>

int main() {
	int C;
	printf("섭씨 온도 입력 >>");
  //입력
	scanf("%d", &C);
  //로직
	double F = ((double)C * 9 / 5) + 32;
  //출
	printf("화씨 온도 >> %f", F );
  
	return 0;

}
