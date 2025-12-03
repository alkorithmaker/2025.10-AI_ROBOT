#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

int main(){

	//1. 변수선언 및 입력
	int year_of_birth;
	int age;
	const int this_year = 2025; //현재 연도
  
	printf("언제 태어 나셨나요? :");
  //입력받음
	scanf("%d", &year_of_birth);

	// 로직
	age = this_year - year_of_birth;

	// 출력
	printf("당신의 나이는 %d세 입니다.", age);

	return 0;

}
