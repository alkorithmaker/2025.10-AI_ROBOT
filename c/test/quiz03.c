#define _CRT_SECURE_NO_WARNINGS
#include<stdio.h>

int main(){

	int num;
	printf("정수를 입력하세요 : ");
  
  //입력
	scanf("%d", &num);
  
  //로직
	if (num % 2 == 0) {
    //출력1
		printf("입력 받은 정수는 짝수입니다.");
	}
    
	else {
    //출력2
		printf("입력 받은 정수는 홀수입니다.");
	}

	return 0;

}
