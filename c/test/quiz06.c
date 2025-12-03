#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

int main(){
	int chul_score;
	int young_score;
	int min_score;

	printf("세 학생의 점수를 입력.\n");
  //입력
	printf("철수>>> ");
	scanf("%d", &chul_score);
	printf("영희>>>");
	scanf("%d", &young_score);
	printf("민수>>>");
	scanf("%d", &min_score);

  //로직&출력
	if (chul_score == young_score && young_score == min_score)
	  printf("모두 동점");

	else if (chul_score > young_score && chul_score > min_score)
	  printf("1등은 철수 %d점", chul_score);

	else if (young_score > min_score)
	  printf("1등은 영희 %d점", young_score);

	else 
		printf("1등은 민수 %d점", min_score);


	return 0;

}
