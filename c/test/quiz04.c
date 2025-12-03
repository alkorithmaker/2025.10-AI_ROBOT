#define _CRT_SECURE_NO_WARNINGS
#include<stdio.h>

int main(){
	int score[3];
	double avg;
	printf("성적 입력 \n");
  
  //입력
	scanf("%d \n %d \n %d", &score[0], &score[1], &score[2]);

  //로직
	int total = score[0] + score[1] + score[2];
	avg = total / 3.0;

  //출력
	printf("총점 : %d\n",total);
	printf("평균 : %.2lf\n",avg);

	return 0;
}
