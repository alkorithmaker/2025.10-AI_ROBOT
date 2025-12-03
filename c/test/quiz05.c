#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

int main(){
	int product_price;
	int paid_price;
  int exchange;
	printf("물건 가격 : ");
  
  //입력
	scanf("%d", &product_price);
	printf("지불 금액 : ");
	scanf("%d", &paid_price);
	printf("\n");

  //로직&출력
	exchange = paid_price - product_price;

	printf("거스름돈 : %d\n", exchange);
  
	printf("(1000원 : %d개 / ", exchange / 1000);
	exchange %= 1000;

	printf("500원 : % d개 / ", exchange / 500);
	exchange %= 500;

	printf("100원 : % d개)\n", exchange / 100);

  //그리드 알고리즘
	return 0;

}
