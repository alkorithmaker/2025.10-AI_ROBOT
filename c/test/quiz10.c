#include <stdio.h>

int plus(int a, int b){
	return a + b;
}

int minus(int a, int b){
	return a - b;
}

int multiple(int a, int b){
	return a * b;
}

double divide(double a, double b){
	return a / b;
}

int main() {

	int x, y;
	scanf("%d %d", &x, &y);

	printf("%d\n", plus(x, y));
	printf("%d\n", minus(x, y));
	printf("%d\n", multiple(x, y));
	printf("%.2lf\n", divide(x, y));

}
