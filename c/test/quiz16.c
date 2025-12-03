#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define len 7 //1000001

int print_max(int* arr){
	int max = -2000000000;
  
	for (int i = 0; i < len; i++) {
		if (max < arr[i]) max = arr[i];
	}
	printf("max>>> %d", max);
}

int main(){
	int array[len] = { 4,5,8,1,2,3,7 };
	print_max(array);
}

