#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>

void print_line(int x);

int main(){

    print_line(30);
    printf("학번\t이름\t전공\t학점\n");
  
    print_line(30);
    
  return 0;
}

void print_line(int x){
    for(int i=0;i<x;i++)
        printf("-");
    printf("\n");    

}
