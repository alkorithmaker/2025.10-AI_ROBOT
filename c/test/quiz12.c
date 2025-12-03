#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>

int avg_fnc(){
    int a;
    double sum=0;
    for(int i=0; i<4;i++){
        scanf("%d",&a);  
        sum+=a;
    }
    printf("%.2lf\n",sum/4);
}

int main(){
    avg_fnc();
}
