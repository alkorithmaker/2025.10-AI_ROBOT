#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//팩토리얼 재귀 함수
int facto(int x){
    int mul=1;
  
    if(x==1){
        return mul;
        //break;
    }
      
    else{
      mul = facto(x-1) * x;
    }
}

int main(){
    int num;
    scanf("%d",&num);
    printf("팩토리얼 재귀 함수 o >> %d",facto(num));
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//점화식 5!=5*4*3*2*1
int main(){
    int num;
    int fac=1;

    scanf("%d",&num);
    for(int i = 1; i<=num ;i++){
        fac*=i;
    }
    printf("팩토리얼 재귀 함수 x >> %d",fac);
}
