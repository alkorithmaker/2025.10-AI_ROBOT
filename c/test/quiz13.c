#include <stdio.h>
#define max 10

//13 - 1 
int main(){
    int grade[max];
    int input;
    for(int i=0;i<max;i++){
        grade[i] = 0; //배열 초기화
    }

    for(int i=0;i<max;i++){
        scanf("%d",&input);
        grade[i]=input; //배열에 값 저장
    }

    for(int i=0;i<max/2;i++){
        printf("%d ",grade[i*2+1]); //출력
    }
}

#include <stdio.h>

//13-2
int main()
{
    int arr[6] = {3, 7, 2, 9, 5, 1};
    int max=-2000000000;
    
    for(int i= 0; i<6;i++){
        if(max<arr[i])  max=arr[i];
    }
  
    for(int i= 0; i<6;i++){
        if(max==arr[i])  printf("max : %d \nindex : %d",max,i);
    }

}
