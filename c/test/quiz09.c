#include <stdio.h>

int main(){
    int n;
    scanf("%d",&n);

/*
1단계 
--------
N? 5
*
**
***
****
*****
 */  
  
    for(int i=0;i<n;i++){
        for(int j=0;j<=i;j++){
            printf("*");
        }
        printf("\n");
    }
    printf("\n");

 /*
2단계 
--------
N? 5
*****
****
***
**
*
 */  

    for(int i=n;i>0;i--){
        for(int j=0;j<i;j++){
            printf("*");
        }
        printf("\n");
    }

  /*
3단계 
--------
N? 5
    *
   **
  ***
 ****
*****
 */  
    for(int i=0;i<n;i++){
        for(int j=0;j<n-i-1;j++){
            printf(" ");
        }
        for(int j=0;j<=i;j++){
            printf("*");
        }  
        printf("\n");
    }
    printf("\n");

   /*
4단계 
--------
N? 5
*****
 ****
  ***
   **
    *
 */ 
    for (int i = 0; i < n; i++) {
        // 공백: 행 번호(i) - 1 만큼 출력
        for (int j = 0; j < i ; j++) {
           printf(" ");
                }
             // 별: N - 행 번호(i) + 1 만큼 출력
               for (int j = 0; j < n - i; j++) {
                    printf("*");
                }
                printf("\n");
            }
/*
5단계 - 마름모
(홀수만가능)
N? 5
   *
  ***
 *****
  ***
   *        
*/
   for (int i = 0; i < n; i += 2) {
                // 공백: (N - i) / 2 만큼 출력
                for (int j = 0; j < (n - i) / 2; j++) {
                    printf(" ");
                }
                // 별: i 만큼 출력
                for (int j = 0; j <= i; j++) {
                    printf("*");
                }
                printf("\n");
            }

            // 하단 삼각형 (i = N-2 down to 1)
            // N이 5일 때, i=3, i=1 순서로 출력
            for (int i = n - 2; i > 0; i -= 2) {
                // 공백: (N - i) / 2 만큼 출력
                for (int j = 0; j < (n - i) / 2; j++) {
                    printf(" ");
                }
                // 별: i 만큼 출력
                for (int j = 0; j < i; j++) {
                    printf("*");
                }
                printf("\n");
            }


}
