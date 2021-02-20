#include <stdio.h>
#include <unistd.h>
 
int main(void)
{
    int i = 0;
    while(1){
        printf("%d\n", i);
        printf("\033[1A"); //先回到上一行
        printf("\033[K");  //清除该行
        i++;
        sleep(1);
    }
    return 0;
}
