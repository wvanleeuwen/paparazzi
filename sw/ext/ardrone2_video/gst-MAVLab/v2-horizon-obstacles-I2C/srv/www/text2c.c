// text2c.c 
//

#include <stdio.h>
#define true 1
#define false 0

int main(int argc, char* argv[])
{
    int eol = true;

    char ch;
    while ((ch = getchar()) != EOF) {
        if (eol) {
            printf ("\"");
            eol = false;
        }
        if (ch == '"')
            printf ("\\\"");
        else if (ch == '\\')
            printf ("\\\\");
        else if (ch == '\n') {
            printf ("\\r\\n\"\n");
            eol = true;
        }
        else
            putchar (ch);
    }

	return 0;
}

