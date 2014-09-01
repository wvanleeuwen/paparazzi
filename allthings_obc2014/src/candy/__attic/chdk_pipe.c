


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main( int argc, char *argv[] )
{

  FILE *fp;
  int status;
  char path[1035];

  /* Open the command for reading. */
  fp = popen("cd ../popcorn/chdkptp/lua/ && ../chdkptp -c -e'rec'", "w");
  if (fp == NULL) {
    printf("Failed to start chdkptp pipe\n" );
    exit;
  }

  sleep(1);

  /* Read the output a line at a time - output it. */
  while (fgets(path, sizeof(path)-1, fp) != NULL) {
    printf("%s", path);
  }

  printf("shoot\n");
  fputs(  "shoot\n", fp);
  if (ferror(fp))
  {
    printf("Error writing shoot to pipe\n");
  }

  /* Read the output a line at a time - output it. */
  while (fgets(path, sizeof(path)-1, fp) != NULL) {
    printf("%s", path);
  }

  printf("shoot\n");
  fputs(  "shoot\n", fp);
  if (ferror(fp))
  {
    printf("Error writing shoot to pipe\n"); 
  }

  /* Read the output a line at a time - output it. */
  while (fgets(path, sizeof(path)-1, fp) != NULL) {
    printf("%s", path);
  }


  /* close */
  pclose(fp);

  return EXIT_SUCCESS;
}
