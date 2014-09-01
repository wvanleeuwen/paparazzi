/* execill - How a parent and child might communicate. */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#define SHELL "/bin/sh"

void main(int argc, char ** argv, char ** envp)
{
	int pid;
	int pc[2]; /* Parent to child pipe */
	int cp[2]; /* Child to parent pipe */
	char ch;
	int incount = 0, outcount = 0;

	/* Make pipes */
	if( pipe(pc) < 0)
	{
		perror("Can't make pipe");
		exit(1);
	}
	if( pipe(cp) < 0)
	{
		perror("Can't make pipe");
		exit(1);
	}


	/* Create a child to run command. */
	switch( pid = fork() )
	{
		case -1: 
				perror("Can't fork");
				exit(1);
		case 0:
				/* Child. */
				close(1); /* Close current stdout. */
				dup( cp[1]); /* Make stdout go to write
						   end of pipe. */
				close(0); /* Close current stdin. */
				dup( pc[0]); /* Make stdin come from read
						   end of pipe. */
				close( pc[1]);
				close( cp[0]);
				//exec("repeat", );//, envp);
				execl (SHELL, SHELL, "-c", "cd ../popcorn/chdkptp/lua/ && ../chdkptp ", NULL);
				perror("No exec");
				signal(getppid(), SIGQUIT);
				exit(1);
		default:
				/* Parent. */
				/* Close what we don't need. */
				
/*
				printf("Input to child:\n");
				while( ( read(0, &ch, 1) > 0) && (ch != '\n') )
				{
					write(pc[1],&ch, 1);
					write(1, &ch, 1);
					incount ++;
				}

*/
                                sleep(5);
                                write(pc[1],"connect\n", 8);

				fcntl(cp[0], F_SETFL, FNDELAY);

                                printf("\nOutput from child:\n");
                                while( (read(cp[0], &ch, 1) == 1) && (ch != '>'))
                                {
                                        write(1, &ch, 1);
                                        outcount++;
                                }
				printf("Finished Reading");
				sleep(3);
				write(pc[1],"rec\n", 4);
				sleep(1);
				printf("\nOutput from child:\n");
				while( (read(cp[0], &ch, 1) == 1) && (ch != '>'))
				{
					write(1, &ch, 1);
					outcount++;
				}
				printf("\n\nTotal characters in: %d\n",incount);
				printf("Total characters out: %d\n", outcount);
				

				write(pc[1],"quit\n",5);

				close(pc[1]);
				close(cp[1]);
				exit(0);
	}

}
