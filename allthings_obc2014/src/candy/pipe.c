#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

pid_t pid = NULL;
int pipefd[2];
FILE* output;
char buf[256];

int main()
{
  pipe(pipefd);
  pid = fork();
  if (pid == 0)
  {
  // Child
    printf("Child Process");
    dup2(pipefd[0], STDIN_FILENO);
    dup2(pipefd[1], STDOUT_FILENO);
    dup2(pipefd[1], STDERR_FILENO);
    printf("CHILD TEXT\n");
    execl("./repeat", "argument", (char*) NULL);
    // Nothing below this line should be executed by child process. If so, 
    // it means that the execl function wasn't successfull, so lets exit:
    exit(1);
  }
  else 
  {
  // The code below will be executed only by parent. You can write and read
  // from the child using pipefd descriptors, and you can send signals to 
  // the process using its pid by kill() function. If the child process will
  // exit unexpectedly, the parent process will obtain SIGCHLD signal that
  // can be handled (e.g. you can respawn the child process).

  // Now, you can write to the process using pipefd[0], and read from pipefd[1]:

  // sleep(1);

  write(pipefd[0], "shooti\n\r", 7); // write message to the process

  //sleep(3);
  read(pipefd[1], buf, sizeof(buf)); // read from the process. Note that this will catch 
  //sleep(3);           
                        // standard  output together with error output
  kill(pid, SIGQUIT); //send signo signal to the child process
  
  printf("%s\n",buf);
  }
  return 0;
}
