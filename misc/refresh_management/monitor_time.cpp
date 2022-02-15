#include <iostream>
#include <sys/wait.h>
#include <fstream>
#include <unistd.h> 
#include <vector>
#include <signal.h>

/*
 * This main program starts Murdock - a social distance high traffic monitoring system
 *
 *  Parent process (Murdock): monitors the time and sends a signal 
 *                            to my object detection process so it can write
 *                            it's violation per hour to file
 *  
 *  Child Process (SDVT aka Social-Distance Violation Tracking): Executes 
 *                          OpenCV object detection code. Violations per hour 
 *                          are reported and written to an output file
 *
 *  Child Process (Qt): Executes Qt program that presents data finding to user.
*/

void create_shared_results_file();
void sigint_handler(int sig);
//#define TEST (1)
int main(){

  signal(SIGINT, sigint_handler);
  create_shared_results_file();
  pid_t SDVT;
  pid_t Qt;

  std::cout << "monitor - group ID: " << getpgid(getpid()) << std::endl;

  if(!(Qt = fork())){//Murdock Proc
 
    if(!(SDVT = fork())){// Murdock Proc
    
      char in;
      
      while (1){
        sleep(4);
        //setpgid(0,getp)

        std::cout << "Slept\n";
       
        kill(SDVT, SIGUSR1);
        std::cout << "Killed\n";
      }
    
    }
    else{ //SDVT Proc

      //setpgid(SDVT, SDVT);
      char *envp[] = {NULL};
      char *command[] = {"./tracking", NULL};
      execve("./tracking", command, envp);
    
    }

  }else{// Qt Proc
    std::cout << "Qt: " << Qt << std::endl;
    //setpgid(Qt, Qt);
    sleep(3);
  
    char *envp[] = {NULL};
    char *command[] = {"./Qt", NULL};
    execve("./Qt", command, envp);
  }
  std::cout << "Ended mon\n";

  return 0;
}


void create_shared_results_file(){

  std::ofstream file("violations.csv", std::ios::trunc);
  std::vector<int> new_set_violations(24,0);
  
  for(auto i = new_set_violations.begin(); i != new_set_violations.end(); ++i){
    file << *i << std::endl;
  }

  file.close();
}

void sigint_handler(int sig){
  int status;
  kill(-getpid(), sig);
  wait(&status);
  return;
}
