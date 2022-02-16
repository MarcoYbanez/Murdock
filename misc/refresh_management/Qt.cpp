#include <signal.h>
#include <unistd.h>
#include <iostream>

void sigH(int sig){

  kill(-getpid(), sig);

  return;
}


int main(){

  std::cout << "QT proc has started\n";
  std::cout << "QT - group ID: " << getpgid(getpid()) << std::endl;

  while(1){


    std::cout << "RUNNING\n" ;
    sleep(2);
  }


  return 0;
}
