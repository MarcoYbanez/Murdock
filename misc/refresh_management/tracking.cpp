#include <iostream>
#include <unistd.h>
#include <fstream>
#include <signal.h>
#include <string>
#include <vector>

std::vector<int> violation_occurances(24, 1);
int cur_hour = 0;
int cur_violations = 0;
int hr_int = 0;

// this handler will export reportings to violations.csv


void sigusr1_handler(int sig);
void sigint_handler(int sig);

int main(){

  std::cout << "tracking - group ID: " << getpgid(getpid()) << std::endl;

  signal(SIGUSR1, sigusr1_handler);
  signal(SIGINT, sigint_handler);
  
  //get current hour at start of program 
    

  // infinite loop 
  // testing signal
  while(1){
    sleep(1);
    std::cout << "asd\n" ;
  }




}

void sigint_handler(int sig){
  kill(-getpid(), sig);
  return;
}

void sigusr1_handler(int sig){



  violation_occurances[cur_hour] = cur_violations;
  
  //if threaded (seems like extra work. probably not necessary)
  //  - make local copy of main vector
  //  - start thread to export 

  //else
  
  // write results to file
  std::ofstream file("violations.csv", std::ios::trunc);

  
  for(auto i = violation_occurances.begin(); i != violation_occurances.end(); ++i){
    file << *i << std::endl;
  }

  file.close();

  // change to next hour & reset violation count
  hr_int++;
  if(hr_int == 24){hr_int == 0;}

  cur_violations = 0;

  return;
}
