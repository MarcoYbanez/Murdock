#include <unistd.h>
#include <iostream>

using namespace std;

int main(){

  pid_t proc; 
  int in(1);
  int temp;
  if(proc= fork() == 0){
    std::cout << in;
    std::cin >> temp;
    std::cout << "END\n";
    return 0;
  }
  else{
    char *const parmList[] = {"./main", NULL};
    execv("./main", parmList);
  }


  return 0;
}
