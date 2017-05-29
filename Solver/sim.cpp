
#include<stdio.h>

#define SIMULATION
#include"Adachi.h"
#undef  SIMULATION

int main(void){
	std::vector<vec> goal = {vec(8,8)};
	Adachi<8> solver(goal);

	solver.print();	
}
