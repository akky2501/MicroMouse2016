#ifndef SOLVER_H
#define SOLVER_H

#include <stdint.h>

enum Direction{
	North = 0,East,South,West,
	Unexplored = 7
};

enum Operator{
	Straight,Left,Right,Back
};

class Solver{
public:

	Solver(){};
	virtual ~Solver(){};

	virtual void setWall(bool left,bool front,bool right) = 0;
	virtual Operator next(void) = 0;

	virtual void reset(void) = 0;

private:
};



#endif // SOLVER_H
