#ifndef ADACHI_H
#define ADACHI_H

#include "Solver.h"
#include <queue>
#include <vector>
#include <algorithm>

#define SET_NORTH(map,x,y,b) if(b){ map[(x)][(y)] |= 1 << Direction::North; map[(x)][(y)+1] |= 1 << Direction::South; }
#define SET_EAST(map,x,y,b)  if(b){ map[(x)][(y)] |= 1 << Direction::East;  map[(x)+1][(y)] |= 1 << Direction::West;  }
#define SET_SOUTH(map,x,y,b) if(b){ map[(x)][(y)] |= 1 << Direction::South; map[(x)][(y)-1] |= 1 << Direction::North; }
#define SET_WEST(map,x,y,b)  if(b){ map[(x)][(y)] |= 1 << Direction::West;  map[(x)-1][(y)] |= 1 << Direction::East;  }

#define RESET_UNEXPLORED(map,x,y) map[x][y] &= ~(1 << Direction::Unexplored);

#define IS_THROUGH(map,x,y,dir) (!(map[(x)][(y)] & (1 << (dir)))) 


struct vec{
	int8_t x,y;
	vec(){}
	vec(int8_t _x,int8_t _y):x(_x),y(_y){}
	bool operator==(const vec& v){ return x == v.x && y == v.y;}
};


template<int N>
class Adachi : Solver{
public:
	
	Adachi(const std::vector<vec>& g)
	: goals(g)
	{
		reset();
	}
	
	~Adachi(){
	}

	void setWall(bool left,bool front,bool right){
		int8_t x = cur.x, y = cur.y;
		switch(cur_dir){
		case Direction::North : SET_WEST (map,x  ,y+1,left); SET_NORTH(map,x  ,y+1,front); SET_EAST (map,x  ,y+1,right); break;
		case Direction::East  : SET_NORTH(map,x+1,y  ,left); SET_EAST (map,x+1,y  ,front); SET_SOUTH(map,x+1,y  ,right); break;
		case Direction::South : SET_EAST (map,x  ,y-1,left); SET_SOUTH(map,x  ,y-1,front); SET_WEST (map,x  ,y-1,right); break;
		case Direction::West  : SET_SOUTH(map,x-1,y  ,left); SET_WEST (map,x-1,y  ,front); SET_NORTH(map,x-1,y  ,right); break;
		}
	}

	Operator next(void){
		RESET_UNEXPLORED(map,cur.x,cur.y);

		return Operator::Back;
	}

	void reset(void){
		cur.x = 1;
		cur.y = 1;
		cur_dir = Direction::North;
	
	
		for(int i=1;i<=N;i++){
			for(int j=1;j<=N;j++){
				map[i][j] = 1 << Direction::Unexplored;
			}
		}

		for(auto& g : goals) map[g.x][g.y] |= 1 << GOAL_FLAG;

		for(int i=1;i<=N;i++){
			SET_NORTH(map,i,N,true);
			SET_EAST (map,N,i,true);
			SET_SOUTH(map,i,1,true);
			SET_WEST (map,1,i,true);
		}
		
		SET_EAST(map,1,1,true);
		RESET_UNEXPLORED(map,1,1);

		calcStep(cur);

	}

#ifdef SIMULATION
	void print(void){
		printf("map | step | parent\n");
		for(int j=N;j>=1;j--){
			for(int i=1;i<=N;i++){
				printf("%02x ",map[i][j]);
			}
			printf("  |  ");
			for(int i=1;i<=N;i++){
				printf("%03d ",step[i][j]);
			}
			printf("  |  ");
			for(int i=1;i<=N;i++){
				printf("%02d-%02d ",parent[i][j].x,parent[i][j].y);
			}
			printf("\n");
		}
	}
	void putWall(uint8_t x,uint8_t y,bool n,bool e,bool s,bool w){
		SET_NORTH(map,x,y,n);
		SET_EAST (map,x,y,e);
		SET_SOUTH(map,x,y,s);
		SET_WEST (map,x,y,w);
	}

#endif

private:
		
	// |8|7|6|5|4|3|2|1|
	// |u|g| | |W|S|E|N|
	// if wall is , set bits
	// u is where machine has gone before.
	uint8_t map[N+2][N+2];

	uint16_t step[N+2][N+2];
	vec parent[N+2][N+2];
	std::vector<vec> goals;
	const uint8_t  GOAL_FLAG = 6;
	const uint16_t INF = 999;
	vec cur;
	Direction cur_dir;


	void calcStep(const vec& s){
		auto comp = [&](vec& a,vec& b){ return step[a.x][a.y] > step[b.x][b.y]; };
		std::priority_queue<vec,std::vector<vec>,decltype(comp)> q(comp);

		for(int i=1;i<=N;i++) for(int j=1;j<=N;j++){
			step[i][j] = INF;
			parent[i][j] = vec(-1,-1);
		}

		for(auto& g : goals) step[g.x][g.y] = 0;
		

		for(auto& g : goals) q.push(g);
int count = 0;
		while(!q.empty()){ // Breadth First
			count++;
			auto c = q.top();
			q.pop();

			//if(c.x == s.x && c.y == s.y) break; // fin calc.

			#define IF(dir,nx,ny,weight) \
			if(IS_THROUGH(map,c.x,c.y,dir) && !isGoal(vec(nx,ny))){    \
				if(step[nx][ny] > step[c.x][c.y] + weight){ \
					/*printf("%d %d\n",nx,ny); print();*/   \
					step[nx][ny] = step[c.x][c.y] + weight; \
					parent[nx][ny] = c;                     \
					q.push(vec(nx,ny));                     \
				}                                           \
			}                                               \

			vec d[4] = {{0,1},{1,0},{0,-1},{-1,0}};
			vec p = parent[c.x][c.y];
			for(int i=0;i<4;i++){
				if(step[c.x][c.y] <= step[c.x+d[i].x][c.y+d[i].y] && step[p.x][p.y] >= step[c.x+d[i].x][c.y+d[i].y]){
					p.x = c.x + d[i].x;
					p.y = c.y + d[i].y;
				}
			}
			
			uint16_t weight_N = isGoal(c) ? 1 : (p == vec(c.x  ,c.y-1) ? 1 : 2);
			uint16_t weight_E = isGoal(c) ? 1 : (p == vec(c.x-1,c.y  ) ? 1 : 2);
			uint16_t weight_S = isGoal(c) ? 1 : (p == vec(c.x  ,c.y+1) ? 1 : 2);
			uint16_t weight_W = isGoal(c) ? 1 : (p == vec(c.x+1,c.y  ) ? 1 : 2);

			

			IF(Direction::North,c.x  ,c.y+1,weight_N);
			IF(Direction::East ,c.x+1,c.y  ,weight_E);
			IF(Direction::South,c.x  ,c.y-1,weight_S);
			IF(Direction::West ,c.x-1,c.y  ,weight_W);
			
			#undef IF
		}

printf("count %d\n",count);
	}

	inline bool isGoal(const vec& v){
		return map[v.x][v.y] & (1 << GOAL_FLAG);
	}

};



#endif // SOLVER_H

