#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>
#include <iostream>
#include <queue>
#include <set>
#include <stack>
#include <stdlib.h>
#include <time.h>
#include <vector>

using namespace std;

// Auxiliary function for calculating powers in O(log(n))
int power(int a, int b){
	int ret = 1;
	while(b > 0){
		if(b%2 == 1){ret *= a;}
		a*=a;
		b /= 2;
	}
	return ret;
}

class board{

	// Attributes


	// Current state of the board
	int	state;

	// Index of the empty position
	int index;

	// Distance from starting node
	int distance;

	// Sum of the Manhattan Distance between the current state and the goal state for each piece
	int heuristic;

	public:

		// Constructor
		board(int s, int i){
			state = s;
			index = i;
		}

		// Alternative constructor to set distances and heuristics
		board(int s, int i, int d, int h){
			state = s;
			index = i;
			distance = d;
			heuristic = h;
		}

		// Destructor
		~board(){}



		// Methods


		// Getters

		int getState(){ return state; }

		int getIndex(){ return index; }

		int getDistance(){ return distance; }

		int getHeuristic(){ return heuristic; }

		bool sameState(board* b){
			return state == (b -> getState());
		}


		// Move functions for uninformed search

		board* moveUp(){
			board* copy = new board(state,index);
			if(index - 3 < 0){ return copy; }
			board* ret = new board(state+((state/power(10,index - 3))%10-(state/power(10,index))%10 )*power(10,index)+((state/power(10,index))%10-(state/power(10,index - 3))%10)*power(10, index - 3), index - 3);
			return ret;
		}

		board* moveDown(){
			board* copy = new board(state,index);
			if(index + 3 > 8){ return copy; }
			board* ret = new board(state+((state/power(10,index + 3))%10-(state/power(10,index))%10 )*power(10,index)+((state/power(10,index))%10-(state/power(10,index + 3))%10)*power(10, index + 3), index + 3);
			return ret;
		}

		board* moveLeft(){
			board* copy = new board(state,index);
			if(index%3 == 0){ return copy; }
			board* ret = new board(state+((state/power(10,index - 1))%10-(state/power(10,index))%10 )*power(10,index)+((state/power(10,index))%10-(state/power(10,index - 1))%10)*power(10, index - 1), index - 1);
			return ret;
		}

		board* moveRight(){
			board* copy = new board(state,index);
			if( (index + 1)%3 == 0){ return copy; }
			board* ret = new board(state+((state/power(10,index + 1))%10-(state/power(10,index))%10 )*power(10,index)+((state/power(10,index))%10-(state/power(10,index + 1))%10)*power(10, index + 1), index + 1);
			return ret;
		}


		// Move functions for informed search

		board* moveUp(int d, int h){
			board* copy = new board(state,index,d,h);
			if(index - 3 < 0){ return copy; }
			board* ret = new board(state+((state/power(10,index - 3))%10-(state/power(10,index))%10 )*power(10,index)+((state/power(10,index))%10-(state/power(10,index - 3))%10)*power(10, index - 3), index - 3);
			return ret;
		}

		board* moveDown(int d, int h){
			board* copy = new board(state,index,d,h);
			if(index + 3 > 8){ return copy; }
			board* ret = new board(state+((state/power(10,index + 3))%10-(state/power(10,index))%10 )*power(10,index)+((state/power(10,index))%10-(state/power(10,index + 3))%10)*power(10, index + 3), index + 3);
			return ret;
		}

		board* moveLeft(int d, int h){
			board* copy = new board(state,index,d,h);
			if(index%3 == 0){ return copy; }
			board* ret = new board(state+((state/power(10,index - 1))%10-(state/power(10,index))%10 )*power(10,index)+((state/power(10,index))%10-(state/power(10,index - 1))%10)*power(10, index - 1), index - 1);
			return ret;
		}

		board* moveRight(int d, int h){
			board* copy = new board(state,index,d,h);
			if( (index + 1)%3 == 0){ return copy; }
			board* ret = new board(state+((state/power(10,index + 1))%10-(state/power(10,index))%10 )*power(10,index)+((state/power(10,index))%10-(state/power(10,index + 1))%10)*power(10, index + 1), index + 1);
			return ret;
		}



		// Solver functions using graph-search


		// Uninformed search

		// Solve using dfs
		board* dfsSolve(int goal){
			board* copy = new board(state, index);
			stack<board*> frontier;
			frontier.push(copy);
			set<int> visited;
			while(!frontier.empty()){
				board* cur = frontier.top();
				cur -> printBoard();
				frontier.pop();
				visited.insert(cur -> getState());

				// Check if goal has been reached
				if(cur -> getState() == goal){ return cur; }

				// Add adjacent nodes to the frontier
				set<int>::iterator it;

				// Move up
				board* insertUp = cur -> moveUp();
				it = visited.find(insertUp -> getState());
				if(!(cur -> sameState(insertUp)) && it == visited.end() ){
					frontier.push(insertUp);
				}

				// Move down
				board* insertDown = cur -> moveDown();
				it = visited.find(insertDown -> getState());
				if(!(cur -> sameState(insertDown)) && it == visited.end() ){
					frontier.push(insertDown);
				}

				// Move left
				board* insertLeft = cur -> moveLeft();
				it = visited.find(insertLeft -> getState());
				if(!(cur -> sameState(insertLeft)) && it == visited.end() ){
					frontier.push(insertLeft);
				}

				// Move right
				board* insertRight = cur -> moveRight();
				it = visited.find(insertRight -> getState());
				if(!(cur -> sameState(insertRight)) && it == visited.end() ){
					frontier.push(insertRight);
				}

				free(cur);
			}
			return NULL;
		}

		// Solve using  bfs
		board* bfsSolve(int goal){
			board* copy = new board(state, index);
			queue<board*> frontier;
			frontier.push(copy);
			set<int> visited;
			while(!frontier.empty()){
				board* cur = frontier.front();
				cur -> printBoard();
				frontier.pop();
				visited.insert(cur -> getState());

				// Check if goal has been reached
				if(cur -> getState() == goal){ return cur; }

				// Add adjacent nodes to the frontier
				set<int>::iterator it;

				// Move up
				board* insertUp = cur -> moveUp();
				it = visited.find(insertUp -> getState());
				if(!(cur -> sameState(insertUp)) && it == visited.end() ){
					frontier.push(insertUp);
				}

				// Move down
				board* insertDown = cur -> moveDown();
				it = visited.find(insertDown -> getState());
				if(!(cur -> sameState(insertDown)) && it == visited.end() ){
					frontier.push(insertDown);
				}

				// Move left
				board* insertLeft = cur -> moveLeft();
				it = visited.find(insertLeft -> getState());
				if(!(cur -> sameState(insertLeft)) && it == visited.end() ){
					frontier.push(insertLeft);
				}

				// Move right
				board* insertRight = cur -> moveRight();
				it = visited.find(insertRight -> getState());
				if(!(cur -> sameState(insertRight)) && it == visited.end() ){
					frontier.push(insertRight);
				}

				free(cur);
			}
			return NULL;
		}

		// Solve using Uniform-Cost Search
		private:
			struct CompUCS{
				bool operator()(const board* b1, const board* b2){
					return (b1 -> distance) > (b2 -> distance);
				}
			};

		public:
			board* uniformCostSearchSolve(int goal){
				board* copy = new board(state, index);
				vector<board*> frontier;
				frontier.push_back(copy);
				make_heap(frontier.begin(), frontier.end(), CompUCS());
				set<int> visited;
				while(!frontier.empty()){
					pop_heap(frontier.begin(), frontier.end(), CompUCS());
					board* cur = frontier.back();
					cur -> printBoard();
					frontier.pop_back();
					visited.insert(cur -> getState());

					// Check if goal has been reached
					if(cur -> getState() == goal){ return cur; }

					// Add adjacent nodes to the frontier
					set<int>::iterator it;

					// Move up
					board* insertUp = cur -> moveUp((cur -> getDistance()) + 1, 0);
					it = visited.find(insertUp -> getState());
					if(!(cur -> sameState(insertUp)) && it == visited.end() ){
						frontier.push_back(insertUp); push_heap(frontier.begin(), frontier.end(), CompUCS());
					}

					// Move down
					board* insertDown = cur -> moveDown((cur -> getDistance()) + 1, 0);
					it = visited.find(insertDown -> getState());
					if(!(cur -> sameState(insertDown)) && it == visited.end() ){
						frontier.push_back(insertDown); push_heap(frontier.begin(), frontier.end(), CompUCS());
					}

					// Move left
					board* insertLeft = cur -> moveLeft((cur -> getDistance()) + 1, 0);
					it = visited.find(insertLeft -> getState());
					if(!(cur -> sameState(insertLeft)) && it == visited.end() ){
						frontier.push_back(insertLeft); push_heap(frontier.begin(), frontier.end(), CompUCS());
					}

					// Move right
					board* insertRight = cur -> moveRight((cur -> getDistance()) + 1, 0);
					it = visited.find(insertRight -> getState());
					if(!(cur -> sameState(insertRight)) && it == visited.end() ){
						frontier.push_back(insertRight); push_heap(frontier.begin(), frontier.end(), CompUCS());
					}

					free(cur);
				}
				return NULL;
			}

		// Informed search

		private:
			// Auxiliary function to calculate heuristic
			int manhattanDistanceSum(int goal){
				int sum = 0;
				int positions[][2] = {{0,0},{0,1},{0,2},{1,0},{1,1},{1,2},{2,0},{2,1},{2,2}};
				for(int i = 0; i<9; i++){
					int s = state;
					for(int j = 0; j<9; j++){
						if(goal%10 == s%10){ sum += abs(positions[i][0]-positions[j][0]) + abs(positions[i][1]-positions[j][1]);}
						s /= 10;
					}
					goal /= 10;
				}
				return sum;
			}

			struct CompGBF{
				bool operator()(const board* b1, const board* b2){
					return (b1 -> heuristic) > (b2 -> heuristic);
				}
			};

			struct CompAS{
				bool operator()(const board* b1, const board* b2){
					return (b1 -> distance + b1 -> heuristic) > (b2 -> distance + b2 -> heuristic);
				}
			};


		public:

			// Solve using Greedy Best-First Search
			board* greedyBestFirstSolve(int goal){
				board* copy = new board(state, index);
				vector<board*> frontier;
				frontier.push_back(copy);
				make_heap(frontier.begin(), frontier.end(), CompGBF());
				set<int> visited;
				while(!frontier.empty()){
					pop_heap(frontier.begin(), frontier.end(), CompGBF());
					board* cur = frontier.back();
					cur -> printBoard();
					frontier.pop_back();
					visited.insert(cur -> getState());

					// Check if goal has been reached
					if(cur -> getState() == goal){ return cur; }

					// Add adjacent nodes to the frontier
					set<int>::iterator it;

					// Move up
					board* insertUp = cur -> moveUp(0, cur -> moveUp() -> manhattanDistanceSum(goal));
					it = visited.find(insertUp -> getState());
					if(!(cur -> sameState(insertUp)) && it == visited.end() ){
						frontier.push_back(insertUp); push_heap(frontier.begin(), frontier.end(), CompGBF());
					}

					// Move down
					board* insertDown = cur -> moveDown(0, cur -> moveDown() -> manhattanDistanceSum(goal));
					it = visited.find(insertDown -> getState());
					if(!(cur -> sameState(insertDown)) && it == visited.end() ){
						frontier.push_back(insertDown); push_heap(frontier.begin(), frontier.end(), CompGBF());
					}

					// Move left
					board* insertLeft = cur -> moveLeft(0, cur -> moveDown() -> manhattanDistanceSum(goal));
					it = visited.find(insertLeft -> getState());
					if(!(cur -> sameState(insertLeft)) && it == visited.end() ){
						frontier.push_back(insertLeft); push_heap(frontier.begin(), frontier.end(), CompGBF());
					}

					// Move right
					board* insertRight = cur -> moveRight(0, cur -> moveRight() -> manhattanDistanceSum(goal));
					it = visited.find(insertRight -> getState());
					if(!(cur -> sameState(insertRight)) && it == visited.end() ){
						frontier.push_back(insertRight); push_heap(frontier.begin(), frontier.end(), CompGBF());
					}
					free(cur);
				}
				return NULL;
			}


			// Solve using A*
			board* aStarSolve(int goal){
				board* copy = new board(state, index);
				vector<board*> frontier;
				frontier.push_back(copy);
				make_heap(frontier.begin(), frontier.end(), CompAS());
				set<int> visited;
				while(!frontier.empty()){
					pop_heap(frontier.begin(), frontier.end(), CompAS());
					board* cur = frontier.back();
					cur -> printBoard();
					frontier.pop_back();
					visited.insert(cur -> getState());

					// Check if goal has been reached
					if(cur -> getState() == goal){ return cur; }

					// Add adjacent nodes to the frontier
					set<int>::iterator it;

					// Move up
					board* insertUp = cur -> moveUp((cur -> getDistance()) + 1, cur -> moveUp() -> manhattanDistanceSum(goal));
					it = visited.find(insertUp -> getState());
					if(!(cur -> sameState(insertUp)) && it == visited.end() ){
						frontier.push_back(insertUp); push_heap(frontier.begin(), frontier.end(), CompAS());
					}

					// Move down
					board* insertDown = cur -> moveDown((cur -> getDistance()) + 1, cur -> moveDown() -> manhattanDistanceSum(goal));
					it = visited.find(insertDown -> getState());
					if(!(cur -> sameState(insertDown)) && it == visited.end() ){
						frontier.push_back(insertDown); push_heap(frontier.begin(), frontier.end(), CompAS());
					}

					// Move left
					board* insertLeft = cur -> moveLeft((cur -> getDistance()) + 1, cur -> moveDown() -> manhattanDistanceSum(goal));
					it = visited.find(insertLeft -> getState());
					if(!(cur -> sameState(insertLeft)) && it == visited.end() ){
						frontier.push_back(insertLeft); push_heap(frontier.begin(), frontier.end(), CompAS());
					}

					// Move right
					board* insertRight = cur -> moveRight((cur -> getDistance()) + 1, cur -> moveRight() -> manhattanDistanceSum(goal));
					it = visited.find(insertRight -> getState());
					if(!(cur -> sameState(insertRight)) && it == visited.end() ){
						frontier.push_back(insertRight); push_heap(frontier.begin(), frontier.end(), CompAS());
					}

					free(cur);
				}
				return NULL;
			}

		void printBoard(){
			int tmp = state;
			int toPrint = tmp%10;

			// Digit 9 is used to represent the empty position
			if(toPrint == 9){cout << "  ";}
			else{cout << toPrint << " ";}
			for(int i = 2; i<=9; i++){
				tmp /= 10;
				toPrint = tmp%10;
				if(toPrint == 9){cout << "  ";}
				else{cout << toPrint << " ";}
				if(i%3 == 0){cout << '\n';}
			}
			cout << "\n";
		}

		void shuffleBoard(){
			srand(time(NULL));
			for(int i = 0; i<(100+(rand()%(1337-100+1))); i++){
				int move = (1+(rand()%(4-1+1)));
				if(move == 1){*this = *this -> moveUp();}
				if(move == 2){*this = *this -> moveDown();}
				if(move == 3){*this = *this -> moveLeft();}
				if(move == 4){*this = *this -> moveRight();}
			}
		}


};

int readOption(int *op){
	long toInt;
	char buffer[1024];

	if(fgets(buffer, 1024, stdin) != NULL){
		size_t len = strlen(buffer);
		if(len > 0 && buffer[len-1] != '\n'){
			int clear;
			while((clear = getchar()) != '\n' && clear != EOF);
		}
	}
	else{ printf("\n"); exit(1);}
	char* end; errno = 0;
	toInt = strtol(buffer, &end, 10);
	if(errno == ERANGE){ return 0; }
	if(end == buffer){ return 0; }
	if(*end && *end != '\n'){ return 0; }
	if(toInt > INT_MAX || toInt < INT_MIN){ return 0; }
	*op = (int)toInt;
	return 1;
}

int main(){

	board* b = new board(987654321,8);
	b -> shuffleBoard();
	cout << "--------------------------------------" << endl;
	cout << "               8 Puzzle" << endl;
	cout << "--------------------------------------" << endl;
	cout << "Randomly generated starting state:" << endl;
	b -> printBoard();
	cout << "1) Solve with DFS" << endl;
	cout << "2) Solve with BFS" << endl;
	cout << "3) Solve with Uniform-Cost Search" << endl;
	cout << "4) Solve with Greedy Best-First Search" << endl ;
	cout << "5) Solve with A*" << endl;
	cout << "0) Quit" << endl;
	cout << "> ";
	int op = 0; readOption(&op);
	switch(op){
		case 1:
			b -> dfsSolve(987654321);
			break;

		case 2:
			b -> bfsSolve(987654321);
			break;

		case 3:
			b -> uniformCostSearchSolve(987654321);
			break;

		case 4:
			b -> greedyBestFirstSolve(987654321);
			break;

		case 5:
			b -> aStarSolve(987654321);
			break;

		case 0:
			exit(0);

		default:
			puts("Invalid Option");
	}
	return 0;
}
