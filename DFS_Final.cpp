#include<bits/stdc++.h>

using namespace std;

//Class that holds the node members
class Node
{
    public:
    int row;
    int col;
    Node* parent;
};

//Class is created for exploration, and to have the code in a structured way
class Explore
{
    public:
    //This function simply checks for boundaries in the grid
    bool checkBounds(int row, int col, int size_row, int size_col)
    {
        if(row >= size_row || col >= size_col || row < 0 || col < 0)
        {
            return false;
        }
        return true;
    }

    //This is the function that has the algorithm implemented
    pair<Node*, vector<vector<bool>>> DFS(vector<vector<int>> grid, int* start, int* goal)
    {
        //Grid size is extracted
        int size_row = grid.size();
        int size_col = grid[0].size();
        //A stack to add the elements to be explored
        stack<pair<int, int>> explore;
        //A stack to add nodes to be explored. This is needed to keep a track of nodes since 
        //the algorithm is implemented using a grid and not a graph
        stack<Node*> node;
        //Order of exploration in reverse order
        //vector<pair<int, int>> directions {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}; 
        vector<pair<int, int>> directions {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
        //Flag for checking boundaries
        bool isValid{false};
        pair<Node*, vector<vector<bool>>> output;
        //To maintain the list of explored/visited nodes
        vector<vector<bool>> visited(size_row, vector<bool> (size_col));
        Node* parent = nullptr;
        Node* current = nullptr;
        parent = new Node();
        pair<int, int> current_coords;
        pair<int, int> goal_coords {goal[0], goal[1]};
        
        //Loops are used in case they are required for other inputs where there
        //can be multiple start points. They are not required in this case but loops are to 
        //generalize the code
        for(int i = start[0]; i < size_row; i++)
        {
            for(int j = start[1]; j < size_col; j++)
            {
                current_coords = {i, j};
                //Check for obsracles and if the cell is already visited or not
                if(grid[i][j] == 0 && !visited[i][j])
                {
                    explore.push({i, j});
                    node.push(parent);
                    visited[i][j] = true;
                    //Run the iterative steps till goal is reached or the grid is explored
                    //Here, it means that the stack has to be emptied
                    while(!explore.empty())
                    {
                        pair<int, int> coords = explore.top();
                        parent = node.top();
                        //Pop a cell to explore the neighbors
                        explore.pop();
                        node.pop();
                        //Mark as visited
                        visited[coords.first][coords.second] = true;
                        //Explore neighbors in all directions
                        for(pair<int, int> dir : directions)
                        {
                            //Get row and col
                            int row = coords.first + dir.first;
                            int col = coords.second + dir.second;
                            //Check if goal is reached
                            if(row == goal[0] && col == goal[1])
                            {
                                //Form the goal node as we need to count it
                                current = new Node();
                                current->row = row;
                                current->col = col;
                                current->parent = parent;
                                visited[row][col] = true;
                                goto here;
                            }  
                            //Check for bounds                         
                            isValid = checkBounds(row, col, size_row, size_col);
                            //Check for obstacles and if the cell is visited or not
                            if(isValid && !visited[row][col] && grid[row][col] == 0)
                            {
                                //For a new node
                                current = new Node();
                                current->parent = parent;
                                current->row = row;
                                current->col = col;
                                //Add in the stack
                                explore.push({row, col});
                                node.push(current);                               
                                isValid = false;    
                            }
                        }
                    }
                }
            }
        }
        //Goal not found
        return output;
        
        here:
        //Goal node is current node
        
        output.first = current;
        output.second = visited;
        return output;
    }

};


int main()
{
    //Add the grid here
    vector<vector<int>> grid 
/*  
                        {{0,0,0,1},
                        {0,0,0,1},
                        {0,1,0,0},
                        {0,0,0,0}};
 */                       

                       {{0,0,0,0,0,0,0,0,1,0},
                        {0,1,1,1,1,0,0,0,1,0},
                        {0,0,0,0,0,0,0,0,1,0},
                        {0,0,0,1,1,1,1,1,1,0},
                        {0,1,0,1,0,0,0,0,0,0},
                        {0,1,0,1,0,1,1,1,1,0},
                        {0,1,0,0,0,1,1,1,1,0},
                        {0,1,1,1,0,0,0,1,1,0},
                        {0,0,0,0,0,0,0,1,0,0},
                        {0,0,0,0,0,0,0,0,0,0}};


    int start[2] {0, 0};
    int goal[2] {0, 9};
    //steps is a variable that tells us the number of cells that were traversed
    //to reach the goal. This is different from the python code presented
    int steps{1};
    //Path contains a list of all the cells that are traversed to reach the goal
    vector<vector<int>> path;
    Node* output = nullptr;
    vector<vector<bool>> visited;
    //This variable is made since we cannot return more than 1 value at a time
    //A pair was made to return the path and the nodes visited 
    pair<Node*, vector<vector<bool>>> output_pair;
    Explore Ex;
    int size_row = grid.size();
    int size_col = grid[0].size();
    //Check if goal is in the grid
    if(goal[0] >= size_row || goal[1] >= size_col)
    {
        cout << "Invalid";
    }
    //Check if goal is obstacle
    else if(grid[goal[0]][goal[1]] == 1)
    {
        cout << "Goal is obtacle";
    }
    else
    {
        output_pair = Ex.DFS(grid, start, goal);
        output = output_pair.first;
        visited = output_pair.second;
        //Check if output is empty
        if(output == 0)
        {
            cout << "Goal not found";
        }
        else
        {
            //Loop to extract the path from the list of nodes that was formed
            while(output->parent != 0)
            {
                int row = output->row;
                int col = output->col;
                vector<int> coord {row, col};
                path.push_back(coord);
                steps++;
                output = output->parent;
            }
            //Add the start coords as in the path
            int row = start[0];
            int col = start[1];
            vector<int> coord {row, col};
            path.push_back(coord);
            //Reverse the path since we used push_back
            reverse(path.begin(), path.end());
            //nodes_visited is the 'steps' in the python code
            int nodes_visited{0};
            //Extract the number of nodes visited from the list
            for(int i = 0; i < visited.size(); i++)
            {
                for(int j = 0; j < visited[0].size(); j++)
                {
                    if(visited[i][j] == 1)
                    {
                        nodes_visited++;
                    }
                }
            }
            //Add a breakpoint here and debug to get the output
            cout << "Done";
        }
        
    }
}