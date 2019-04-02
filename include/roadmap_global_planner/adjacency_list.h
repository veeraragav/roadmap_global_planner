
#include <iostream>
#include <vector>
#include <cstdlib>

 
namespace roadmap_global_planner {


struct AdjListNode{
    int x;
    int y;
};

struct AdjList{

    std::vector<AdjListNode> list;
};



struct Graph{

    std::vector<AdjList> graph;
    
};

}