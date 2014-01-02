#ifndef ReMod3D_Project3_Control_Solution_h
#define ReMod3D_Project3_Control_Solution_h

/*
 * Keyboard Map
 * See Camera.h for detailed camera control information
 * Buttons 1,2,3,4,5,6,w,a,s,d,(up arrow),(down arrow),(left arrow),(right arrow) control the camera
 * Holding down the left mouse button and moving the mouse manipulates the orientation of the camera
 * Button p sets program pause state (pause/unpause)
 * Button f switches between Wheelbot view and global view (NOTE: Color blob sensing will not work in global view, and the camera cannot be manually
 * moved in Wheelbot view)
 */


////PROJECT 3, LEVEL 1, Change number of wumpuses and number of pits in Project3.cpp to 0
////PROJECT 3, LEVEL 2, Change number of wumpuses to 0 and number of pits to > 0 in Project3.cpp
////PROJECT 3, LEVEL 3, Change number of wumpuses to > 0 and number of pits to > 0 in Project3.cpp.
////PROJECT 3, LEVEL 4, Change number of wumpuses to > 0 and number of pits to > 0 and pass true for shootingNoise in Project3.cpp
////PROJECT 3, POSSIBLE LEVEL 4, TBD


/* ----------------------------------------- DO NOT MODIFY ------------------------------------------------------*/
#include "ModuleProgram.h"
#include "WheelbotModule.h"
#include "SimulationUtility.h"
#include "ActionLog.h"
#include <map>
#include <queue>
#include <utility>
#include <stack>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/math/distributions/normal.hpp>
using namespace std;

//Possible Wheelbot actions
enum WheelbotActions {
    ACTION_FORWARD = 0,
    ACTION_BACKWARD,
    ACTION_LEFT,
    ACTION_RIGHT,
    ACTION_STOP,
    ACTION_SHOOT
};

//Structure for storing Wumpus information
struct Wumpus {
    const char *name;
    int index;
};

//Data structure for rendering laser shooting
struct RenderLine {
    PxVec3 p0;
    PxVec3 p1;
    int lifeTime;
    
};

//Graph data structure
class Graph {
public:
    vector<PxVec3> graphPoints;
    vector<pair<int, int> > graphEdges;
    Graph(vector<PxVec3> p, vector<pair<int, int> >  e) {
        this->graphPoints = p;
        this->graphEdges = e;
    }
    Graph(){}
};

//Node representing a search node in A*
class Node {
public:
    //Give each node an integer to represent the state
    int state;
    //Pointer to the parent (for reconstructing solution path)
    Node *parent;
    int parentState;
    //Actions available to this node
    vector<int> actions;
    int currentAction;
    //Action directions
    vector<PxVec2> actionDirections;
    //Heuristic value, cost along optimal path, and fVal (adds heuristic value and cost)
    double hVal;
    double fVal;
    double gVal;
    
    //Constructor
    Node (int s, Node * p, vector<int> a, double h, double g) {
        this->state = s;
        this->parent = p;
        this->actions = a;
        this->hVal = h;
        this->gVal = g;
        this->fVal = h + g;
    }
    Node (int s, int p, vector<PxVec2> a, double h, double g) {
        this->state = s;
        this->parentState = p;
        this->actionDirections = a;
        this->hVal = h;
        this->gVal = g;
        this->fVal = h + g;
        this->currentAction = 0;
    }
};


//Compares two Node fVals. We want lower fVals at the front of the priority queue
class CompareNode {
public:
    bool operator()(Node * n1, Node * n2)
    {
        if (n1->fVal > n2->fVal) return true;
        return false;
    }
};

class Project3_Control:rm3d::ai::ModuleProgram<rm3d::ai::WheelbotDock *> {
private:
    //Simulator base object and Wheelbot world model
    WheelbotSimBase* simulator;
    rm3d::ai::WheelbotModel *model;
    //Obstacle material
    PxMaterial *obMaterial;
    //Radius around goal specifying when we have arrived at a goal
    static PxReal currentGoalRadius;
    //Magnitude of wheel velocities for movements
    PxReal velocityMagnitude;
    //start node
    static int start;
    //Minimum distance between nodes
    static float minDistance;
    //goal node
    static int goal;
    //Input Graph
    static Graph g;
    //Learned Graph, initially empty
    static Graph learnedG;
    //Current position of Wheelbot
    static PxVec3 currentWheelbotPosition;
    //A plan of points to visit
    static vector<int> plan;
    //Pits in the graph
    static vector<int> graphPits;
    //Actions from each state
    static vector<vector<int> > actions;
    //Winds associated with each edge
    static vector<float> graphWinds;
    //Wumpuses on the graph
    static vector<Wumpus> graphWumpuses;
    //Smells associated with each edge
    static vector<float> graphSmell;
    //Currently adding noise to shooting
    static bool shootingNoise;
    //Distribution for shoot action sampling
    static boost::mt19937 rngShoot;
    static boost::normal_distribution<> shootDistribution;
    static boost::variate_generator< boost::mt19937, boost::normal_distribution<> > shootResampler;
    static vector<RenderLine> renderLines;
	PxVec3 lastNodePosition;
	PxVec2 B;
	PxVec3 nextNode;
	bool underAction;
	bool backTrack;
	bool allExplored;
	int BTIndex;
	int backTrackVia;
	vector<Node *> backtrackPlan;

	PxVec3 currentParent;
	int currentParentIndex;
	int currentIndex;

	map<int,set<int>> toBeExplored;
	map<int,int> dfsParent;

	set<int> pits;
	int currentActionIndex;

public:
    
    
    //Helpful rendering of paths, map nodes, pits, wumpuses, smells, and winds, as well as the learned map
    static void render() {
        //Render raycasts for a certain number of iterations (otherwise, they are just blips and hard to see)
        vector<int> deleteIndices;
        for (int i = 0; i<renderLines.size(); i++) {
            if (renderLines[i].lifeTime > 0) {
                rm3d::Renderer::DrawLine(renderLines[i].p0, renderLines[i].p1, rm3d::Color(0,0,1));
                renderLines[i].lifeTime = renderLines[i].lifeTime - 1;
            } else {
                deleteIndices.push_back(i);
            }
        }
        
        int sizeRLines = renderLines.size();
        for (int i=0; i<deleteIndices.size(); i++) {
            renderLines.erase(renderLines.begin() + deleteIndices[i]);
        }
        
        renderLines.resize(sizeRLines - deleteIndices.size());
        for (int i=0; i<g.graphPoints.size(); i++) {
            bool isPit = false;
            bool isWumpus = false;
            for (int j=0; j<graphPits.size(); j++) {
                if (graphPits[j] == i) {
                    isPit = true;
                    break;
                }
            }
            for (int j=0; j<graphWumpuses.size(); j++) {
                if (graphWumpuses[j].index == i) {
                    isWumpus = true;
                    break;
                }
            }
            
            
            //Draw the map nodes with colors corresponding to regular node (blue), pit (pink), wumpus (black) and current node (light blue)
            if ((currentWheelbotPosition - g.graphPoints[i]).magnitude() <= currentGoalRadius + 0.05) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,1,1));
            } else if (isPit) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(148.0/256.0,0,211.0/256.0));
            } else if (isWumpus){
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,0,0));
            } else {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,0,1));
            }
            
        }
        
        //Draw the map edges (smelly edges black, pit edges red)
        for (int j=0; j<g.graphEdges.size(); j++) {
            if (graphWinds[j] > 0.0) {
                rm3d::Renderer::DrawLine(PxVec3(g.graphPoints[g.graphEdges[j].first].x,
                                            g.graphPoints[g.graphEdges[j].first].y + 0.05,
                                            g.graphPoints[g.graphEdges[j].first].z),
                                     PxVec3(g.graphPoints[g.graphEdges[j].second].x,
                                            g.graphPoints[g.graphEdges[j].second].y + 0.05,
                                            g.graphPoints[g.graphEdges[j].second].z), rm3d::Color(1,0,0));
            } else if (graphSmell[j] > 0.0) {
                rm3d::Renderer::DrawLine(PxVec3(g.graphPoints[g.graphEdges[j].first].x,
                                                g.graphPoints[g.graphEdges[j].first].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].first].z),
                                         PxVec3(g.graphPoints[g.graphEdges[j].second].x,
                                                g.graphPoints[g.graphEdges[j].second].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].second].z), rm3d::Color(0,0,0));
            } else {
                rm3d::Renderer::DrawLine(PxVec3(g.graphPoints[g.graphEdges[j].first].x,
                                                g.graphPoints[g.graphEdges[j].first].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].first].z),
                                         PxVec3(g.graphPoints[g.graphEdges[j].second].x,
                                                g.graphPoints[g.graphEdges[j].second].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].second].z), rm3d::Color(0,1,1));
            }
        }
        
        //Draw the learned map nodes and edges in a similar way
        for (int i=0; i<learnedG.graphPoints.size(); i++) {
            if ((currentWheelbotPosition - learnedG.graphPoints[i]).magnitude() <= currentGoalRadius + 0.05) {
                rm3d::Renderer::DrawSphere(PxTransform(PxVec3(learnedG.graphPoints[i].x,
                                                              learnedG.graphPoints[i].y + 0.05,
                                                              learnedG.graphPoints[i].z)), 0.2, rm3d::Color(1,1,0));
            } else {
                rm3d::Renderer::DrawSphere(PxTransform(PxVec3(learnedG.graphPoints[i].x,
                                                              learnedG.graphPoints[i].y + 0.05,
                                                              learnedG.graphPoints[i].z)), 0.2, rm3d::Color(1,0,0));
            }
            
        }
        for (int j=0; j<learnedG.graphEdges.size(); j++) {
            rm3d::Renderer::DrawLine(PxVec3(learnedG.graphPoints[learnedG.graphEdges[j].first].x,
                                            learnedG.graphPoints[learnedG.graphEdges[j].first].y + 0.1,
                                            learnedG.graphPoints[learnedG.graphEdges[j].first].z),
                                     PxVec3(learnedG.graphPoints[learnedG.graphEdges[j].second].x,
                                            learnedG.graphPoints[learnedG.graphEdges[j].second].y + 0.1,
                                            learnedG.graphPoints[learnedG.graphEdges[j].second].z), rm3d::Color(0,1,0));
        }
        
    }
    
    //Constructor
    Project3_Control(WheelbotSimBase* simulator, Graph graph, int startNode, int goalNode, vector<int> pits, vector<float> winds,
                     vector<Wumpus> wumpuses, vector<float> smell, bool shootNoise, float minDistanceBetween) {
        graphPits = pits;
        graphWinds = winds;
        graphSmell = smell;
        shootingNoise = shootNoise;
        graphWumpuses = wumpuses;
        minDistance = minDistanceBetween;
        obMaterial = simulator->getSimulationPhysics()->createMaterial(0.5, 0.5, 0.5);
        this->programCounter = 0;
        this->actionQueue = new rm3d::ai::ActionLog*[4];
        this->simulator = simulator;
        this->actionQueue[0] = NULL;
        this->actionQueue[1] = NULL;
        this->actionQueue[2] = NULL;
        this->actionQueue[3] = NULL;
        this->velocityMagnitude = 15.0;
        start = startNode;
        goal = goalNode;
        g = graph;
		underAction = false;
		backTrack = false;
		allExplored = false;
		BTIndex = 1;
						
		currentParentIndex = -1;
		currentIndex = 0;

        //Compute actions for each state
        for (int i=0; i<g.graphPoints.size(); i++) {
            actions.push_back(vector<int>());
            for (int j=0; j<graph.graphEdges.size(); j++) {
                if (g.graphEdges[j].first == i) {
                    actions[i].push_back(g.graphEdges[j].second);
                }
            }
        }
        
    }
    
    ~Project3_Control() {
        if (this->actionQueue[0] != NULL) delete this->actionQueue[0];
        if (this->actionQueue[1] != NULL) delete this->actionQueue[1];
        if (this->actionQueue[2] != NULL) delete this->actionQueue[2];
        if (this->actionQueue[3] != NULL) delete this->actionQueue[3];
        delete actionQueue;
    }
    
    rm3d::ai::ActionLog **getActionQueue() {
        return this->actionQueue;
    }
    rm3d::ai::Message<rm3d::ai::WheelbotDock*> **getMessageQueue() {
        return this->messageQueue;
    }
    rm3d::ai::RangedMessage** getRangedMessageQueue() {
        return this->rangedMessageQueue;
    }
    int getNumberOfRangedMessages() {
        return this->numRangedMessages;
    }
    int getNumberOfActions() {
        return this->numActions;
    }
    int getNumberOfMessages() {
        return this->numMessages;
    }
    
    //Sensor for current wheelbot pose
    PxTransform getCurrentWheelbotPose() {
        return boost::any_cast<PxTransform>(this->model->getCurrentState()->sensorValues[rm3d::ai::WheelbotModule::posOrientInt]);
    }
    
    //Sensor indicating whether or not we are at a node. If we are not at a node, we must be on an edge somewhere between nodes
    bool atANode() {
        for (int i=0; i<g.graphPoints.size(); i++) {
            if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                return true;
            }
        }
        return false;
    }
    
    //Returns the current node point
    PxVec3 getCurrentNode() {
        for (int i=0; i<g.graphPoints.size(); i++) {
            if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                return g.graphPoints[i];
            }
        }
        return PxVec3(-100000);
    }
    
    //Returns the actions (2D direction vectors) available to our robot at each node
    vector<PxVec2> getActionVectorsForCurrentNode() {
        vector<PxVec2> currentActions;
        bool atNode = atANode();
        if (atNode) {
            int node = -1;
            for (int i=0; i<g.graphPoints.size(); i++) {
                if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                    node = i;
                }
            }
            vector<int> actionsForNode = actions[node];
            for (int i=0; i<actionsForNode.size(); i++) {
                PxVec2 vec = PxVec2(g.graphPoints[actionsForNode[i]].x - getCurrentWheelbotPose().p.x,
                                    g.graphPoints[actionsForNode[i]].z - getCurrentWheelbotPose().p.z);
                vec.normalize();
                currentActions.push_back(vec);
                
            }
        }
        return currentActions;
    }
    
    //Returns the vector corresponding to the  given action at the given node in the learned graph
    PxVec2 getVectorForActionAtNode(int stateInLearnedGraph, int action) {
        PxVec2 actionVec;
        int node = -1;
        for (int i=0; i<g.graphPoints.size(); i++) {
            if ((learnedG.graphPoints[stateInLearnedGraph] - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                node = i;
            }
        }
        
        if (action == -1) {
            PxVec3 vec(g.graphPoints[node] - getCurrentWheelbotPose().p);
            actionVec = PxVec2(vec.x, vec.z);
        } else {
            PxVec3 vec(g.graphPoints[actions[node][action]] - getCurrentWheelbotPose().p);
            actionVec = PxVec2(vec.x, vec.z);
        }
        return actionVec;
    }
    
    //Returns the winds associated with the actions available at the current node
    vector<float> getCurrentWinds() {
        vector<float> currentWinds;
        bool atNode = atANode();
        if (atNode) {
            int node = -1;
            for (int i=0; i<g.graphPoints.size(); i++) {
                if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                    node = i;
                }
            }
            vector<int> actionsForNode = actions[node];
            for (int i=0; i<actionsForNode.size(); i++) {
                int edge = -1;
                for (int j=0; j<g.graphEdges.size(); j++) {
                    if (g.graphEdges[j].first == node && g.graphEdges[j].second == actionsForNode[i]) {
                        edge = j;
                    }
                }
                currentWinds.push_back(graphWinds[edge]);
                
            }
        }
        return currentWinds;
    }
    
    //Returns the smells associated with the actions available at the current node
    vector<float> getCurrentSmells() {
        vector<float> currentSmells;
        bool atNode = atANode();
        if (atNode) {
            int node = -1;
            for (int i=0; i<g.graphPoints.size(); i++) {
                if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                    node = i;
                }
            }
            vector<int> actionsForNode = actions[node];
            for (int i=0; i<actionsForNode.size(); i++) {
                int edge = -1;
                for (int j=0; j<g.graphEdges.size(); j++) {
                    if (g.graphEdges[j].first == node && g.graphEdges[j].second == actionsForNode[i]) {
                        edge = j;
                    }
                }
                currentSmells.push_back(graphSmell[edge]);
                
            }
        }
        return currentSmells;
    }
    
    //If we have killed the Wumpus
    void updateGraphOnWumpusDeath(Wumpus w) {
        vector<Wumpus> ws;
        for (int i=0; i<graphWumpuses.size(); i++) {
            if (graphWumpuses[i].index != w.index) {
                ws.push_back(graphWumpuses[i]);
            }
        }
        graphWumpuses.clear();
        graphWumpuses = ws;
        vector<int> actionsForNode = actions[w.index];
        for (int i=0; i<actionsForNode.size(); i++) {
            int edge = -1;
            for (int j=0; j<g.graphEdges.size(); j++) {
                if ((g.graphEdges[j].first == w.index && g.graphEdges[j].second == actionsForNode[i]) ||
                    (g.graphEdges[j].second == w.index && g.graphEdges[j].first == actionsForNode[i])) {
                    graphSmell[j] = 0.0;
                }
            }
            
        }
        for (int j =0; j<graphWumpuses.size(); j++) {
            for (int i=0; i<g.graphEdges.size(); i++) {
                if (g.graphEdges[i].first == graphWumpuses[j].index) {
                    graphSmell[i] = (g.graphPoints[g.graphEdges[i].first] - g.graphPoints[g.graphEdges[i].second]).magnitude();
                } else if (g.graphEdges[i].second == graphWumpuses[j].index) {
                    graphSmell[i] = (g.graphPoints[g.graphEdges[i].second] - g.graphPoints[g.graphEdges[i].first]).magnitude();
                }
            }
        }
    }
    
    //Make Wumpus run away
    void updateGraphOnWumpusFlee(Wumpus w) {
        vector<int> actionsForNode = actions[w.index];
        for (int i=0; i<actionsForNode.size(); i++) {
            int edge = -1;
            for (int j=0; j<g.graphEdges.size(); j++) {
                if ((g.graphEdges[j].first == w.index && g.graphEdges[j].second == actionsForNode[i]) ||
                    (g.graphEdges[j].second == w.index && g.graphEdges[j].first == actionsForNode[i])) {
                    graphSmell[j] = 0.0;
                }
            }
            
        }
        
        bool okNode = false;
        int itCounter = 0;
        int newIndex = -1;
        while (!okNode && itCounter < actionsForNode.size()) {
            okNode = true;
            newIndex = actionsForNode[rand() % actionsForNode.size()];
            
            for (int i=0; i<graphPits.size(); i++) {
                if (newIndex == graphPits[i]) {
                    okNode = false;
                }
            }
            
            for (int i=0; i<graphWumpuses.size(); i++) {
                if (newIndex == graphWumpuses[i].index) {
                    okNode = false;
                }
            }
            
            if ((g.graphPoints[newIndex] - getCurrentWheelbotPose().p).magnitude() < minDistance) {
                okNode = false;
            }
            
            itCounter++;
            
        }
        
        if (okNode) {
            cout<<"Moving From "<<w.index<<" to "<<newIndex<<endl;
            for (int i=0; i<graphWumpuses.size(); i++) {
                if (graphWumpuses[i].index == w.index) {
                    graphWumpuses[i].index = newIndex;
                }
            }
            
            vector<rm3d::ai::Obstacle> obs = simulator->getObstacles();
            for (int i=0; i<obs.size(); i++) {
                if (strcmp(obs[i].obstacleBody->getName(), w.name) == 0) {
                    PxRigidDynamic *rd = (PxRigidDynamic *)obs[i].obstacleBody;
                    rd->setGlobalPose(PxTransform(PxVec3(g.graphPoints[newIndex]))*PxTransform(PxVec3(0,1,0)));
                }
            }
        }
        
        for (int j =0; j<graphWumpuses.size(); j++) {
            for (int i=0; i<g.graphEdges.size(); i++) {
                if (g.graphEdges[i].first == graphWumpuses[j].index) {
                    graphSmell[i] = (g.graphPoints[g.graphEdges[i].first] - g.graphPoints[g.graphEdges[i].second]).magnitude();
                } else if (g.graphEdges[i].second == graphWumpuses[j].index) {
                    graphSmell[i] = (g.graphPoints[g.graphEdges[i].second] - g.graphPoints[g.graphEdges[i].first]).magnitude();
                }
            }
        }
    }
    
    //Explode robot if we reach too close to a pit or a Wumpus
    void checkForFailure() {
        bool fail = false;
        for (int i=0; i<graphPits.size(); i++) {
            if ((getCurrentWheelbotPose().p - g.graphPoints[graphPits[i]]).magnitude() <= currentGoalRadius) {
                fail = true;
            }
        }
        
        for (int i=0; i<graphWumpuses.size(); i++) {
            if ((getCurrentWheelbotPose().p - g.graphPoints[graphWumpuses[i].index]).magnitude() <= currentGoalRadius) {
                fail = true;
            }
        }
        
        if (fail) {
            rm3d::ai::WheelbotModule * wm = (rm3d::ai::WheelbotModule *)simulator->getModuleAtIndex(0);
            cout<<"Boom"<<endl;
            wm->getBody()->addForce(PxVec3(50,100,0));
        }
    }
    
    //Wheelbot action that can be taken.
    void takeAction(WheelbotActions action, PxVec2 direction = PxVec2(1,0), float distance = 1.0) {
        this->numActions = 4;
        if (action == ACTION_BACKWARD) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)velocityMagnitude);
        } else if (action == ACTION_FORWARD) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)-velocityMagnitude);
        } else if (action == ACTION_LEFT) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)-velocityMagnitude);
        } else if (action == ACTION_RIGHT) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)velocityMagnitude);
        } else if (action == ACTION_STOP) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)0);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)0);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)0);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)0);
        } else if (action == ACTION_SHOOT) {
            //Shoot at the wumpus. The direction is perturbed if noise is turned on, so you may miss. If you hit the wumpus, he dies. If you don't he moves to a new node (randomly)
            PxRaycastHit hit;
            const PxSceneQueryFlags outputFlags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
            PxVec3 origin = (PxTransform(getCurrentNode())*PxTransform(PxVec3(0,1.0,0))).p;
            PxVec3 direction3 = PxVec3(direction.x, 0, direction.y);
            direction3 += (shootingNoise ? PxVec3(shootResampler(), 0, shootResampler()) : PxVec3(0));
            direction3.normalize();
            bool status = simulator->getSimulationScene()->raycastSingle(origin, direction3, distance, outputFlags, hit);
            RenderLine rl;
            rl.p0 = origin;
            rl.p1 = (PxTransform(origin)*PxTransform(distance*direction3)).p;
            rl.lifeTime = 60;
            renderLines.push_back(rl);
            
            if (status) {
                cout<<"Hit wumpus"<<endl;
                vector<rm3d::ai::Obstacle> obs = simulator->getObstacles();
                for (int i=0; i<obs.size(); i++) {
                    if (strcmp(obs[i].obstacleBody->getName(), hit.shape->getActor().getName()) == 0) {
                        PxRigidDynamic *rd = (PxRigidDynamic *)obs[i].obstacleBody;
                        rd->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, false);
                        rd->addForce(PxVec3(10000, 5000, 0));
                    }
                }
                for (int i=0; i<graphWumpuses.size(); i++) {
                    if (strcmp(graphWumpuses[i].name, hit.shape->getActor().getName()) == 0) {
                        updateGraphOnWumpusDeath(graphWumpuses[i]);
                    }
                }
            } else {
                cout<<"Missed Wumpus"<<endl;
                for (int i=0; i<graphWumpuses.size(); i++) {
                    updateGraphOnWumpusFlee(graphWumpuses[i]);
                }
            }
        }
    }
    
    int getNumGraphNodes() {
        return g.graphPoints.size();
    }
    
    int getNumGraphEdges() {
        return g.graphEdges.size();
    }
    
    /* -------------------------------- END DO NOT MODIFY ------------------------------------------*/

    
    
    //Implementation of A* that you may choose to use or not use
    Node* AStar(int startNode, Graph graphObject) {
		//cout<<"\nJust Inside A Star";
        priority_queue<Node*, vector<Node*>, CompareNode> open;
        map<int, Node *> openCompare;
        map<int, Node *> closedCompare;
        map<int, double> openFVals;
        vector<Node*> closed;
        vector<vector<int> > actions(graphObject.graphPoints.size());
        //Compute actions for each state
        for (int i=0; i<graphObject.graphPoints.size(); i++) {
			actions[i] = vector<int>();

			if(pits.count(i)){
				continue;
			}
            
            for (int j=0; j<graphObject.graphEdges.size(); j++) {
				
                if (graphObject.graphEdges[j].first == i) {
					if(pits.count(graphObject.graphEdges[j].second)){
						continue;
					}
					else{
						actions[i].push_back(graphObject.graphEdges[j].second);
					}
                }
            }
        }

		/*
		cout<<"\n the dfs Parents:";
		for(int i = 0; i<graphObject.graphPoints.size(); i++){
			cout<<"\n Parent of "<<i<<" is ==>"<<dfsParent[i];
		}

		cout<<"\nThe Actions available to graph points in A Star";
		for(int i = 0; i<graphObject.graphPoints.size(); i++){
			cout<<"\nNode "<<i<<"==>";
			for(int j = 0; j< actions[i].size();j++){
				cout<<actions[i][j]<<", ";
			}

		}
        */
        Node *node = new Node(start, NULL, actions[start], (graphObject.graphPoints[start] - graphObject.graphPoints[goal]).magnitude(), 0);
        open.push(node);
        Node *solution = NULL;
        while (true) {
            if (open.empty()) {
                break;
            }
            Node *n = open.top();
            open.pop();
            if (n->state == goal) {
                solution = n;
            }
            for (int i=0; i<n->actions.size(); i++) {
                double h = (graphObject.graphPoints[n->actions[i]] - graphObject.graphPoints[goal]).magnitude();
                double stepCost = 0;
                stepCost += (graphObject.graphPoints[n->actions[i]] - graphObject.graphPoints[n->state]).magnitude();
                double g = stepCost + n->gVal;
                Node *child = new Node (n->actions[i], n, actions[n->actions[i]], h, g);
                if (closedCompare[child->state] == NULL && openCompare[child->state] == NULL) {
                    open.push(child);
                    openCompare[child->state] = child;
                    openFVals[child->state] = child->fVal;
                } else if (openCompare[child->state] != NULL && closedCompare[child->state] == NULL) {
                    if (child->fVal < openFVals[child->state]) {
                        //We must update the current fVal of this node
                        vector<Node *> openNodes;
                        while (!open.empty()) {
                            Node *compNode = open.top();
                            open.pop();
                            if (compNode->state != child->state) {
                                openNodes.push_back(compNode);
                            } else {
                                openNodes.push_back(child);
                            }
                        }
                        for (int j=0; j<openNodes.size(); j++) {
                            open.push(openNodes[j]);
                        }
                    }
                }
            }
            closed.push_back(n);
            closedCompare[n->state] = n;
        }

		//cout<<"\n***Right before A Star finishes";
        return solution;
    }
    
    vector<Node *> reconstructSolution(Node *s) {
        vector<Node *> solutionR, solution;
        while (s != NULL)  {
            solutionR.push_back(s);
            s = s->parent;
        }
        for (int i=solutionR.size() - 1; i>=0; i--) {
            solution.push_back(solutionR[i]);
        }
		//cout<<"\n***Right before Reconstruct solution finishes, solution size = "<<solution.size();
        return solution;
    }

	int backtrackTo(int Node){
		int destination = Node;

		while(toBeExplored[destination].size() == 0){
			if(destination == 0){
				break;
			}
			destination = dfsParent[destination];
		}

		return destination;
	}

	float distance2D(PxVec3 A, PxVec3 B){
		float distance = sqrt(pow(A.x-B.x,2) + pow(A.z-B.z,2));

		return distance;
	}

	//returns -1 of pit does not exist, otherwise the index of pit in learned graph
	int pitAlreadyExists(PxVec3 pitNode){
		int retval = -1;
		for(int i = 0; i< learnedG.graphPoints.size();i++){
			PxVec3 A = learnedG.graphPoints.at(i);
			if(distance2D(pitNode,A) < 3){
				retval = i;
				break;
			}
		}

		return retval;
	}

	void pitFound(int index, int currentIndex){
		PxVec3 currentNode = getCurrentNode();
		vector<PxVec2> actions = getActionVectorsForCurrentNode();
		vector<float> winds = getCurrentWinds();

		float theta_p = atan2(actions[index].x,actions[index].y);
		
		float pitX = winds[index]*sin(theta_p) +currentNode.x;
		float pitZ = winds[index]*cos(theta_p) +currentNode.z;
		PxVec3 pitNode = PxVec3(pitX,currentNode.y,pitZ);
		float pitDistance = distance2D(pitNode,currentNode);//sqrt(pow(pitNode.x-currentNode.x,2) + pow(pitNode.z-currentNode.z,2));
		/*
		cout<<"\nWind magnitude: "<<winds[index]<<", Calculated Pit Distance: "<< pitDistance;
		cout<<"\nTheta : "<<theta_p;
		cout<<"\nnodeX="<<currentNode.x<<", pitNodeX="<<pitNode.x<<", nodeZ="<<currentNode.z<<", pitNodeZ="<<pitNode.z; 
		*/

		// Check if the currently detected pit has already been identified
		int pitExists = pitAlreadyExists(pitNode);
		int pitIndex;
		if(pitExists == -1){
			cout<<"\nNew Pit Detected";
			learnedG.graphPoints.push_back(pitNode);
			pitIndex = learnedG.graphPoints.size()-1;
			pits.insert(pitIndex);			
		}
		else{
			cout<<"\nPit Already Detected in the past";
			pitIndex = pitExists;
		}

		//add edges to pit in the learnedG
		learnedG.graphEdges.push_back(pair<int, int>(pitIndex, currentIndex));
		learnedG.graphEdges.push_back(pair<int, int>(currentIndex, pitIndex));		

	}

	int getActionIndex(){
		//cout<<"\n***Inside getActionIndex***";
		int retval = 0;
		float min = PxPi;
		PxVec3 presentNode = getCurrentNode();
		vector<PxVec2> actions = getActionVectorsForCurrentNode();
		PxVec2 AB = PxVec2(nextNode.x - presentNode.x,nextNode.z - presentNode.z);
		AB.normalize();
		float mindist = sqrt(pow(AB.x-actions[0].x,2) + pow(AB.y-actions[0].y,2));
		for(int i = 1; i<actions.size(); i++){
			float tempdist = sqrt(pow(AB.x-actions[i].x,2) + pow(AB.y-actions[i].y,2));
			if(tempdist < mindist){
				mindist = tempdist;
				retval = i;
			}
		}
		/*
		float theta1 = atan2(AB.x,AB.y);
		float theta2,theta3;
		cout<<"\nAB-->"<<AB.x<<", "<<AB.y;
		for(int i = 0; i<actions.size(); i++){
			
			cout<<"\naction "<<i<<"-->"<<actions[i].x<<", "<<actions[i].y;
			theta2 = atan2(actions[i].x,actions[i].y);
			theta3 = theta2-theta1; // the angular difference between two vectors

			while (theta3 <= - PxPi)
				theta3 += 2 * PxPi;
        
			while (theta3 > PxPi)
				theta3 -= 2 * PxPi;

			if(fabs(theta3) < min){
				min = theta3;
				retval = i;
			}
		}
		*/
		return retval;
	}

    
    void step(void *model) {
        /* -------------------------------- DO NOT MODIFY ----------------------------------------------- */
        simulator->setCustomRenderFunction(render);
        this->model = (rm3d::ai::WheelbotModel *)model;
        currentWheelbotPosition = getCurrentWheelbotPose().p;
        //Housekeeping
        if (this->actionQueue[0] != NULL) delete this->actionQueue[0];
        if (this->actionQueue[1] != NULL) delete this->actionQueue[1];
        if (this->actionQueue[2] != NULL) delete this->actionQueue[2];
        if (this->actionQueue[3] != NULL) delete this->actionQueue[3];
        this->numActions = 4;
        //End Housekeeping
        checkForFailure();
        /* -------------------------------- END DO NOT MODIFY -------------------------------------------- */
        

		/*
        //In this project, you can take the action ACTION_SHOOT and a movement action simultaneously
        //When you shoot, the Wumpus hears it and moves randomly. You cannot count on the Wumpus being in the same place throughout your exploration!!!!!
        takeAction(ACTION_STOP);
        //For the shoot action, you give it a direction and a distance to shoot. You can shoot in any direction, no matter your orientation
        //When you come across a Wumpus, you will know how far to shoot by reading the magnitude of the smells.
        
        PxVec2 direction(shootResampler(), shootResampler());
        direction.normalize();
        if (this->programCounter % 60 == 0)takeAction(ACTION_SHOOT, direction, 5.0);
        cout<<"At A Node?: "<<(atANode() ? "True" : "False")<<endl;
        vector<PxVec2> actionsForCurrentNode = getActionVectorsForCurrentNode();
        vector<float> smells = getCurrentSmells();
        vector<float> winds = getCurrentWinds();
        for (int i=0; i<actionsForCurrentNode.size(); i++) {
            cout<<"Action "<<i<<": ("<<actionsForCurrentNode[i].x<<", "<<actionsForCurrentNode[i].y<<")"<<endl;
            cout<<"Wind "<<i<<": "<<winds[i]<<endl;
            cout<<"Smell "<<i<<": "<<smells[i]<<endl;
        }
        if (this->programCounter == 0){
            //Learn an initial node
            learnedG.graphPoints.push_back(getCurrentNode());
            //Learn a RANDOM new node and edge. NOTE: This is FOR ILLUSTRATION PURPOSES ONLY. When you code your solution, you need to learn the nodes and edges of the actual
            //map. When you learn a new node, use the getCurrentNode() function to place the node appropriately. You can only learn nodes that you actually visit (or can infer, in
            //the case of pits)
            learnedG.graphPoints.push_back((PxTransform(getCurrentNode())*PxTransform(PxVec3(5,0,0))).p);
            learnedG.graphEdges.push_back(pair<int, int>(1, 0));
            cout<<"Adding New Node At: ("<<getCurrentNode().x<<", "<<getCurrentNode().y<<", "<<getCurrentNode().z<<")"<<endl;
        }
		*/

		//***Code Starts Here***
		
		if(allExplored == true){
			//cout<<"\nSuccess: The whole map has been Explored";
			takeAction(ACTION_STOP);
			return;
		}	
		
		PxVec2 posX2D = PxVec2(getCurrentWheelbotPose().p.x, getCurrentWheelbotPose().p.z);
		PxVec3 posAlongX = (getCurrentWheelbotPose()*PxTransform(PxVec3(0.3,0,0))).p;
        PxVec2 posAlongX2D = PxVec2(posAlongX.x, posAlongX.z);
		PxVec2 A = posAlongX2D - posX2D;
		
		//cout<<"\nRight before getting current node index";
		//atANode() && underAction==false
		
		if(backTrack == false && (this->programCounter == 0 ||(atANode() && (getCurrentNode() != currentParent) )) ){
			
			PxVec3 currentNode = getCurrentNode();
			//int currentIndex;
			//First, add the node and edges to learnedG
			if(std::find(learnedG.graphPoints.begin(), learnedG.graphPoints.end(), currentNode) == learnedG.graphPoints.end()){
				// The current node is not in the learned graph
				cout<<"\nAdding a new Node to learned graph";
				learnedG.graphPoints.push_back(currentNode);
				currentIndex = learnedG.graphPoints.size()-1;
				if(currentParentIndex == -1){
					currentParentIndex = 0;						
					dfsParent[currentIndex] = -1;
				}
				
				else{
					learnedG.graphEdges.push_back(pair<int, int>(currentParentIndex, currentIndex));
					learnedG.graphEdges.push_back(pair<int, int>(currentIndex, currentParentIndex));
					dfsParent[currentIndex] = currentParentIndex;
					currentParentIndex = currentIndex;
				}
				currentParent = currentNode;
			
			
				currentIndex = std::find(learnedG.graphPoints.begin(), learnedG.graphPoints.end(), currentNode) - learnedG.graphPoints.begin();
				//Now add/modify the actions which haven't been explored from the current node

				vector<PxVec2> actionsForCurrentNode = getActionVectorsForCurrentNode();
				vector<float> winds = getCurrentWinds();
				vector<float> smells = getCurrentSmells();
				//toBeExplored[currentNode] = actionsForCurrentNode;
				if (this->programCounter == 0){
					for (int i=0; i<actionsForCurrentNode.size(); i++){
						//cout<<"\nAction "<<i<<": ("<<actionsForCurrentNode[i].x<<", "<<actionsForCurrentNode[i].y<<")"<<" winds[i]: "<<winds[i]<<" smells[i]: "<<smells[i];
						if(winds[i]>0){
							//call function to add pit node 
							pitFound(i,currentIndex);						
						}
						else{
							
							//cout<<"\nInserting "<<i<<" in "<<currentIndex;
							toBeExplored[currentIndex].insert(i);
						}
					}
				}
				else{
					for (int i=0; i<actionsForCurrentNode.size(); i++) {
						//cout<<"\nAction "<<i<<": ("<<actionsForCurrentNode[i].x<<", "<<actionsForCurrentNode[i].y<<")"<<" winds[i]: "<<winds[i]<<" smells[i]: "<<smells[i];
						if(winds[i]>0){
							//call function to add pit node 
							pitFound(i,currentIndex);
						
						}
						else{
							toBeExplored[currentIndex].insert(i);
							PxVec2 actionVector_i = actionsForCurrentNode[i];
							float theta_i = atan2(actionVector_i.x,actionVector_i.y);
							float thetaB = atan2(-B.x, -B.y);
							float thetaDiff = thetaB - theta_i;
		
							while (thetaDiff <= - PxPi)
								thetaDiff += 2 * PxPi;
        
							while (thetaDiff > PxPi)
								thetaDiff -= 2 * PxPi;
						
							PxReal angleError = thetaDiff;
						
							if(fabs(angleError) > 0.1 ){
								//cout<<"\nInserting "<<i<<" in "<<currentIndex;
							
							}
							else{
								//cout<<"\nSkipping "<<i<<" for "<<currentIndex;
								toBeExplored[currentIndex].erase(i);
							}
						}
					}
				}

				

				if(toBeExplored[currentIndex].size() != 0){
					set<int>::iterator nextActionNumber = (toBeExplored[currentIndex].begin());
					currentActionIndex = *nextActionNumber;
					B = actionsForCurrentNode[*nextActionNumber];
					//cout<<"\nB==> "<<B.x<<", "<<B.y<<"\n";
					toBeExplored[currentIndex].erase(nextActionNumber);
										
				}
				else{
					//cout<<"\n    Changing backTrack to true";
					backTrack = true;	
					backTrackVia = dfsParent[currentIndex];
				}
			}
			else{
				//Current node is already in the learned Graph
				cout<<"\nCurrent node is already in the learned Graph";
				currentIndex = std::find(learnedG.graphPoints.begin(), learnedG.graphPoints.end(), currentNode)- learnedG.graphPoints.begin();
				//cout<<"\nCurrent: "<<currentIndex<<", Parent: "<<currentParentIndex;
				learnedG.graphEdges.push_back(pair<int, int>(currentParentIndex, currentIndex));
				learnedG.graphEdges.push_back(pair<int, int>(currentIndex, currentParentIndex));

				vector<PxVec2> actionsForCurrentNode = getActionVectorsForCurrentNode();
				for (int i=0; i<actionsForCurrentNode.size(); i++) {
					//cout<<"\nAction "<<i<<": ("<<actionsForCurrentNode[i].x<<", "<<actionsForCurrentNode[i].y<<")";	
					
					PxVec2 actionVector_i = actionsForCurrentNode[i];
					float theta_i = atan2(actionVector_i.x,actionVector_i.y);
					float thetaB = atan2(-B.x, -B.y);
					float thetaDiff = thetaB - theta_i;
		
					while (thetaDiff <= - PxPi)
						thetaDiff += 2 * PxPi;
        
					while (thetaDiff > PxPi)
						thetaDiff -= 2 * PxPi;
						
					PxReal angleError = thetaDiff;
						
					if(fabs(angleError) < 0.1 ){
						//cout<<"\nRemoving "<<i<<" for "<<currentIndex;
						toBeExplored[currentIndex].erase(i);	
						break;
					}
				}

				//cout<<"\n    Changing backTrack to true";
				backTrack = true;
				backTrackVia = currentParentIndex;
			}


			if(backTrack == true){
				//cout<<"\n Inside the condition which calls A Star";
				int destination = backtrackTo(backTrackVia);
				cout<<"\nBacktracking from Node: "<<currentIndex<<" to Node: "<<destination;
				

				if( destination == 0 &&(toBeExplored[0].size() == 0)){
					allExplored = true;
					cout<<"\n Exploration Complete\n";
					cout<<"\nNumber of Nodes in Learned Graph: "<<learnedG.graphPoints.size()<<", Number of Nodes in Original Graph: "<<g.graphPoints.size();
					cout<<"\nNumber of Edges in Learned Graph: "<<learnedG.graphEdges.size()<<", Number of Edges in Original Graph: "<<g.graphEdges.size();
					
					if((learnedG.graphPoints.size() == g.graphPoints.size()) && (learnedG.graphEdges.size() == g.graphEdges.size()) ){
						cout<<"\n\nSuccess: The whole map has been Explored";
					}
					else{
						cout<<"\n\nSuccess: But the Orginal Graph couldn't be explored completely, since a portion is not reachable due to pits";
					}
					takeAction(ACTION_STOP);
					return;
				}

				//Need to handle the case where destination is same as the current node
				if(destination == currentIndex){
					//cout<<"\nHandling case of backtrack destination = currentNode";
					start = backTrackVia;
					goal = currentIndex;
					backtrackPlan = reconstructSolution(AStar(currentIndex,learnedG));
					BTIndex = 2;
					currentParent = PxVec3(NULL);
					takeAction(ACTION_STOP);
					return;
				}

				

				start = currentIndex;
				goal = destination;					
				backtrackPlan = reconstructSolution(AStar(currentIndex,learnedG));
				BTIndex = 0;
				nextNode = learnedG.graphPoints.at(backtrackPlan.at(1)->state);
				currentActionIndex = getActionIndex();
				//cout<<"\nActionIndex: "<<currentActionIndex;
				nextNode = learnedG.graphPoints.at(backtrackPlan.at(BTIndex)->state);
				cout<<"\nBackTrack Plan----";
				for(int i = 0; i < backtrackPlan.size(); i ++){
					cout<<"\nNode "<<i<<" in Plan==> "<<backtrackPlan.at(i)->state;
				}
			}
			
			
		}
		else if(backTrack == true){
			//cout<<"\nInside bacTrack ==true condition, bacTrack = "<<backTrack;
			if(BTIndex < backtrackPlan.size()){
				
				if(getCurrentNode() == nextNode){
					BTIndex++;
					if(BTIndex == backtrackPlan.size()){
						cout<<"\nBactracking done till destination";
						currentParent = PxVec3(NULL);
						takeAction(ACTION_STOP);
						return;
					}
					int next = backtrackPlan.at(BTIndex)->state;
					nextNode = learnedG.graphPoints.at(next);	
					//Need to update the currentActionIndex corresponding to nextNode
					currentActionIndex = getActionIndex();
					//cout<<"\nActionIndex: "<<currentActionIndex;
					B = PxVec2(nextNode.x, nextNode.z) - posX2D;
					B.normalize();
					//cout<<"\nB==> "<<B.x<<", "<<B.y<<"\n";
				}
								 
				B = PxVec2(nextNode.x, nextNode.z) - posX2D;
				B.normalize();
				//cout<<"\nB==> "<<B.x<<", "<<B.y<<"\n";
			}
			else{
				//Final node in backtrack plan
				
				if(atANode() && getCurrentNode() != currentParent){
					int currentIndex = backtrackPlan.at(BTIndex-1)->state;
					vector<PxVec2> actionsForCurrentNode = getActionVectorsForCurrentNode();
				
					set<int>::iterator nextActionNumber = (toBeExplored[currentIndex].begin());	
					currentActionIndex = *nextActionNumber;
					B = actionsForCurrentNode[currentActionIndex];
					//cout<<"\nB==> "<<B.x<<", "<<B.y<<"\n";
					toBeExplored[currentIndex].erase(nextActionNumber);
					currentParent = getCurrentNode();
					currentParentIndex = currentIndex;
					
				}
				else{
					cout<<"\nBacktracking Phase completed";
					backTrack = false;
				}
			}
		}

		
		if(atANode()){
			
			vector<float> smells = getCurrentSmells();
			if(smells[currentActionIndex]>0){
				//cout<<"\nWumpus at action "<<currentActionIndex<<" at node "<<currentActionIndex<<"\n";
				cout<<"\n";
				//Shoot the Wumpus		
				PxVec2 direction(B.x,B.y);
				direction.normalize();
				takeAction(ACTION_SHOOT, direction,smells[currentActionIndex]);
				//cout<<"\n Just after shooting action";
				//If the smell still exists ie the wumpus is trapped, then downt move, keep on shooting in future time steps
				smells = getCurrentSmells();
				//cout<<"\n After Shooting, checking if smell persists";
				if(smells[currentActionIndex]>0){
					//cout<<"\n Smell Persists!!!(Wumpus Still in front";
					takeAction(ACTION_STOP);
					this->programCounter++;
					return;
				}
				
				//cout<<"\n No smell persists for current action";				
				takeAction(ACTION_STOP);
				this->programCounter++;
				return;
			}				
		}

		

		A.normalize();
        //B.normalize();

		float thetaA = atan2(A.x, A.y);
        float thetaB = atan2(B.x, B.y);
		/*
		cout<<"\n thetaA = "<<thetaA;
		cout<<"\n thetaB = "<<thetaB;
		cout<<"\n underaction = "<<underAction;
        */
        float thetaAB = thetaB - thetaA;
        
        while (thetaAB <= - PxPi)
            thetaAB += 2 * PxPi;
        
        while (thetaAB > PxPi)
            thetaAB -= 2 * PxPi;

		PxReal angleError = thetaAB;
		//cout<<"\n fabs(angleError) = "<<fabs(angleError);
		if (fabs(angleError) > 0.01) {
            //turn right if it is less than zero, otherwise turn left
            if (angleError < 0) {
				//cout<<"\n Turning Right";
                takeAction(ACTION_RIGHT);
            } else {
				//cout<<"\n Turning Left";
                takeAction(ACTION_LEFT);
            }
        //If we are close enough to the goal
        } else if (atANode() && underAction==true) {
            //stop
			//cout<<"\nInside stop of if else";
            takeAction(ACTION_STOP);
			underAction = false;
            // Check if coming back after encountering a block
			
        //Otherwise, we need to keep going forward
        } else {
			//cout<<"\nGoing Forward";

			if(atANode()){
			
				vector<float> smells = getCurrentSmells();
				if(smells[currentActionIndex]>0){
					//cout<<"\nWumpus at action "<<currentActionIndex<<" at node "<<currentActionIndex<<"\n";
					cout<<"\n";
					//Shoot the Wumpus		
					PxVec2 direction(B.x,B.y);
					direction.normalize();
					takeAction(ACTION_SHOOT, direction,smells[currentActionIndex]);
					//cout<<"\n Just after shooting action";
					//If the smell still exists ie the wumpus is trapped, then downt move, keep on shooting in future time steps
					smells = getCurrentSmells();
					//cout<<"\n After Shooting, checking if smell persists";
					if(smells[currentActionIndex]>0){
						//cout<<"\n Smell Persists!!!";
						takeAction(ACTION_STOP);
						this->programCounter++;
						return;
					}
				
					//cout<<"\n No smell persists for current action";				
					takeAction(ACTION_STOP);
					this->programCounter++;
					return;
				}			
				else{
					takeAction(ACTION_FORWARD);
					if(!underAction){			
						underAction = true;
					}
				}
			}
            takeAction(ACTION_FORWARD);
			if(!underAction){			
				underAction = true;
			}
        }

        //cout<<"\nNeed to learn a graph with: "<<getNumGraphNodes()<<" nodes"<<endl;
        //cout<<"and: "<<getNumGraphEdges()<<" edges"<<endl;
        //cout<<endl<<endl;
        this->programCounter++;
    }
};

//Current position of Wheelbot
PxVec3 Project3_Control::currentWheelbotPosition;
//A plan of points to visit
vector<int> Project3_Control::plan;
//Radius around goal specifying when we have arrived at a goal
PxReal Project3_Control::currentGoalRadius = .40;
//Graph object on which to plan
Graph Project3_Control::g;
int Project3_Control::goal;
int Project3_Control::start;
Graph Project3_Control::learnedG;
vector<vector<int> > Project3_Control::actions;
vector<int> Project3_Control::graphPits;
float Project3_Control::minDistance = 5.0;
bool Project3_Control::shootingNoise = true;
vector<float> Project3_Control::graphWinds;
vector<float> Project3_Control::graphSmell;
vector<Wumpus> Project3_Control::graphWumpuses;
boost::mt19937 Project3_Control::rngShoot;
boost::normal_distribution<> Project3_Control::shootDistribution(0.0, 10.0);
boost::variate_generator< boost::mt19937, boost::normal_distribution<> > Project3_Control::shootResampler(rngShoot, shootDistribution);
vector<RenderLine> Project3_Control::renderLines;
#endif

