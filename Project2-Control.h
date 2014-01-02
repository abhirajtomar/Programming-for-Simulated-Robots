

#ifndef ReMod3D_Project2_Control_h
#define ReMod3D_Project2_Control_h


/*
 * Keyboard Map
 * See Camera.h for detailed camera control information
 * Buttons 1,2,3,4,5,6,w,a,s,d,(up arrow),(down arrow),(left arrow),(right arrow) control the camera
 * Holding down the left mouse button and moving the mouse manipulates the orientation of the camera
 * Button p sets program pause state (pause/unpause)
 * Button f switches between Wheelbot view and global view (NOTE: Color blob sensing will not work in global view, and the camera cannot be manually
 * moved in Wheelbot view)
 */

#include "ModuleProgram.h"
#include "WheelbotModule.h"
#include "SimulationUtility.h"
#include "ActionLog.h"
#include <map>
#include <queue>
#include <utility>
using namespace std;



///DO NOT MODIFY-------------------------------------------//
//Possible Wheelbot actions
enum WheelbotActions {
    ACTION_FORWARD = 0,
    ACTION_BACKWARD,
    ACTION_LEFT,
    ACTION_RIGHT,
    ACTION_STOP
};

//Class for representing a graph - in this case a map
class Graph {
public:
    //vector of map nodes (3d points) - indices of nodes in this vector are the node "names"
    vector<PxVec3> graphPoints;
    //vector of map edges (pairs of node indices) - nodes represented as their indices
    vector<pair<int, int> > graphEdges;
    Graph(vector<PxVec3> p, vector<pair<int, int> >  e) {
        this->graphPoints = p;
        this->graphEdges = e;
    }
    Graph(){}
};
///END DO NOT MODIFY-------------------------------------------//

class Project2_Control:rm3d::ai::ModuleProgram<rm3d::ai::WheelbotDock *> {
private:
    WheelbotSimBase* simulator;
    rm3d::ai::WheelbotModel *model;
    //Radius around goal, indicating when we have arrived at a map node
    static PxReal currentGoalRadius;
    //Magnitude of wheel velocities for movement
    PxReal velocityMagnitude;
    //start map node index
    static int start;
    //goal map node index
    static int goal;
    //Flag for determining whether or not a plan has been made already and is still being executed
    bool havePlanned;
    //Current step in plan being executed
    int currentPlanStep;
    //Map of the cities
    static Graph g;
    //Current position of Wheelbot - used for rendering
    static PxVec3 currentWheelbotPosition;
    //Position that is the "current goal" (i.e., the current node Wheelbot is going toward).
    //It will not correspond to the final goal until we make it to a node adjacent to the goal.
    static PxVec3 currentGoalPosition;
    //A plan of points (cities) to visit
    static vector<PxVec3> plan;
    //Should render path planned - useful for debugging
    static bool showPath;
	// An array of sets to store the neighbors of each graph node
	//set<int> *neighbors;
	bool retreat ;
	int goalReached ;
	map<int,set<int>> blockList;
	
public:
    //Helpful rendering functionality to show map and current path
    ///DO NOT MODIFY-------------------------------------------//
    static void render() {
        for (int i=0; i<g.graphPoints.size(); i++) {
            if ((currentWheelbotPosition - g.graphPoints[i]).magnitude() <= currentGoalRadius + 0.05) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,1,1));
            } else if (i == goal) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,1,0));
            } else if (i == start) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(1,1,0));
            } else {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,0,1));
            }
            
        }
        for (int j=0; j<g.graphEdges.size(); j++) {
            rm3d::Renderer::DrawLine(PxVec3(g.graphPoints[g.graphEdges[j].first].x,
                                            g.graphPoints[g.graphEdges[j].first].y + 0.05,
                                            g.graphPoints[g.graphEdges[j].first].z),
                                     PxVec3(g.graphPoints[g.graphEdges[j].second].x,
                                            g.graphPoints[g.graphEdges[j].second].y + 0.05,
                                            g.graphPoints[g.graphEdges[j].second].z), rm3d::Color(0,1,1));
        }
        
        
        if (showPath) {
            if (plan.size() > 0) {
                for (int i=0; i<plan.size() -1; i++) {
                    rm3d::Renderer::DrawLine(PxVec3(plan[i].x, plan[i].y + .1, plan[i].z),
                                             PxVec3(plan[i + 1].x,
                                             plan[i + 1].y + .1, plan[i + 1].z), rm3d::Color(0,1,0));
                }
            }
        }
        
    }
    ///END DO NOT MODIFY-------------------------------------------//
    
    Project2_Control(WheelbotSimBase* simulator, Graph graph, int startNode, int goalNode) {
        this->programCounter = 0;
        this->actionQueue = new rm3d::ai::ActionLog*[4];
        this->simulator = simulator;
        this->actionQueue[0] = NULL;
        this->actionQueue[1] = NULL;
        this->actionQueue[2] = NULL;
        this->actionQueue[3] = NULL;
        this->havePlanned = false;
        this->velocityMagnitude = 15.0;
        start = startNode;
        goal = goalNode;
        g = graph;
		retreat = false;
		goalReached = 0;
		int n = g.graphPoints.size();
		//neighbors = (set<int>*) malloc(n*n);
		//neighbors = new vector<set<int>>(n,set<int>(n));
		/*
		int n = g.graphPoints.size();
		vector<set<int>> tempNbrs(n);
		cout << "Just befor nbrs";
		for (int i=0; i<g.graphPoints.size(); i++){
			set<int> temp;
			for (int j=0; j<g.graphEdges.size(); j++){
				if(g.graphEdges[j].first == i){
					temp.insert(g.graphEdges[j].second);
				}
			}
			tempNbrs[i] = temp;
			temp.clear();
		}
		neighbors = &tempNbrs;
		*/
		
    }
    
    ///DO NOT MODIFY-------------------------------------------//
    ~Project2_Control() {
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
    
    PxTransform getCurrentWheelbotPose() {
        return boost::any_cast<PxTransform>(this->model->getCurrentState()->sensorValues[rm3d::ai::WheelbotModule::posOrientInt]);
    }
    
    PxVec3 getGoalPosition() {
        return currentGoalPosition;
    }
    
    PxRaycastHit getRangeResult() {
        PxRaycastHit hit = boost::any_cast<PxRaycastHit>(this->model->getCurrentState()->sensorValues[rm3d::ai::WheelbotModule::xAxisRaycastInt]);
        return hit;
    }
    
    void takeAction(WheelbotActions action) {
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
        }
    }
    ///END DO NOT MODIFY-------------------------------------------//
    
	float pointDist(int a, int b){
		PxVec3  A = g.graphPoints[a];
		PxVec3  B = g.graphPoints[b];

		float distance = pow((A.x - B.x),2) + pow((A.y - B.y),2) + pow((A.z - B.z),2);

		distance = sqrt(distance);

		return distance;

	}

	int setMin(set<int> S, float Vals[]){
		int min = *S.begin();
		for(set<int>::iterator it=S.begin(); it!=S.end(); ++it){
			if(Vals[*it] < Vals[min] ){
				min = *it;
			}
		}
		return min;
	}

	
	void createPlan(map<int,int> pred, int current){
		if(pred.find(current)!= pred.end()){
			createPlan(pred,pred[current]);
			plan.push_back(g.graphPoints[current]);
			//cout<<"\nPlan Point: "<< current;
		}
		else{
			plan.push_back(g.graphPoints[current]);
			//cout<<"\nPlan Point: "<< current;
		}

	}

	int aStarSearch(){
		set<int> evaluatedSet;		//The set of nodes already evaluated
		set<int> unfinishedSet;			//The set of tentative nodes to be evaluated, initially containing the start node
		map<int,int> predecessor;	//The map of navigated nodes.
		//cout<<"Inside aStarSearch\n";
		float *gVal;
		float *fVal;
		float *hVal;
		gVal = (float*)malloc(g.graphPoints.size()*sizeof(float));
		fVal = (float*)malloc(g.graphPoints.size()*sizeof(float));
		hVal = (float*)malloc(g.graphPoints.size()*sizeof(float));
		
		gVal[start] = 0.0;
		fVal[start] = gVal[start] + pointDist(start,goal);
		unfinishedSet.insert(start);
		while(!unfinishedSet.empty()){
			
			int current = setMin(unfinishedSet, fVal);
			//cout<<"While Iteration , current = "<< current<<"\n";
			if (current == goal){
				//goal found
				createPlan(predecessor,current);
				return 1;
				//return reconstruct_path(predecessor,goal);
			}

			unfinishedSet.erase(current);
			evaluatedSet.insert(current);
			//set<int> currentNbrSet = neighbors[current];
			// Creating a set of neighbors of current
			set<int> neighbors;
			for (int j=0; j<g.graphEdges.size(); j++){
				if(g.graphEdges[j].first == current){
					//cout<<"Nbr of "<<current<<": "<<g.graphEdges[j].second;
					neighbors.insert(g.graphEdges[j].second);
				}
			}

			//removing nodes following a blocked path
			if(blockList.find(current)!= blockList.end()){
				for(set<int>::iterator block=blockList[current].begin(); block!= blockList[current].end(); ++block){
					neighbors.erase(*block);
				}
			}

			for(set<int>::iterator nbr=neighbors.begin(); nbr!=neighbors.end(); ++nbr){

				float tempgVal = gVal[current] + pointDist(current,*nbr);
				float tempfVal = tempgVal + pointDist(*nbr,goal);
				if(evaluatedSet.count(*nbr)!=0 && tempfVal >= fVal[*nbr]){
					continue;
				}

				if(unfinishedSet.count(*nbr)==0 || tempfVal < fVal[*nbr] ){
					predecessor[*nbr] = current;
					gVal[*nbr] = tempgVal;
					fVal[*nbr] = tempfVal;
					if(unfinishedSet.count(*nbr)==0){
						unfinishedSet.insert(*nbr);
					}
				}
			}			
		}
		return 0;
		//no goal found
	}


    void step(void *model) {
        ///DO NOT MODIFY------------------///
        simulator->setCustomRenderFunction(render);
        this->model = (rm3d::ai::WheelbotModel *)model;
        currentWheelbotPosition = getCurrentWheelbotPose().p;
        if (this->actionQueue[0] != NULL) delete this->actionQueue[0];
        if (this->actionQueue[1] != NULL) delete this->actionQueue[1];
        if (this->actionQueue[2] != NULL) delete this->actionQueue[2];
        if (this->actionQueue[3] != NULL) delete this->actionQueue[3];
        this->numActions = 4;
        ///END DO NOT MODIFY------------------///
        
        
		// Check if goal has been reached
		if(goalReached == 1){
			cout<<"\nSuccess: Goal Reached";
			takeAction(ACTION_STOP);
			return;
		}	
		// Check if it is impossible to reach goal
		else if(goalReached == -1){
			cout << "\nFailure: No possible path to goal.";
			takeAction(ACTION_STOP);
			return;
		}

		// Check if it is needed to conduct a new A Star search for goal 
        if (!havePlanned) {
			cout<<"Start: "<< start <<", Goal: "<< goal <<"\n";
            //Make sure currentPlanStep is reset
            currentPlanStep = 0;
            //Set the flag to true, so we know we have planned and are going to start executing
            havePlanned  = true;
			// Clearing the previous Plan
			plan.clear();

			//Calling A Star search
			//cout<< "Calling A* search\n";
			int searchStatus = aStarSearch();

			// Checking if the search returned a valid plan
			if(searchStatus == 1){
				currentPlanStep++;
				currentGoalPosition = plan[currentPlanStep];	
			}
			else{
				takeAction(ACTION_STOP);
				cout << "Failure: No possible path to goal.";
				goalReached = -1;
				return;
			}

        }
        
        
        //Check our raycast sensor
        PxRaycastHit hit = getRangeResult();
        //If we see something less than 0.28 meters in front of us, we stop and return
        if (hit.distance < 0.28 && hit.distance > 0.01) {
            takeAction(ACTION_STOP);
            //NOTE: the return is simply so we don't run into the obstacle. You should remove it when
            //you code your solution because you will have to return to the most recent city you were at.
            			
			// Setting cuurent goal to last valid node in the plan 
			currentGoalPosition = plan[currentPlanStep-1];			
			retreat = true;
        }
        
        //The following code is almost exactly taken from the solution to Project 1
        
        //2D position vector of Wheelbot (in global frame)
        PxVec2 pos2D = PxVec2(getCurrentWheelbotPose().p.x, getCurrentWheelbotPose().p.z);
        //3D position vector of point 0.3 units along Wheelbot's x-axis (in global frame)
        PxVec3 posAlongX = (getCurrentWheelbotPose()*PxTransform(PxVec3(0.3,0,0))).p;
        //2D position vector of point 0.3 units along Wheelbot's x-axis (in global frame)
        PxVec2 posAlongX2D = PxVec2(posAlongX.x, posAlongX.z);
        //2D position vector of goal (in global frame)
        PxVec2 goal2D = PxVec2(getGoalPosition().x, getGoalPosition().z);
        //2D Vector of length 0.3 along Wheelbot's x-axis starting at Wheelbot's position
        PxVec2 A = posAlongX2D - pos2D;
        //2D Vector from Wheelbot's position to the goal
        PxVec2 B = goal2D - pos2D;
        
        //The following code extracts the angle between the above vectors using atan2
        A.normalize();
        B.normalize();
        
        float thetaA = atan2(A.x, A.y);
        float thetaB = atan2(B.x, B.y);
        
        float thetaAB = thetaB - thetaA;
        
        while (thetaAB <= - PxPi)
            thetaAB += 2 * PxPi;
        
        while (thetaAB > PxPi)
            thetaAB -= 2 * PxPi;
        
        //Here is the result: the angular error between Wheelbot's heading and the goal direction
        PxReal angleError = thetaAB;
        
        //If the angular error is too large
        if (fabs(angleError) > 0.05) {
            //turn right if it is less than zero, otherwise turn left
            if (angleError < 0) {
                takeAction(ACTION_RIGHT);
            } else {
                takeAction(ACTION_LEFT);
            }
        //If we are close enough to the goal
        } else if ((goal2D - pos2D).magnitude() <= currentGoalRadius) {
            //stop
            takeAction(ACTION_STOP);
            // Check if coming back after encountering a block
			if(retreat){
				start = find(g.graphPoints.begin(), g.graphPoints.end(), currentGoalPosition) - g.graphPoints.begin();
				int block = find(g.graphPoints.begin(), g.graphPoints.end(), plan[currentPlanStep]) - g.graphPoints.begin();
				
				// Marking the blocked edges by adding start and block to each other's blocklists
				if(blockList.find(start)!= blockList.end()){
					blockList[start].insert(block);				
				}
				else{
					set<int> temp;
					temp.insert(block);
					blockList[start] = temp;
				}
				if(blockList.find(block)!= blockList.end()){
					blockList[block].insert(start);				
				}
				else{
					set<int> temp;
					temp.insert(start);
					blockList[block] = temp;
				}

				havePlanned =  false;
				retreat = false;
			}
			else{
				//Increment the plan step
				currentPlanStep++;
				//If the plan step is less than the plan size
				if (currentPlanStep < plan.size()) {
					//set the currentGoalPosition to the next node (city) on the path
					currentGoalPosition = plan[currentPlanStep];					
				}
				else{
					if(plan[currentPlanStep-1] == g.graphPoints[goal]){
						cout<<"\nSuccess: Goal Reached";
						takeAction(ACTION_STOP);
						goalReached = true;
						return;
					}

				}
			}
        //Otherwise, we need to keep going forward
        } else {
            takeAction(ACTION_FORWARD);
        }
        this->programCounter++;
    }
};

//Static variable definitions

PxVec3 Project2_Control::currentWheelbotPosition;
PxVec3 Project2_Control::currentGoalPosition;
vector<PxVec3> Project2_Control::plan;
//DO NOT MODIFY ----------------------------------//
PxReal Project2_Control::currentGoalRadius = 0.1;
//END DO NOT MODIFY ------------------------------//
bool Project2_Control::showPath = true;
Graph Project2_Control::g;
int Project2_Control::goal;

int Project2_Control::start;
#endif
