package exploration;

import agents.RealAgent;
import communication.PropModel1;
import config.SimConstants;
import environment.ContourTracer;
import environment.Frontier;

import java.awt.*;
import java.util.*;
import java.util.List;

public class LFComms {

    private static final LFState START_STATE = LFState.GoingHome;

    private enum LFState {
        GoingHome, // Obvious
        GoingToFrontier, // Obvious
        WaitForSuccessors, // Used when chain is reforming
        MovingToPredecessor, // Used when chain is reforming
        WaitAtBS, // Waiting at the base station for the rest of the team (POC)
        WaitInChain, // Waiting in the chain as they cannot move forward
        MovingToPoint, // Moving from last chain point to new chain point

    }

    private boolean begun = false;

    private static LFComms singleton;
    private RealAgent baseStation;
    private RealAgent leader = null;



    // Links each robot to its index in the chain
    private final Map<RealAgent, Integer> agentToIndex;
    private final Map<Integer, RealAgent> indexToAgent;




    // Each state s_i represents the state of robot i
    private final List<LFState> agentStates;



    private PriorityQueue<Frontier> frontiers;
    private Frontier f;

    private LFComms(){
        agentToIndex = new HashMap<>();
        indexToAgent = new HashMap<>();
        frontiers = new PriorityQueue<>();
        agentStates = new ArrayList<>();
    }

    //                            Initialisation Methods

    // Assign the robot a placeholder index, as we don't know if all robots have been registered yet
    private void addRobot(RealAgent a){
        agentToIndex.put(a,0);
    }
    public synchronized static LFComms register(RealAgent a){
        a.announce("Registered");
        if(singleton == null){
            singleton = new LFComms();
            singleton.baseStation = a.baseStation;
        }
        singleton.addRobot(a);
        return singleton;
    }

    // Once we know all the robots that are participating, we assign them all indices
    // These indices are universal for all the lists the robot will access in this class
    // This will also find which agent should be the leader, which is important for finding frontiers
    private void assignIndices(){
        List<RealAgent> agents = new ArrayList<>(agentToIndex.keySet());
        agents.sort(Comparator.comparingInt(RealAgent::getRobotNumber));
        agentToIndex.replace(baseStation,-1);
        indexToAgent.put(-1, baseStation);
        leader = agents.get(agents.size()-1);
        for(int i = 0; i < agents.size(); i++){
            agentToIndex.replace(agents.get(i),i);
            indexToAgent.put(i, agents.get(i));
            agentStates.add(START_STATE);
        }

    }




    //                  Next Step Method

    // Gives agents their paths and checks if they need new ones
    public synchronized Point getNextPosition(RealAgent a){
        if(!begun){
            a.announce("Initiated Algorithm");
            assignIndices();
            begun = true;
        }
        Point next;

        switch (agentStates.get(agentToIndex.get(a))){
            case GoingHome:
                a.announce("Going Home");
                // Ensures that the agents all go to the base station immediately
                // Only called in the first timestep
                if(a.getPath() == null){
                    a.setPath(a.calculatePath(baseStation.getLocation(), false));
                }

                if(a.getLocation().equals(baseStation.getLocation())){
                    setState(a, LFState.WaitAtBS);
                    return a.stay();
                }

                next = a.getNextPathPoint();

                if(a.getPath().getIndex() == 0){
                    return baseStation.getLocation();
                }
                if(isPositionOkay(a, next)){
                    return next;
                } else{
                    a.getPath().resetStep();
                    return a.stay();
                }

            case WaitAtBS:
                a.announce("Waiting at Base");
                if(allInState(LFState.WaitAtBS)){
                    agentToIndex.keySet().forEach(agent -> setState(agent, LFState.GoingToFrontier));
                    newPaths();
                }
                return a.stay();

            case GoingToFrontier:
                a.announce("Going to Frontier");
                next = a.getNextPathPoint();

                if(isPositionOkay(a, next) && !a.getPath().isFinished()){
                    // If a has moved forward, its successors may be able to move now
                    triggerSuccessors(a);
                    return next;
                } else{
                    // assume that this means the robot is as far as it can go
                    a.getPath().resetStep();
                    setState(a, LFState.WaitInChain);
                    return a.stay();
                }
            case WaitInChain:
                a.announce("Waiting in Chain");
                if(allInState(LFState.WaitInChain)){
                    goHome();
                    agentToIndex.keySet().forEach(agent -> setState(agent, LFState.GoingHome));
                    return getNextPosition(a);
                } else{
                    return a.stay();
                }
            case WaitForSuccessors:
                a.announce("How?");
                break;
            case MovingToPredecessor:
                a.announce("How?");
                break;
            case MovingToPoint:
                break;
        }

        return a.stay();

    }




    public LFState getState(RealAgent a){
        return agentStates.get(agentToIndex.get(a));
    }

    public synchronized void setState(RealAgent a, LFState newState){
        agentStates.set(agentToIndex.get(a), newState);
    }


    private synchronized boolean allInState(LFState target){
        for(LFState state : agentStates){
            if(state != target){
                return false;
            }
        }
        return true;
    }


    private synchronized void triggerSuccessors(RealAgent a){
        for(int i = agentToIndex.get(a); i < agentStates.size(); i++){
            agentStates.set(i, LFState.GoingToFrontier);
        }

    }

    public synchronized boolean isPositionOkay(RealAgent a, Point p){
        RealAgent prevChain = indexToAgent.get(agentToIndex.get(a)-1);
        if(a == leader){
            // the leader has no successor
            return PropModel1.isConnected(baseStation.getOccupancyGrid(), prevChain.getCommRange(), p, prevChain.getLocation());
        } else{
            // check if the point we can go to is viable for communication from both the predecessor and successor
            RealAgent nextChain = indexToAgent.get(agentToIndex.get(a) + 1);
            return PropModel1.isConnected(baseStation.getOccupancyGrid(), prevChain.getCommRange(), p, prevChain.getLocation())
                    &&
                    PropModel1.isConnected(baseStation.getOccupancyGrid(), a.getCommRange(), p, nextChain.getLocation());
        }


    }

    // Boilerplate code to find the frontiers
    private void calculateFrontiers(){
        // Set the old list of frontiers to dirty, and clear the queue
        // this "dirtying" is for rendering, and so isn't strictly needed
        frontiers.forEach((f) -> leader.addDirtyCells(f.getPolygonOutline()));
        frontiers.clear();

        // Find all the boundaries between the "known" tiles and "unknown tiles"
        LinkedList<LinkedList<Point>> contours = ContourTracer.findAllContours(leader.getOccupancyGrid());
        // convert the contours to frontiers, and filter out all those that are invalid
        for (LinkedList<Point> contour : contours) {
            Frontier frontier = new Frontier(leader.getX(), leader.getY(), contour);
            if(frontier.getArea() >= SimConstants.MIN_FRONTIER_SIZE && !leader.isBadFrontier(frontier)){
                frontiers.add(frontier);
            }
        }

    }

    private void newPaths(){
        calculateFrontiers();

        // exploration is over
        if(frontiers.isEmpty()){
            goHome();
            return;
        }

        // frontiers is a priority queue, so we poll the head and plan a path from the base station
        f = frontiers.poll();

        agentToIndex.keySet().forEach(a -> {
            if(a.getPath() != null){
                a.getPath().setInvalid();
            }
            // this should give each agent the same path
            a.setPath(baseStation.calculatePath(f.getCentre(), false));
        });
    }

    private void goHome(){
        agentToIndex.keySet().forEach(a -> {
            a.announce("Going home");
            if(a.getPath() != null){
                a.getPath().AlecReverse();
            } else{
                a.setPath(a.calculatePath(baseStation.getLocation(), false));
            }

        });
    }


    // Path checking algo:
    // Intuition here is that we are essentially simulating the robots walking along the path
    /**
     * Point base = baseStation.getLocation();
     * int range = baseStation.getRange();
     * int agent = 0;
     * Point prev;
     * for(Point p : path.getPoints():
     *     if(isPositionOkay(indexToAgent(0,p)):
     *         prev = p;
     *     else:
     *         agent.setPath(prev, false);
     *         agent++;
     *         base = prev;
     * After this each agent plans a path to this point
     * We then need to compare paths iteratively starting from the base of the chain
     * to make sure that they can be followed without breaking communications
     * A good start to doing this is to iterate along each path at the same time and check if both points are in range
     * As soon as one is not, we set all
     */
}
