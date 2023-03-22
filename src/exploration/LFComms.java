package exploration;

import agents.RealAgent;
import communication.PropModel1;
import config.SimConstants;
import environment.ContourTracer;
import environment.Frontier;
import path.Path;

import java.awt.*;
import java.util.*;
import java.util.List;


// TODO: Dynamic chain
// Instead of checking if the position is in range of the predecessor and successor, check that it maintains connection with base station
// how?
// Check that another agent exists that has a BS connection, if there is we can move (assuming we're not as far in the path)
public class LFComms {

    private static final State START_STATE = State.Initial;
    private static final boolean EXACT_PATH = true;

    private enum State {
        Initial, // first state to make them stay
        GoingHome, // Obvious
        GoingToFrontier, // Obvious
        WaitForSuccessors, // Used when chain is reforming
        MovingToPredecessor, // Used when chain is reforming
        WaitAtBS, // Waiting at the base station for the rest of the team
        WaitInChain, // Waiting in the chain as they cannot move forward
        MovingToPoint, // Moving between chains

    }

    private boolean begun = false;
    private static LFComms singleton;
    private RealAgent baseStation;
    private RealAgent leader = null;


    // Links each robot to its index in the chain
    private final Map<RealAgent, Integer> agentToIndex;
    private final Map<Integer, RealAgent> indexToAgent;


    // Each state s_i represents the state of robot i
    private final List<State> agentStates;
    private List<Point> agentPoints;
    private Point predPoint;
    private int predId;

    private List<Path> agentBSPaths;


    private final PriorityQueue<Frontier> frontiers;

    private final List<Frontier> failPlanFrontiers;
    private Frontier f;

    private LFComms(){
        agentToIndex = new HashMap<>();
        indexToAgent = new HashMap<>();
        frontiers = new PriorityQueue<>();
        agentStates = new ArrayList<>();
        agentPoints = new ArrayList<>();
        agentBSPaths = new ArrayList<>();
        failPlanFrontiers = new ArrayList<>();
    }

    //                            Initialisation Methods

    /**
     *  Assign the robot a placeholder index, as we don't know if all robots have been registered yet
     */
    private void addRobot(RealAgent a){
        agentToIndex.put(a,0);
    }

    /**
     * Registers a robot to participate in the Leader-Follower Algorithm
     * @param a Agent
     * @return LFComms singleton
     */
    public synchronized static LFComms register(RealAgent a){
        a.announce("Registered");
        if(singleton == null){
            singleton = new LFComms();
            singleton.baseStation = a.baseStation;
        }
        singleton.addRobot(a);
        return singleton;
    }

    /**
     * Once we know all the robots that are participating, we assign them all indices.
     * These indices are universal for all the lists the robot will access in this class.
     * This will also find which agent should be the leader, which is important for finding frontiers.
      */
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

    /**
     * Leader-Follower Algorithm entry method
     * @param a Agent
     * @return Next Point
     */
    public synchronized Point getNextPosition(RealAgent a){
        if(!begun){
            a.announce("Initiated Algorithm");
            assignIndices();
            begun = true;
        }
        a.announce("Current location ".concat(a.getLocation().toString()));

        switch (getState(a)){
            case Initial:
                return initial(a);
            case GoingHome:
                return goingHome(a);
            case WaitAtBS:
                return waitAtBase(a);
            case GoingToFrontier:
                return goingToFrontier(a);
            case WaitInChain:
                return waitingInChain(a);
            case WaitForSuccessors:
                return waitingForSuccessors(a);
            case MovingToPredecessor:
                return movingToPredecessor(a);
            case MovingToPoint:
                return movingToPoint(a);
        }

        return a.stay();
    }

    //   Waiting methods
    private synchronized Point initial(RealAgent a){
        setState(a, State.GoingHome); // Exists to allow sensor data to come in before we plan
        return a.stay();
    }

    private synchronized Point waitAtBase(RealAgent a){
        a.announce("Waiting at Base");

        if(allInState(State.WaitAtBS)){
            a.announce("Moving");
            clearEnvErrors();
            newPaths();
            return getNextPosition(a);
        } else{
            return a.stay();
        }
    }
    private synchronized Point waitingInChain(RealAgent a){
        a.announce("Waiting in Chain");
        if(allInState(State.WaitInChain)){
            leader.addBadFrontier(f);
            newPaths();
            return getNextPosition(a);
        } else{
            return a.stay();
        }
    }
    private synchronized Point waitingForSuccessors(RealAgent a){
        a.announce("Waiting for Successors");
        a.setEnvError(false);
        if(allInState(State.WaitForSuccessors)){
            Collections.fill(agentStates, State.MovingToPoint);
            return movingToPoint(a);
        } else{
            return a.stay();
        }
    }


    //     Movement Methods
    private synchronized Point goingHome(RealAgent a){
        a.announce("Going Home");

        if(a.getLocation().equals(baseStation.getLocation())){
            a.announce("Reached Base");
            a.setPathInvalid();
            setState(a, State.WaitAtBS);
            return a.stay();
        }

        if(a.getLocation().distance(getPredecessor(a).getLocation()) < 10){
            setState(getPredecessor(a), State.GoingHome);
        }

        // Should only be called in the first timestep
        if(a.getPath() == null || !a.getPath().isValid() || a.getPath().isFinished() || a.getEnvError() || a.getPath().isAlecDone()){
            a.announce("Path null");
            a.setEnvError(false);
            a.setPath(a.calculateAStarPath(getPredecessor(a).getLocation(), EXACT_PATH));
        }

        Point next = a.getNextPathPoint();
        if(isPositionOkay(a, next)){
            a.announce("Position Okay");
            return next;
        } else{
            a.announce("Position Not Okay");
            a.getPath().resetStep();
            return a.stay();
        }
    }

    private synchronized Point goingToFrontier(RealAgent a){
        a.announce("Going to Frontier");
        Point next = a.getNextPathPoint();

        if(isPositionOkay(a, next) && !a.getPath().isFinished()){
            // If a has moved forward, its successors may be able to move now
            setSuccessors(a, State.GoingToFrontier);
            return next;
        } else{
            // assume that this means the robot is as far as it can go
            a.getPath().resetStep();
            setState(a, State.WaitInChain);
            return waitingInChain(a);
        }
    }

    private synchronized Point movingToPredecessor(RealAgent a){
        a.announce("Moving to Predecessors");

        if(a.getEnvError()){
            a.setEnvError(false);
            a.setPathInvalid();
            a.setPath(a.calculateAStarPath(predPoint, EXACT_PATH));
        }

        if(a.getLocation().equals(predPoint)){
            setState(a, State.WaitForSuccessors);
            a.setPath(a.calculateAStarPath(agentPoints.get(predId), EXACT_PATH)); // we can assume this will be the same as the path the original agent planned
            return waitingForSuccessors(a);
        } else{
            Point next = a.getNextPathPoint();
            a.announce("Target = ".concat(predPoint.toString()).concat(", Next Point = ".concat(next.toString())));
            return next;
        }
    }

    private synchronized Point movingToPoint(RealAgent a){
        a.announce("Moving to Point");
        assert a.getPath() != null;

        if(a.getLocation().equals(getPoint(a))){
            setState(a, State.WaitInChain);
            return waitingInChain(a);
        } else{
            if(a.getEnvError()){
                a.announce("Env Error, oops!");
                leader.addBadFrontier(f);
                clearEnvErrors();
                goHome();
            }
            if(!a.getPath().isValid()){
                a.announce("Path invalid");
                a.setPath(a.calculateAStarPath(getPoint(a),EXACT_PATH));
            }

            if(a.getPath().isFinished()){
                // transition from moving between chains to moving up the chain
                a.announce("Switching to new path");
                a.setPathInvalid();
                a.setPath(a.calculateAStarPath(getPoint(a), EXACT_PATH));
            }

            return a.getNextPathPoint();
        }

    }


    //               Path Switching

    /**
     * Boilerplate code to find the frontiers
     */
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
            if(frontier.getArea() >= SimConstants.MIN_FRONTIER_SIZE && !leader.isBadFrontier(frontier) && !failPlanFrontiers.contains(frontier) && !explored.contains(frontier)){
                frontiers.add(frontier);
            }
        }

    }


    // Step 1) Get new frontier
    // Step 2) Get path to new frontier
    // Step 3) Calculate agent points on this path
    // Step 4) Get path from each agent to this new point
    // Step 5) Check each path can be followed while maintaining connection to previous agent
    // Step 6) If a path is not, the agent and all agents past it must return to the previous one

    List<Frontier> explored = new ArrayList<>();
    private synchronized void newPaths(){
        baseStation.announce("Triggered newPaths()");
        Path p;
        while(true) {
            do {
                // Step 1)
                calculateFrontiers();

                // In this algorithm we actually want to poll the frontier closest to the leader, since each exploration requires
                // a lot of coordination. By choosing the frontier closest to the leader we will generally not need to move the chain as much

                Optional<Frontier> ft = frontiers.stream()
                        .filter(f -> !leader.getOccupancyGrid().obstacleWithinDistance(f.getCentre().x, f.getCentre().y, 10))
                        .min((f1, f2) -> (int) (f1.getCentre().distance(leader.getLocation()) - f2.getCentre().distance(leader.getLocation())));

                //
                if(ft.isEmpty()){
                    baseStation.announce("Empty Frontiers List");
                    baseStation.setMissionComplete(true);
                    goHome();
                    return;
                } else{
                    f = ft.get();
                }
                baseStation.announce(f.toString());
                failPlanFrontiers.add(f);
                // Step 2)
                p = baseStation.calculateAStarPath(f.getCentre(), EXACT_PATH);
            } while (!p.isValid());
            failPlanFrontiers.clear();

            baseStation.setPath(p);
            List<Path> newBSPaths = new ArrayList<>();
            for (int i = 0; i < agentToIndex.size(); i++) {
                newBSPaths.add(baseStation.calculateAStarPath(f.getCentre(), EXACT_PATH));
            }

            explored.add(f);

            // Step 3)
            agentPoints = findAgentPositions(p);

            Point lP = agentPoints.get(agentToIndex.get(leader));

            if (lP.distance(f.getCentre()) > leader.getSenseRange() ||
                    leader.getOccupancyGrid().numObstaclesOnLine(lP.x, lP.y, f.getCentre().x, f.getCentre().y) > 0) {
                leader.addBadFrontier(f);
                p = null;
                f = null;
                continue;
            }
            baseStation.announce(agentPoints.toString());
            List<Path> paths = new ArrayList<>();

            // Step 4)
            for (int i = 0; i < agentToIndex.keySet().size(); i++) {
                Path pt = baseStation.calculatePath(indexToAgent.get(i).getLocation(), agentPoints.get(i), true, EXACT_PATH);
                if(pt.isValid()){
                    paths.add(pt);
                } else{
                    goHome();
                    explored.remove(f);
                    return;
                }

            }
            baseStation.announce(paths.toString());

            // Step 5)
            // Base case: First path against base station
            if (!checkPathAgainstPoint(indexToAgent.get(0).getCommRange(), baseStation.getLocation(), paths.get(0))) {
                baseStation.announce("Agent ".concat(String.valueOf(0).concat(") Path is not okay")));
                goHome();
                return;
            } else {
                baseStation.announce("Agent ".concat(String.valueOf(0).concat(") Path is okay")));
                indexToAgent.get(0).setPathInvalid();
                indexToAgent.get(0).setPath(paths.get(0));
                setState(indexToAgent.get(0), State.WaitForSuccessors);
            }

            // Recursive case: Each path against the last one
            for (int i = 1; i < paths.size(); i++) {
                if (checkPaths(baseStation.getCommRange(), paths.get(i - 1), paths.get(i))) {
                    baseStation.announce("Agent ".concat(String.valueOf(i).concat(") Path is okay")));
                    indexToAgent.get(i).setPathInvalid();
                    indexToAgent.get(i).setPath(paths.get(i));
                    setState(indexToAgent.get(i), State.WaitForSuccessors);
                } else {
                    // Step 6
                    predPoint = indexToAgent.get(i - 1).getLocation();
                    predId = i - 1;
                    setSuccessors(indexToAgent.get(i), State.MovingToPredecessor);
                    for (int j = i; j < agentToIndex.size(); j++) {
                        baseStation.announce("Agent ".concat(String.valueOf(j).concat(") Path is not okay")));
                        RealAgent agent = indexToAgent.get(j);
                        setAgentBSPath(agent);
                    }
                    break;
                }
            }

            agentBSPaths = newBSPaths;
            return;
        }
    }

    private void goHome(){
        failPlanFrontiers.clear();
        predPoint = baseStation.getLocation();
        baseStation.announce("Triggered Full Recall");
        agentToIndex.keySet().forEach(a -> {
            if(a.getLocation().equals(baseStation.getLocation())){
                setState(a, State.WaitAtBS);
            } else{
                setState(a, State.WaitForSuccessors);
                setAgentBSPath(a);
            }
        });
        setState(leader, State.GoingHome);
    }


    /**
     * Given a path from the base station, find the points that each robot will stop on.
     * @param p Path to check
     * @return List of agent positions on p
     */
    private synchronized List<Point> findAgentPositions(Path p){
        assert p.getStartPoint() == baseStation.getLocation();

        List<Point> output = new ArrayList<>(agentToIndex.size());
        Point prev = p.getStartPoint(); // should always be the base station
        baseStation.announce("Finding agent positions");
        baseStation.announce("Start = ".concat(prev.toString()));
        baseStation.announce("Goal = ".concat(p.getGoalPoint().toString()));
        int agentI = 0;
        int range = indexToAgent.get(agentI).getCommRange();

        for(int index = 0; index < p.getPoints().size()-1; index++) {
            // if the next point is out of range, this point must be where the agent stops
            if (!isPositionOkay(range, prev, p.getPoints().get(index + 1))) {
                prev = p.getPoints().get(index);
                output.add(prev);
                agentI++;
                if (agentToIndex.get(leader) < agentI) {
                    break;
                }
                range = indexToAgent.get(agentI).getCommRange();
            }
        }
        // fill the unfilled values with the goal point
        while(output.size() <= agentToIndex.get(leader)){
            output.add(p.getGoalPoint());
        }

        return output;
    }

    /**
     * Checks if b can be traversed while maintaining connection to an agent traversing a
     * We assume a can be traversed while maintaining connection to its previous path
     * (Exact order of a and b doesn't actually matter)
    **/
    private synchronized boolean checkPaths(int range, Path a, Path b){
        baseStation.announce("");
        baseStation.announce("Checking ".concat(a.toString()).concat(" against ").concat(b.toString()));
        baseStation.announce(a.toString().concat(" has ".concat(String.valueOf(a.getPoints().size()).concat(" points"))));
        baseStation.announce(b.toString().concat(" has ".concat(String.valueOf(b.getPoints().size()).concat(" points"))));

        int aI = 0;
        int bI = 0;
        while(aI < a.getPoints().size()-1 || bI < b.getPoints().size()-1){

            if(!isPositionOkay(range, a.getPoints().get(aI), b.getPoints().get(bI))){
                return false;
            }

            if(aI < a.getPoints().size()-1){
                aI++;
            }
            if(bI < b.getPoints().size()-1){
                bI++;
            }
        }
        return true;
    }

    /**
     * Checks if b can be traversed while maintaining connection to point a
     */
    private synchronized boolean checkPathAgainstPoint(int range, Point a, Path b){
        baseStation.announce("checking ".concat(b.toString()).concat(" against ").concat(a.toString()));
        for(Point p : b.getPoints()){
            if(!isPositionOkay(range, a, p)){
                return false;
            }
        }
        return true;
    }

    //                Auxilliary Functions
    private State getState(RealAgent a){
        return agentStates.get(agentToIndex.get(a));
    }
    private synchronized void setState(RealAgent a, State newState){
        if (!a.equals(baseStation)) {
            agentStates.set(agentToIndex.get(a), newState);
        }
    }
    private Point getPoint(RealAgent a){
        return agentPoints.get(agentToIndex.get(a));
    }

    private void setAgentBSPath(RealAgent a){
        Path p = agentBSPaths.get(agentToIndex.get(a));
        a.setPath(p);
        p.budge(a.getLocation());
        p.AlecReverse();
        p.setAlecFinish(predPoint);
    }
    private RealAgent getPredecessor(RealAgent a){
        return indexToAgent.get(agentToIndex.get(a)-1);
    }

    /**
     * Check if all agents are in a state
     */
    private synchronized boolean allInState(State targetState){
        for(State state : agentStates){
            if(state != targetState){
                return false;
            }
        }
        return true;
    }

    private void clearEnvErrors(){
        for(RealAgent a : agentToIndex.keySet()){
            a.setEnvError(false);
        }
    }

    /**
     * Set all agents above agent a in the chain to a new state (Including agent a)
     */
    private synchronized void setSuccessors(RealAgent a, State newState){
        for(int i = agentToIndex.get(a); i < agentStates.size(); i++){
            agentStates.set(i, newState);
        }
    }


    //                Position Checking
    private synchronized boolean isPositionOkay(RealAgent a, Point p){
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

    private synchronized boolean isPositionOkay(int range, Point a, Point b){
        return PropModel1.isConnected(baseStation.getOccupancyGrid(), range, a, b);
    }
}
