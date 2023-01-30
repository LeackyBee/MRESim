package exploration;

import agents.Agent;
import agents.RealAgent;
import agents.TeammateAgent;
import config.RobotConfig;
import config.SimConstants;
import config.SimulatorConfig;
import environment.ContourTracer;
import environment.Frontier;

import java.awt.*;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;

public class FrontierAlec extends BasicExploration implements Exploration{

    private static final boolean EXACT_PATH = true;
    private static int RETURN_TIMER = 100;
    private boolean timerEnabled = true;
    private int timer = 0;



    private enum State{
        EXPLORE,
        RETURN,
    }

    private State state = State.RETURN;

    PriorityQueue<Frontier> frontiers = new PriorityQueue<>();
    Point destination = null;
    boolean exactPath = false;

    Frontier tempBad = null;
    HashSet<Integer> oldComms = new HashSet<>();


    /**
     * @param agent The agent using this ExplorationStrategy
     */
    public FrontierAlec(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        super(agent, simConfig, initialState);
        agent.announce("Frontier-Based Initialised");
    }


    @Override
    public Point takeStep(int timeElapsed) {
        agent.flushLog();
        if(timeElapsed == 0){
            return agent.stay();
        }

        switch(state){
            case RETURN:
                return takeStep_ReturnToBase(timeElapsed);
            default: // or EXPLORE
                return takeStep_explore(timeElapsed);
        }
    }

    @Override
    protected Point takeStep_explore(int timeElapsed) {
        agent.announce("Exploring");
        if(timer == RETURN_TIMER && timerEnabled){
            agent.announce("Timer expired, returning to base");
            agent.setPathToBaseStation(EXACT_PATH);
            state = State.RETURN;
            RETURN_TIMER += 50;
            timer = 0;
            return takeStep_ReturnToBase(timeElapsed);
        } else {
            timer++;
        }

        // Checks if the robot has begun communicating with another robot
        // This check is important as this signals that new data has been received
        AtomicBoolean newComm = new AtomicBoolean(false);

        HashSet<Integer> newComms = new HashSet<>();
        getCommunications()
                .forEach(a ->
                {
                    newComm.set(newComm.get() || !oldComms.contains(a.getRobotNumber()));
                    newComms.add(a.getRobotNumber());
                });

        oldComms = newComms;

        // if there has been a new communication, reset variables
        if(newComm.get()){
            agent.announce("New Communication, resetting");
            destination = null;
            agent.setPathInvalid();
            agent.setPath(null);
            agent.setEnvError(false);
        }

        // If we have an EnvError the agent is stuck, so we need to replan
        if(agent.getEnvError()){
            agent.setEnvError(false);
            agent.setPathInvalid();
            agent.addBadFrontier(tempBad);
            agent.setPath(null);
            destination = null;
        }

        if(agent.getPath() != null && !agent.getPath().isFinished()){
            return agent.getNextPathPoint();
        }

        chooseFrontier(getCommunications(), agent.getRobotNumber());

        if(destination != null && !agent.isMissionComplete()){
            agent.announce("New Destination: ".concat(destination.toString()));
            agent.setPath(agent.calculatePath(destination,EXACT_PATH));
            if(agent.getPath() == null || !agent.getPath().found || !agent.getPath().isValid()){
                agent.addBadFrontier(tempBad);
                agent.announce("Added to bad frontiers: ".concat(tempBad.toString()));
                destination = null;
                agent.setEnvError(true);
                return agent.stay();
            }

            return agent.getNextPathPoint();
        } else {
            disableTimer();
            agent.setPathToBaseStation(EXACT_PATH);
            agent.announce("Mission over, going back to base station");
            return agent.getNextPathPoint();
        }
    }

    private Point takeStep_ReturnToBase(int timeElapsed){
        agent.announce("Returning to Base");
        if(agent.getTeammateByNumber(SimConstants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()){
            agent.setPathInvalid();
            state = State.EXPLORE;
            calculateFrontiers();
            agent.setPath(agent.calculatePath(destination, EXACT_PATH));

            return takeStep_explore(timeElapsed);
        }

        if(agent.getPath() == null || agent.getEnvError()){
            agent.setEnvError(false);
            agent.setPathToBaseStation(EXACT_PATH);
        }

        return agent.getNextPathPoint();
    }

    protected void chooseFrontier(Stream<TeammateAgent> communications, int robotNumber){
        // if we don't know where to go, or have reached our destination, choose a new one.
        if (destination == null || agent.getLocation().equals(destination)) {
            // figure out the options
            calculateFrontiers();

            // figure out which option we choose
            long index = communications.filter(a -> a.getRobotNumber() < robotNumber)
                    .count();

            // choose the option
            if(frontiers.size() == 0){
                index = 0;
            } else{
                index = index % frontiers.size();
            }

            Frontier frontier = frontiers.poll();
            for(int i = 0; i < index; i++){
                frontier = frontiers.poll();
            }
            tempBad = frontier;
            if(frontier == null){
                // should only occur if the frontier list is empty, which tells us we need to go back to the base
                destination = agent.baseStation.getLocation();
            } else{
                destination = frontier.getCentre();
            }
        }
    }

    // Boilerplate code to find the frontiers
    private void calculateFrontiers(){
        // Set the old list of frontiers to dirty, and clear the queue
        // this "dirtying" is for rendering, and so isn't strictly needed
        frontiers.forEach((f) -> agent.addDirtyCells(f.getPolygonOutline()));
        frontiers.clear();

        // Find all the boundaries between the "known" tiles and "unknown tiles"
        LinkedList<LinkedList<Point>> contours = ContourTracer.findAllContours(agent.getOccupancyGrid());

        // convert the contours to frontiers, and filter out all those that are invalid
        for (LinkedList<Point> contour : contours) {
            Frontier frontier = new Frontier(agent.getX(), agent.getY(), contour);
            if(frontier.getArea() >= SimConstants.MIN_FRONTIER_SIZE && !agent.isBadFrontier(frontier) && frontier != tempBad){
                frontiers.add(frontier);
            }
        }

    }

    public PriorityQueue<Frontier> getFrontiers() {
        return frontiers;
    }


    protected Stream<TeammateAgent> getCommunications(){
        return agent.getAllTeammates().values().stream()
                .filter(TeammateAgent::hasCommunicationLink).filter(a -> a.getRole() == RobotConfig.roletype.Explorer);
    }

    protected void disableTimer(){
        timerEnabled = false;
    }
}
