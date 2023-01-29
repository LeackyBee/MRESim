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

    PriorityQueue<Frontier> frontiers = new PriorityQueue<>();
    Point destination = null;
    boolean exactPath = false;

    HashSet<Integer> oldComms = new HashSet<>();


    /**
     * @param agent        The agent using this ExplorationStrategy
     */
    public FrontierAlec(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        super(agent, simConfig, initialState);
        System.out.println("Frontier Alec Initialised");
    }

    @Override
    protected Point takeStep_explore(int timeElapsed) {
        // not needed
        return null;
    }

    protected Stream<TeammateAgent> getCommunications(){
        return agent.getAllTeammates().values().stream()
                .filter(TeammateAgent::hasCommunicationLink).filter(a -> a.getRole() == RobotConfig.roletype.Explorer);
    }

    @Override
    public Point takeStep(int timeElapsed) {
        agent.flushLog();

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
            exactPath = false;
            destination = null;
            agent.setEnvError(false);
            if(agent.getPath() != null){
                agent.getPath().setInvalid();
            }
            agent.setPath(null);
            agent.setEnvError(false);
        }

        // If we have an EnvError the agent is stuck, so we need an exact path
        if(agent.getEnvError()){
            agent.setEnvError(false);
            agent.getPath().setInvalid();
            exactPath = true;
        } else{
            exactPath = false;
        }

        if(agent.getPath() != null && !agent.getPath().isFinished()){
            return agent.getNextPathPoint();
        }

        chooseFrontier(getCommunications(), agent.getRobotNumber());

        if(destination != null && !agent.isMissionComplete()){
            agent.announce("New Destination: ".concat(destination.toString()));
            //agent.setPath(agent.calculatePath(agent.getLocation(), destination, true, exactPath));
            agent.setPath(agent.calculatePath(destination,exactPath));
            return agent.getNextPathPoint();
        } else {
            agent.setPathToBaseStation(exactPath);
            agent.announce("Going to Base Station");
            return agent.getNextPathPoint();
        }


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
            if(frontier.getArea() >= SimConstants.MIN_FRONTIER_SIZE && !agent.isBadFrontier(frontier)){
                frontiers.add(frontier);
            }
        }

    }

    public PriorityQueue<Frontier> getFrontiers() {
        return frontiers;
    }

    protected void dealWithEnvError(){
        if(agent.getPath() != null){
            agent.getPath().setInvalid();
            agent.setPath(null);
        }
        agent.setEnvError(false);
    }
}
