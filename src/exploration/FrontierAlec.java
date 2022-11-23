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
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;

public class FrontierAlec extends BasicExploration implements Exploration{

    PriorityQueue<Frontier> frontiers = new PriorityQueue<>();
    Point destination = null;
    boolean exactPath = false;


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
        System.out.println("take-step-explore");
        calculateFrontiers();
        //chooseFrontier();
        if(destination != null){
            return destination;
        } else{
            return agent.baseStation.getLocation();
        }
    }

    protected Stream<TeammateAgent> getCommunications(){
        return agent.getAllTeammates().values().stream()
                .filter(TeammateAgent::hasCommunicationLink).filter(a -> a.getRole() == RobotConfig.roletype.Explorer);
    }

    @Override
    public Point takeStep(int timeElapsed) {

        Stream<TeammateAgent> communications = getCommunications();

        AtomicBoolean newComm = new AtomicBoolean(false);

        if(newComm.get()){
            path = null;
            exactPath = false;
            destination = null;
            agent.setEnvError(false);
            agent.getPath().setInvalid();
        }

        if(agent.getLocation().equals(destination)) {
            announce("Destination Reached");
            destination = null;
            if (agent.getPath() != null) {
                agent.getPath().setInvalid();
                exactPath = false;
            }
        }

        if(agent.getEnvError() && exactPath){
            announce("Randomly Walking");
            agent.setEnvError(false);
            return RandomWalk.randomStep(agent, agent.getSpeed());
        } else if(agent.getEnvError()){
            if (agent.getPath() != null){
                agent.getPath().setInvalid();
            }
            announce("Planning Path");
            exactPath = true;
            agent.setEnvError(false);
            path = agent.calculatePath(destination, true);
            agent.setPath(path);
            return agent.getNextPathPoint();
        } else if(exactPath){
            announce("Following Path");
            return agent.getNextPathPoint();
        }

        chooseFrontier(communications, agent.getRobotNumber());
        if(destination != null && !agent.isMissionComplete()){
            announce("New Destination: ".concat(destination.toString()));
            return destination;
        } else {
            announce("Base Station");
            return agent.baseStation.getLocation();
        }


    }

    // Prepends the robots number, so it is easy to tell which robot triggered the statement.
    private void announce(String message){
        System.out.println("(".concat(String.valueOf(agent.getRobotNumber())).concat(") ").concat(message));
    }


    protected void chooseFrontier(Stream<TeammateAgent> communications, int robotNumber){
        if (destination == null || agent.getLocation().equals(destination)) {
            // figure out the options
            calculateFrontiers();


            // figure out which option we choose
            long index = communications.filter(a -> a.getRobotNumber() < robotNumber)
                    .count();

            // choose the option
            Frontier frontier = frontiers.poll();
            for(int i = 0; i < index % frontiers.size(); i++){
                frontier = frontiers.poll();
            }
            if(frontier == null){
                destination = null;
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
}
