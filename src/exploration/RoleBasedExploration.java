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
import java.util.stream.Stream;

public class RoleBasedExploration extends BasicExploration implements Exploration{

    private static final int RETURN_TIMER_DEFAULT = 50;
    private static final State INITIAL_STATE = State.RunningToRelay;
    private static final int COMMUNICATION_TIMEOUT = 10;
    private static final boolean EXACT_PATH = true;


    private enum State{
        ReturnToBase,
        RunningToRelay,
        WaitingAtRelay,
        Exploring
    }

    private final RoleComms comm;
    private State agentState = INITIAL_STATE;
    private int returnTimer = RETURN_TIMER_DEFAULT;
    private final int partnerNumber;
    // The number of timesteps between successive communications
    // Prevents the robots infinitely recognising they are talking and never escaping
    private int timeSinceLastComm = Integer.MAX_VALUE - 10;

    private boolean stillInComms = false;

    private final PriorityQueue<Frontier> frontiers = new PriorityQueue<>();

    private Point destination;
    private Frontier frontierTarget;

    /**
     * Just builds the object and initializes the agent.
     *
     * @param agent        The agent using this ExplorationStrategy
     * @param simConfig    Passed to super()
     * @param initialState Passed to super()
     */
    public RoleBasedExploration(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        super(agent, simConfig, initialState);
        agent.announce("Initiated Role-Based");

        if(agent.getRole() == RobotConfig.roletype.Explorer){
            partnerNumber = agent.getParent();
            comm = RoleComms.getCommunication(agent.getRobotNumber());
            comm.setExplorer(agent);
        } else{
            partnerNumber = agent.getChild();
            comm = RoleComms.getCommunication(partnerNumber);
            comm.setRelay(agent);
        }
        comm.setRendezvous(agent.baseStation.getLocation());
    }

    @Override
    public Point takeStep(int timeElapsed) {
        agent.flushLog();
        if(timeElapsed == 0){
            return agent.stay();
        }

        if(!agent.getTeammateByNumber(partnerNumber).hasCommunicationLink()){
            stillInComms = false;
            timeSinceLastComm++;
        }

        agent.announce("Rendezvous is ".concat(comm.getRendezvous().toString()));
        agent.announce("Current Position is ".concat(agent.getLocation().toString()));
        if(agent.isExplorer() && destination != null){
            agent.announce("Destination is ".concat(destination.toString()));
        }


        switch (agentState) {
            case Exploring:
                return takeStep_explore(timeElapsed);
            case ReturnToBase:
                return takeStep_ReturnToBase(timeElapsed);
            case RunningToRelay:
                return takeStep_RunningToRelay(timeElapsed);
            case WaitingAtRelay:
                return takeStep_WaitingAtRelay(timeElapsed);
            default:
                throw new IllegalArgumentException();
        }
    }

    @Override
    protected Point takeStep_explore(int timeElapsed) {
        agent.announce("Exploring");
        returnTimer--;

        if(returnTimer == 0){
            agentState = State.RunningToRelay;
            agent.setPathInvalid();
            destination = null;
            agent.setPath(agent.calculatePath(comm.getRendezvous(), EXACT_PATH));
            return takeStep_RunningToRelay(timeElapsed);
        }

        if(agent.getEnvError()){
            agent.setEnvError(false);
            agent.setPathInvalid();
            agent.setPath(null);
            destination = null;
        }


        // If relay is met while exploring, we may as well skip going to the rendezvous point and find a new one
        if(agent.getTeammate(partnerNumber).hasCommunicationLink() && timeSinceLastComm >= COMMUNICATION_TIMEOUT && !stillInComms){
            return meetRendezvous();
        }

        if(agent.getPath() != null && !agent.getPath().isFinished() && agent.getPath().isValid()){
            return agent.getNextPathPoint();
        } else{
            // Need to plan a new path
            chooseFrontier();

            if(destination != null && !agent.isMissionComplete()) {
                agent.announce("New Destination: ".concat(destination.toString()));
                agent.setPath(agent.calculatePath(destination, EXACT_PATH));
                while(agent.getPath() == null || !agent.getPath().found || !agent.getPath().isValid()){
                    agent.addBadFrontier(frontierTarget);
                    if(frontierTarget == null){
                        agentState = State.RunningToRelay;
                        return takeStep(timeElapsed);
                    }
                    agent.announce("Added to bad frontiers: ".concat(frontierTarget.toString()));
                    destination = null;
                    chooseFrontier();
                    agent.setPath(agent.calculatePath(destination,EXACT_PATH));
                }
            }
        }
        return agent.getNextPathPoint();
    }

    private Point takeStep_ReturnToBase(int timeElapsed){
        agent.announce("Running To Base");

        if(agent.getEnvError()){
            agent.setPathInvalid();
        }

        if(agent.getPath() == null){
            agent.announce("Planning New Path to Base Station");
            agent.setPathToBaseStation(EXACT_PATH);
        }

        if(agent.getTeammate(partnerNumber).hasCommunicationLink() && timeSinceLastComm >= COMMUNICATION_TIMEOUT && !stillInComms){
            return meetRendezvous();
        }

        // Can now switch to "RunningToRelay"
        if(agent.getTeammateByNumber(SimConstants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()){
            agent.setPathInvalid();
            agent.setPath(agent.calculatePath(comm.getRendezvous(), EXACT_PATH));
            agentState = State.RunningToRelay;
            return takeStep_RunningToRelay(timeElapsed);
        } else {
            return agent.getNextPathPoint();
        }
    }

    private Point takeStep_RunningToRelay(int timeElapsed){
        agent.announce("Running to Rendezvous");

        if(agent.getTeammate(partnerNumber).hasCommunicationLink() && timeSinceLastComm >= COMMUNICATION_TIMEOUT && !stillInComms){
            return meetRendezvous();
        }

        if(agent.getLocation().equals(comm.getRendezvous())){
            agent.announce("Reached Rendezvous");
            agent.setPathInvalid();
            agent.setPath(null);
            agentState = State.WaitingAtRelay;
            return takeStep_WaitingAtRelay(timeElapsed);
        }


        if(agent.getEnvError()){
            agent.announce("Dealt with Env Error");
            agent.setEnvError(false);
            agent.setPathInvalid();
            agent.setPath(null);
        }


        if(agent.getPath() == null || !agent.getPath().isValid() || !agent.getPath().found){
            agent.announce("Planning path to rendezvous");
            agent.setPath(agent.calculatePath(comm.getRendezvous(), EXACT_PATH));
        }


        return agent.getNextPathPoint();
    }

    private Point takeStep_WaitingAtRelay(int timeElapsed){
        agent.announce("Waiting at Rendezvous");

        // Should never get triggered
        if(!agent.getLocation().equals(comm.getRendezvous())){
            agent.announce("Not Actually At Rendezvous (Problem)");
            agentState = State.RunningToRelay;
            return takeStep_RunningToRelay(timeElapsed);
        }

        if(agent.getTeammate(partnerNumber).hasCommunicationLink()){
            return meetRendezvous();
        } else {
            return agent.stay();
        }
    }

    private synchronized Point meetRendezvous(){
        agent.announce("Met Partner");
        stillInComms = true;

        timeSinceLastComm = 0;

        agent.setPathInvalid();

        if(agent.isExplorer()){
            // destination must be null for chooseFrontier to run
            destination = null;

            chooseFrontier();
            agent.announce("Rendezvous set to ".concat(comm.getRendezvous().toString()));
            agentState = State.Exploring;
            agent.setPath(agent.calculatePath(destination,EXACT_PATH));
            while(agent.getPath() == null || !agent.getPath().found || !agent.getPath().isValid()){
                agent.addBadFrontier(frontierTarget);
                agent.announce("Added to bad frontiers: ".concat(frontierTarget.toString()));
                destination = null;
                chooseFrontier();
                // TODO: if frontiers == empty, exit
                agent.setPath(agent.calculatePath(destination,EXACT_PATH));
            }

            comm.setRendezvous(agent.getPath().getMidpoint());
            returnTimer = getReturnTimer();
            agent.announce("New return timer: ".concat(String.valueOf(returnTimer)));
        } else{
            agent.setOccupancyGrid(comm.getExplorer().getOccupancyGrid());
            agentState = State.ReturnToBase;
            agent.setPathToBaseStation(EXACT_PATH);
        }
        return agent.getNextPathPoint();
    }

    // Boilerplate code to find the frontiers
    private void chooseFrontier(){
        // if we don't know where to go, or have reached our destination, choose a new one.
        if (destination == null || agent.getLocation().equals(destination)) {
            // figure out the options
            calculateFrontiers();

            // figure out which option we choose
            long index = getCommunications().filter(a -> a.getRobotNumber() < agent.getRobotNumber())
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
            frontierTarget = frontier;
            if(frontier == null){
                // should only occur if the frontier list is empty, which tells us we need to go back to the base
                destination = agent.baseStation.getLocation();
            } else{
                destination = frontier.getCentre();
            }
        }
    }

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
            if(frontier.getArea() >= SimConstants.MIN_FRONTIER_SIZE && !agent.isBadFrontier(frontier) && frontier != frontierTarget){
                frontiers.add(frontier);
            }
        }
    }


    private Stream<TeammateAgent> getCommunications(){
        return agent.getAllTeammates().values().stream()
                .filter(TeammateAgent::hasCommunicationLink).filter(a -> a.getRole() == RobotConfig.roletype.Explorer);
    }

    // Provides a fairly good estimate for how long the relay will take to return
    private int getReturnTimer(){
        double l1 = agent.calculatePath(agent.baseStation.getLocation(), false).getLength();
        double l2 = agent.baseStation.calculatePath(comm.getRendezvous(), false).getLength();
        int timer = (int) ((l1+l2)/agent.getSpeed());
        if(timer == 0){
            return 10;
        } else{
            return timer;
        }
    }
}
