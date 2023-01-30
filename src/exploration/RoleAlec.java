package exploration;

import agents.Agent;
import agents.RealAgent;
import config.RobotConfig;
import config.SimConstants;
import config.SimulatorConfig;

import java.awt.*;
//TODO: Better return timer reset value
public class RoleAlec extends FrontierAlec{

    private static final int RETURN_TIMER_DEFAULT = 50;
    private static final State INITIAL_STATE = State.RunningToRelay;
    private static final int COMMUNICATION_TIMEOUT = 10;


    private enum State{
        ReturnToBase,
        RunningToRelay,
        WaitingAtRelay,
        Exploring
    }

    private final AlecCommunication comm;
    private State agentState;

    private int returnTimer;

    private final int partnerNumber;
    // The number of timesteps between successive communications
    // Prevents the robots infinitely recognising they are talking and never escaping
    private int timeSinceLastComm = Integer.MAX_VALUE - 10;



    /**
     * Just builds the object and initializes the agent.
     *
     * @param agent        The agent using this ExplorationStrategy
     * @param simConfig    Passed to super()
     * @param initialState Passed to super()
     */
    public RoleAlec(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        super(agent, simConfig, initialState);
        super.disableTimer();
        agent.announce("Initiated Role-Based");

        returnTimer = RETURN_TIMER_DEFAULT;
        agentState = INITIAL_STATE;

        if(agent.getRole() == RobotConfig.roletype.Explorer){
            partnerNumber = agent.getParent();
            comm = AlecCommunication.getCommunication(agent.getRobotNumber());
        } else{
            partnerNumber = agent.getChild();
            comm = AlecCommunication.getCommunication(partnerNumber);

        }

        comm.setRendezvous(agent.baseStation.getLocation());
    }

    @Override
    public Point takeStep(int timeElapsed) {
        agent.flushLog();
        if(timeElapsed == 0){
            return agent.stay();
        }

        timeSinceLastComm++;

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
        returnTimer = returnTimer-1;

        if(returnTimer == 0){
            agentState = State.RunningToRelay;
            if(agent.getPath() != null){
                agent.getPath().setInvalid();
            }
            destination = null;
            agent.setPath(agent.calculatePath(comm.getRendezvous(), false));
            return takeStep_RunningToRelay(timeElapsed);
        }


        // If relay is met while exploring, we may as well skip going to the rendezvous point and find a new one
        if(agent.getTeammate(partnerNumber).hasCommunicationLink() && timeSinceLastComm >= COMMUNICATION_TIMEOUT){
            return meetRendezvous(timeElapsed);
        }

        return super.takeStep(timeElapsed);
    }

    private Point takeStep_ReturnToBase(int timeElapsed){
        agent.announce("Running To Base");

        if(agent.getEnvError()){
            exactPath = true;
            agent.setPathInvalid();
        }

        if(agent.getPath() == null){
            agent.announce("Planning New Path to Base Station");
            agent.setPathToBaseStation(exactPath);
            exactPath = false;
        }


        if(agent.getTeammate(partnerNumber).hasCommunicationLink() && timeSinceLastComm >= COMMUNICATION_TIMEOUT){
            return meetRendezvous(timeElapsed);
        }


        // Can now switch to "RunningToRelay"
        if(agent.getTeammateByNumber(SimConstants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()){
            agent.setPathInvalid();
            agent.setPath(agent.calculatePath(comm.getRendezvous(), false));
            agentState = State.RunningToRelay;
            return takeStep_RunningToRelay(timeElapsed);
        } else {
            return agent.getNextPathPoint();
        }
    }

    private Point takeStep_RunningToRelay(int timeElapsed){
        agent.announce("Running to Rendezvous");

        if(agent.getTeammate(partnerNumber).hasCommunicationLink() && timeSinceLastComm >= COMMUNICATION_TIMEOUT){
            return meetRendezvous(timeElapsed);
        }


        if(agent.getLocation().equals(comm.getRendezvous())){
            agent.announce("Reached Rendezvous");
            if(agent.getPath() != null){
                agent.getPath().setInvalid();
            }
            agent.setPath(null);
            agentState = State.WaitingAtRelay;
            return takeStep_WaitingAtRelay(timeElapsed);
        } else if(agent.getPath() != null && agent.getPath().isFinished()){
            agent.announce("Path did not end at rendezvous, replanning (problem)");
            agent.getPath().setInvalid();
            agent.setPath(null);
        } // Shouldn't get triggered

        if(agent.getEnvError()){
            agent.setPathInvalid();
            exactPath = true;
        }


        if(agent.getPath() == null){
            agent.announce("Planning path to rendezvous");
            agent.setPath(agent.calculatePath(comm.getRendezvous(), exactPath));
            exactPath = false;
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

        if(agent.getTeammate(partnerNumber).hasCommunicationLink() && timeSinceLastComm >= COMMUNICATION_TIMEOUT){
            return meetRendezvous(timeElapsed);
        } else {
            return agent.stay();
        }
    }

    private synchronized Point meetRendezvous(int timeElapsed){
        returnTimer = RETURN_TIMER_DEFAULT;
        agent.announce("Met Partner");
        // Make sure the explorer passes here first
        if(!agent.isExplorer() && !comm.isRendezvousChanged()){
            System.out.println("caught");
            System.out.println(agent.getName());
            return agent.stay();
        }

        timeSinceLastComm = 0;

        if(agent.getPath() != null){
            agent.getPath().setInvalid();
        }

        // This section ensures that only one of the pair change the rendezvous, and deals with race conditions
        if(agent.isExplorer()){
            // destination must be null for chooseFrontier to run
            destination = null;

            chooseFrontier(getCommunications(), agent.isExplorer() ? agent.getRobotNumber() : partnerNumber);
            comm.setRendezvous(destination);
            agent.announce("Rendezvous set to ".concat(comm.getRendezvous().toString()));

            agentState = State.Exploring;
            agent.setPath(agent.calculatePath(destination,false));
            return takeStep_explore(timeElapsed);
        } else{
            // this clears the branch for the next time we get here
            comm.ackRendezvous();
            agentState = State.ReturnToBase;
            agent.setPathToBaseStation(false);
            return takeStep_ReturnToBase(timeElapsed);
        }

    }
}
