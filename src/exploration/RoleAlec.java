package exploration;

import agents.Agent;
import agents.RealAgent;
import config.RobotConfig;
import config.SimConstants;
import config.SimulatorConfig;
import environment.Frontier;

import java.awt.*;
import java.util.ArrayList;

public class RoleAlec extends FrontierAlec{

    private enum State{
        ReturnToBase,
        RunningToRelay,
        WaitingAtRelay,
        Exploring
    }

    AlecCommunication comm;
    State agentState;

    int returnTimer;
    int returnTimerDefault;
    int partnerNumber;


    /**
     * Just builds the object and initializes the agent.
     *
     * @param agent        The agent using this ExplorationStrategy
     * @param simConfig
     * @param initialState
     */
    public RoleAlec(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        super(agent, simConfig, initialState);
        System.out.println("It's using the Role-Based");
        returnTimerDefault = 50;
        returnTimer = 50;
        agentState = State.WaitingAtRelay;
        agent.announce(String.valueOf(agent.getRobotNumber()));
        if(agent.getRole() == RobotConfig.roletype.Explorer){
            partnerNumber = agent.getParent();
            comm = AlecCommunication.getCommunication(agent.getRobotNumber());
        } else{
            partnerNumber = agent.getChild();
            comm = AlecCommunication.getCommunication(partnerNumber);

        }
        comm.setRendezvous(agent.baseStation.getLocation());

        System.out.println(comm.hashCode());
    }

    // ? to find time to return to relay, figure out round trip time of travelling to each relay inbetween using distance/speed
    @Override
    public Point takeStep(int timeElapsed) {
        agent.flushLog();
        agent.announce("Rendezvous is ".concat(comm.getRendezvous().toString()));
        agent.announce("Current Position is ".concat(agent.getLocation().toString()));
        if(timeElapsed == 0){
            return agent.baseStation.getLocation();
        }
        switch (agentState){
            case Exploring:
                agent.announce("Exploring");
                return takeStep_explore(timeElapsed);
            case ReturnToBase:
                agent.announce("Running To Base");
                return takeStep_ReturnToBase(timeElapsed);
            case RunningToRelay:
                agent.announce("Running to Rendezvous");
                return takeStep_RunningToRelay(timeElapsed);
            case WaitingAtRelay:
                agent.announce("Waiting at Rendezvous");
                return takeStep_WaitingAtRelay(timeElapsed);
            default:
                agent.announce("Default");
                return super.takeStep(timeElapsed);
        }
    }

    @Override
    protected Point takeStep_explore(int timeElapsed) {
        returnTimer = returnTimer-1;
        if(returnTimer == 0){
            agentState = State.RunningToRelay;
            if(agent.getPath() != null){
                agent.getPath().setInvalid();
            }
            return comm.getRendezvous();
        }
        return super.takeStep(timeElapsed);
    }

    private Point takeStep_ReturnToBase(int timeElapsed){
        agent.setPathToBaseStation(false);
        if(agent.getTeammateByNumber(SimConstants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()){
            agentState = State.RunningToRelay;
            return takeStep_RunningToRelay(timeElapsed);
        }
        return agent.getNextPathPoint();
    }

    private Point takeStep_RunningToRelay(int timeElapsed){
        if(isNearEnough(agent.getLocation(), comm.getRendezvous())){
            agentState = State.WaitingAtRelay;
            return takeStep_WaitingAtRelay(timeElapsed);
        }

        if(agent.getEnvError() && exactPath){
            agent.announce("Randomly Walking");
            agent.setEnvError(false);
            return RandomWalk.randomStep(agent, agent.getSpeed());
        } else if(agent.getEnvError()){
            if (agent.getPath() != null){
                agent.getPath().setInvalid();
            }
            agent.announce("Planning Path");
            exactPath = true;
            agent.setEnvError(false);
            path = agent.calculatePath(destination, true);
            agent.setPath(path);
            return agent.getNextPathPoint();
        } else if(exactPath){
            agent.announce("Following Path");
            return agent.getNextPathPoint();
        } else {
            return comm.getRendezvous();
        }
    }

    // TODO: path planning for relay
    private Point takeStep_WaitingAtRelay(int timeElapsed){
        if(agent.getEnvError()){
            // One cause of EnvErrors is standing still, so we need to remove this before changing to the next state
            agent.setEnvError(false);
        }

        // Uncomment to see where the agent is and if it is actually at the rendezvous
        //agent.announce(agent.getLocation().toString().concat(" : ").concat(comm.getRendezvous().toString()));

        if(!isNearEnough(agent.getLocation(), comm.getRendezvous())){
            agent.announce("Not Near Enough");
            agentState = State.RunningToRelay;
            return takeStep_RunningToRelay(timeElapsed);
        }

        if(agent.getTeammate(partnerNumber).hasCommunicationLink()){
            if(agent.isExplorer()){
                agentState = State.Exploring;
                return takeStep_explore(timeElapsed);
            } else {
                agentState = State.ReturnToBase;
                comm.setRendezvous(chooseFrontier(getCommunications(), partnerNumber));
                agent.announce("Rendezvous set to ".concat(comm.getRendezvous().toString()));
                return takeStep_ReturnToBase(timeElapsed);
            }
        }
        return null;
    }

    private boolean isNearEnough(Point p1, Point p2){
        return p1.distance(p2) < 10;
    }
}
