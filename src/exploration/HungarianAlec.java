package exploration;

import agents.Agent;
import agents.RealAgent;
import config.SimulatorConfig;

import java.awt.*;

public class HungarianAlec extends BasicExploration implements Exploration{

    private static final boolean EXACT_PATH = true;

    private enum AgentState{
        Explore,
        Return,
        Wait,
    }

    private AgentState agentState;

    private HungarianComms comms;

    private Point meetup = agent.baseStation.getLocation();
    private Point target;




    /**
     * Just builds the object and initializes the agent.
     *
     * @param agent        The agent using this ExplorationStrategy
     * @param simConfig
     * @param initialState
     */
    public HungarianAlec(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        super(agent, simConfig, initialState);
        agentState = AgentState.Return;
        comms = HungarianComms.register(agent);
    }

    @Override
    protected Point takeStep_explore(int timeElapsed) {
        agent.announce("Exploring");
        if(target.equals(meetup)){
            agent.announce("Didn't get a frontier :(");
            // This robot has not been given a frontier this time
            agentState = AgentState.Return;
            return takeStep_Return();
        }
        if(agent.getLocation().equals(target)){
            agent.announce("Reached Target");
            agentState = AgentState.Return;
            agent.setPath(agent.calculateAStarPath(meetup, EXACT_PATH));
            return takeStep_Return();
        }

        if(agent.getPath() == null || agent.getPath().isFinished()){
            agent.announce("Planning Explore Path");
            agent.setPath(agent.calculateAStarPath(target, EXACT_PATH));
        }

        if(agent.getEnvError()){
            agent.setEnvError(false);
            agentState = AgentState.Return;
            agent.setPath(agent.calculateAStarPath(meetup, EXACT_PATH));
            return takeStep_Return();
        }

        return agent.getNextPathPoint();
    }

    @Override
    public synchronized Point takeStep(int timeElapsed) {

        switch(agentState){
            case Explore:
                return takeStep_explore(timeElapsed);
            case Return:
                return takeStep_Return();
            case Wait:
                return takeStep_Wait();
        }

        return null;
    }

    private Point takeStep_Return(){
        agent.announce("Returning");
        if(agent.getLocation().equals(meetup)){
            agent.announce("Reached Meetup");
            agentState = AgentState.Wait;
            agent.setPath(null);
            comms.atRendezvous(agent);
            return agent.stay();
        }

        if(agent.getPath() == null || !agent.getPath().isValid() || agent.getEnvError()){
            agent.setEnvError(false);
            agent.announce("Planning Return Path");
            agent.setPath(agent.calculateAStarPath(meetup, EXACT_PATH));
        }

        return agent.getNextPathPoint();
    }

    private Point takeStep_Wait(){
        agent.announce("Waiting");
        if(comms.allAtRendezvous()){
            agent.announce("All at meetup!");
            agentState = AgentState.Explore;
            target = comms.getNextTarget(agent);
            meetup = comms.getMeetup();
        }
        return agent.stay();
    }
}
