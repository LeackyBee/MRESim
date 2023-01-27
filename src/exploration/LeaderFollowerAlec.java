package exploration;

import agents.Agent;
import agents.RealAgent;
import communication.PropModel1;
import config.SimulatorConfig;

import java.awt.*;

public class LeaderFollowerAlec extends BasicExploration implements Exploration{

    LFComms comms;
    Point next;


    /**
     * Just builds the object and initializes the agent.
     *
     * @param agent        The agent using this ExplorationStrategy
     * @param simConfig
     * @param initialState
     */
    public LeaderFollowerAlec(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        super(agent, simConfig, initialState);
        comms = LFComms.register(agent);
        next = null;
    }

    @Override
    protected Point takeStep_explore(int timeElapsed) {
        return null;
    }

    @Override
    public Point takeStep(int timeElapsed) {
        agent.flushLog();
        if(timeElapsed == 0){
            // This exists so that the agents can gather information and transmit to the base station for planning
            return agent.stay();
        }

        return comms.getNextPosition(agent);
    }
}
