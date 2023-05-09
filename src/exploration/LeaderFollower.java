package exploration;

import agents.Agent;
import agents.RealAgent;
import config.SimulatorConfig;

import java.awt.*;

public class LeaderFollower extends BasicExploration implements Exploration{

    LFComms comms;


    /**
     * Just builds the object and initializes the agent.
     *
     * @param agent        The agent using this ExplorationStrategy
     * @param simConfig
     * @param initialState
     */
    public LeaderFollower(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        super(agent, simConfig, initialState);
        comms = LFComms.register(agent);
    }

    @Override
    protected Point takeStep_explore(int timeElapsed) {
        return null;
    }

    @Override
    public Point takeStep(int timeElapsed) {
        if(agent.getEnvError()){
            agent.getPath().resetStep();
        }

        if(timeElapsed == 0){
            // This exists so that the agents can gather information and transmit to the base station for planning
            // If we ignore this we get null paths
            return agent.stay();
        }
        return comms.getNextPosition(agent);
    }
}
