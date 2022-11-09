package exploration;

import agents.Agent;
import agents.RealAgent;
import config.SimulatorConfig;

import java.awt.*;

public class FrontierAlec extends BasicExploration implements Exploration{

    /**
     * Just builds the object and initializes the agent.
     *
     * @param agent        The agend using this ExplorationStrategy
     * @param simConfig
     * @param initialState
     */
    public FrontierAlec(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        super(agent, simConfig, initialState);
    }

    @Override
    protected Point takeStep_explore(int timeElapsed) {
        return new Point(agent.getLocation().x+1,agent.getLocation().y+1);
    }

    @Override
    public Point takeStep(int timeElapsed) {
        return new Point(agent.getLocation().x+1,agent.getLocation().y+1);
    }


    private void calculateFrontiers(){

    }
}
