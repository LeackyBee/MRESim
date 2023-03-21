/*
 *     Copyright 2010, 2015, 2017 Julian de Hoog (julian@dehoog.ca),
 *     Victor Spirin (victor.spirin@cs.ox.ac.uk),
 *     Christian Clausen (christian.clausen@uni-bremen.de
 *
 *     This file is part of MRESim 2.3, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our papers:
 *
 *     @inproceedings{deHoog2009,
 *         title = "Role-Based Autonomous Multi-Robot Exploration",
 *         author = "Julian de Hoog, Stephen Cameron and Arnoud Visser",
 *         year = "2009",
 *         booktitle =
 *     "International Conference on Advanced Cognitive Technologies and Applications (COGNITIVE)",
 *         location = "Athens, Greece",
 *         month = "November",
 *     }
 *
 *     @incollection{spirin2015mresim,
 *       title={MRESim, a Multi-robot Exploration Simulator for the Rescue Simulation League},
 *       author={Spirin, Victor and de Hoog, Julian and Visser, Arnoud and Cameron, Stephen},
 *       booktitle={RoboCup 2014: Robot World Cup XVIII},
 *       pages={106--117},
 *       year={2015},
 *       publisher={Springer}
 *     }
 *
 *     MRESim is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     MRESim is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License along with MRESim.
 *     If not, see <http://www.gnu.org/licenses/>.
 */
package exploration;

import agents.Agent;
import agents.RealAgent;
import agents.TeammateAgent;
import config.SimConstants;
import config.SimulatorConfig;
import environment.ContourTracer;
import environment.Frontier;
import environment.OccupancyGrid;
import environment.TopologicalMap;
import exploration.Frontier.FrontierUtility;
import exploration.rendezvous.NearRVPoint;
import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.PriorityQueue;
import path.Path;
import path.TopologicalNode;

/**
 * Calculate Frontiers, weight them by utility and decide which one to explore first.
 *
 * @author julh, Christian Clausen
 */
public class FrontierExploration extends BasicExploration implements Exploration {

    SimulatorConfig.frontiertype frontierExpType;
    RealAgent baseStation;
    int noReturnTimer;

    PriorityQueue<Frontier> frontiers;
    Frontier lastFrontier;          // Keep track of last frontier of interest
    //Frontiers that are impossible to reach, so should be discarded
    HashMap<Frontier, Boolean> badFrontiers;
    private double last_percentage_known = 0;
    private int no_change_counter = 0;
    private int max_no_change_counter = 0;
    SimulatorConfig.relaytype relayType = SimulatorConfig.relaytype.None;
    TopologicalMap tmap;
    private boolean relay_handling = true;

    /**
     * Normal Constructor
     *
     * @param agent
     * @param simConfig
     * @param baseStation
     */
    public FrontierExploration(RealAgent agent, SimulatorConfig simConfig, RealAgent baseStation) {
        super(agent, simConfig, Agent.ExplorationState.Initial);
        this.agent = agent;
        this.frontierExpType = simConfig.getFrontierAlgorithm();
        this.baseStation = baseStation;
        this.noReturnTimer = 0;
        this.frontiers = new PriorityQueue<>();
        this.relayType = simConfig.getRelayAlgorithm();
        this.tmap = agent.getTopologicalMap();
    }

    /**
     * Constructor for Explorations using a specific frontier-type
     *
     * @param agent
     * @param simConfig
     * @param baseStation
     * @param frontierType
     */
    protected FrontierExploration(RealAgent agent, SimulatorConfig simConfig, RealAgent baseStation, SimulatorConfig.frontiertype frontierType) {
        super(agent, simConfig, Agent.ExplorationState.Initial);
        this.relayType = simConfig.getRelayAlgorithm();
        this.frontierExpType = frontierType;
        this.baseStation = baseStation;
        this.noReturnTimer = 0;
        this.frontiers = new PriorityQueue<>();
        this.tmap = agent.getTopologicalMap();
        this.relay_handling = false;
    }

    @Override
    public Point takeStep(int timeElapsed) {
        Point nextStep;

        //Preprocessing
        if (agent.getEnvError()) {
            agent.setEnvError(false);
            agent.getPath().setInvalid();
            this.recentEnvError = true;
        }

        if (agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()) {
            agent.getStats().setTimeLastDirectContactCS(1);
            agent.getStats().setLastContactAreaKnown(agent.getStats().getAreaKnown());
        } else {
            agent.getStats().incrementLastDirectContactCS();
        }

        //Statemachine
        switch (agent.getExploreState()) {
            case Initial:
                nextStep = RandomWalk.randomStep(agent, 10);
                agent.getStats().setTimeSinceLastPlan(0);
                if (timeElapsed >= SimConstants.INIT_CYCLES - 1) {
                    agent.setExploreState(Agent.ExplorationState.Explore);
                }
                break;
            case Explore:
                if (((agent.getStats().getTimeSinceLastPlan() < SimConstants.REPLAN_INTERVAL && agent.getStateTimer() > 1)
                        && agent.getPath() != null && agent.getPath().isValid())) {
                    nextStep = processRelay();
                    //nextStep = agent.getPath().nextPoint();
                } else {
                    nextStep = takeStep_explore(timeElapsed);
                }
                break;
            case ReturnToBase:
                if (agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()) {
                    agent.setExploreState(Agent.ExplorationState.Explore);
                }
                if (agent.getStateTimer() <= 1 || agent.getPath() == null || !agent.getPath().isValid()) {
                    agent.setPathToBaseStation(recentEnvError);
                }
                if (agent.getPath().isValid()) {
                    nextStep = agent.getPath().nextPoint();
                } else {
                    nextStep = RandomWalk.randomStep(agent, 4);
                }
                break;
            case SettingRelay:
                agent.dropComStation();
                agent.setExploreState(agent.getPrevExploreState());
                nextStep = agent.stay();
                break;
            case TakingRelay:
                agent.liftComStation();
                agent.setExploreState(agent.getPrevExploreState());
                nextStep = agent.stay();
                break;
            case GoToRelay:
                nextStep = takeStep_GoToRelay();
                break;
            case Finished:
            default:
                nextStep = agent.stay();
            //nextStep = RandomWalk.randomStep(agent, 4);

        }

        //postprocessing
        agent.getStats().incrementTimeSinceLastPlan();
        if (agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()) {
            noReturnTimer = 0;
        } else {
            noReturnTimer++;
        }
        this.recentEnvError = false;
        agent.setDynamicInfoText("" + noReturnTimer + "/" + (int) (simConfig.PERIODIC_RETURN_PERIOD * ((double) (max_no_change_counter + 10) / 10)));
        max_no_change_counter = Math.max(max_no_change_counter, no_change_counter);
        return nextStep;
    }

    @Override
    protected Point takeStep_explore(int timeElapsed) {
        Point nextStep;
        if (last_percentage_known == agent.getStats().getPercentageKnown() && agent.getStateTimer() == 1) {
            no_change_counter++;
        }

        agent.getStats().setTimeSinceLastPlan(0);

        if (frontierExpType.equals(SimulatorConfig.frontiertype.PeriodicReturn) && noReturnTimer > (int) (simConfig.PERIODIC_RETURN_PERIOD * ((double) (max_no_change_counter + 10) / 10))) {
            agent.setExploreState(Agent.ExplorationState.ReturnToBase);
        }

        if (frontierExpType.equals(SimulatorConfig.frontiertype.UtilReturn)) {
            double infoRatio = (double) agent.getStats().getCurrentBaseKnowledgeBelief()
                    / (double) (agent.getStats().getCurrentBaseKnowledgeBelief() + agent.getStats().getNewInfo());
            if (infoRatio < simConfig.TARGET_INFO_RATIO) {
                agent.setExploreState(Agent.ExplorationState.ReturnToBase);
            }
        }
        calculateFrontiers();

        //If no frontiers found, or reached exploration goal, return to ComStation
        if (((frontiers.isEmpty()) || no_change_counter > 20 || (agent.getStats().getPercentageKnown() >= SimConstants.TERRITORY_PERCENT_EXPLORED_GOAL))) {
            agent.setMissionComplete(true);
            agent.setExploreState(Agent.ExplorationState.ReturnToBase);
        } else {
            last_percentage_known = agent.getStats().getPercentageKnown();
        }

        FrontierUtility best;
        best = chooseFrontier(true);

        //If could not find frontier, try to disregard other agents when planning
        if (best == null) {
            best = chooseFrontier(false);
        } else {
            agent.setFrontier(best.getFrontier());
            if (this.recentEnvError) {
                agent.setExactPath(best.getPath(agent));
            } else {
                agent.setPath(best.getPath(agent));
            }
        }

        //If no frontier could be assigned, then go back to base.">
        if (best == null) {
            // mission complete
            agent.setMissionComplete(true);
            agent.setExploreState(Agent.ExplorationState.ReturnToBase);
        }

        //If overlapping another agent, take random step
        for (TeammateAgent teammate : agent.getAllTeammates().values()) {
            if (agent.getLocation().equals(teammate.getLocation())) {
                nextStep = RandomWalk.randomStep(agent, 4);
                agent.getStats().setTimeSinceLastPlan(0);
                return nextStep;
            }
        }

        // Note: Path to best frontier has already been set when calculating
        // utility, no need to recalculate
        // Check that we have a path, otherwise take random step
        if ((agent.getPath() == null)
                || !agent.getPath().isValid()) {
            nextStep = RandomWalk.randomStep(agent, 4);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setEnvError(false);
            return nextStep;
        }
        // If we reach this point, we have a path.  Remove the first point
        // since this is the robot itself.
        nextStep = agent.getPath().nextPoint();
        return nextStep;
    }

    /**
     * Filter frontiers to remove to small frontiers and frontiers not reachable from basestation
     * (don't know why). Unreachable frontiers well be saved in badFrontiers
     *
     * @param lastFrontier current goal-frontier to consider it in any case, otherwise agent might
     * oszillate
     * @param grid current agents occupancygrid to see if a frontier is still interresting because
     * next to unknown
     */
    private PriorityQueue<Frontier> frontiersOfInterest(PriorityQueue<Frontier> frontierlist, Frontier lastFrontier, OccupancyGrid grid) {
        PriorityQueue<Frontier> list = new PriorityQueue<>();

        int counter = 0;
        for (Frontier currFrontier : frontierlist) {
            // To avoid oscillation, add last frontier to list (just in case it
            // still is the best, but is not one of the closest)
            if (currFrontier == lastFrontier) {
                counter++;
                continue;
            }
            if (counter >= SimConstants.MAX_NUM_FRONTIERS) {
                continue;
            }
            if (currFrontier.getArea() >= SimConstants.MIN_FRONTIER_SIZE
                    && currFrontier.hasUnknownBoundary(grid)) {//TODO Unneccessary!?!?
                //ignore frontiers not reachable from base //TODO WHY??? Woudn't path from agent be better?
                /*Path pathToFrontier = agent.calculatePath(agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).getLocation(),
                        currFrontier.getCentre(), false);*/
                Path pathToFrontier = agent.calculatePath(agent.getLocation(), currFrontier.getCentre(), false, false);
                if (!pathToFrontier.found) {
                    agent.addBadFrontier(currFrontier);
                } else {
                    list.add(currFrontier);
                    counter++;
                }
                // no need to break here, as we still want to add last frontier
                // if we haven't iterated through it yet.
            }

        }

        return list;
    }

    /**
     * Calculates Euclidean distance from all known teammates and self to frontiers of interest
     */
    private PriorityQueue<FrontierUtility> initializeUtilities(PriorityQueue<Frontier> frontierlist, boolean considerOtherAgents) {
        PriorityQueue<FrontierUtility> utilities = new PriorityQueue<>();

        // For each frontier of interest
        for (Frontier frontier : frontierlist) {
            // Add own utilities
            utilities.add(new FrontierUtility(agent, frontier));
            // Add teammates' utilities
            if(considerOtherAgents){
                for (TeammateAgent teammate : agent.getAllTeammates().values()) {
                    if (teammate.isExplorer() && teammate.getTimeSinceLastComm() < SimConstants.REMEMBER_TEAMMATE_FRONTIER_PERIOD) {
                        utilities.add(new FrontierUtility(teammate, frontier));
                    }
                }
            }

        }

        utilities.forEach((FrontierUtility u) -> {
            u.getExactUtility(agent);
        });
        return utilities;
    }

    protected FrontierUtility chooseFrontier(boolean considerOtherAgents) {
        // Step 1:  Create list of frontiers of interest
        PriorityQueue<Frontier> frontiersOfInterest = frontiersOfInterest(frontiers, agent.getFrontier(), agent.getOccupancyGrid());
        // Step 2:  Create utility estimates
        PriorityQueue<FrontierUtility> utilities = initializeUtilities(frontiersOfInterest, considerOtherAgents);
        // Step 3
        FrontierUtility best;

        ArrayList<FrontierUtility> badUtilities = new ArrayList<>();
        Iterator<FrontierUtility> util_iter = utilities.iterator();
        while (util_iter.hasNext()) {
            best = util_iter.next();
            if (badUtilities.contains(best)) {
                util_iter.remove();
                continue;
            }

            // Check if this is the only remaining frontier would be a little optimization
            // it's not possible to plan a path to this frontier or we are already there, so eliminate it entirely
            if (best.getExactUtility(agent) < 0) {
                badUtilities.add(best);
                for (FrontierUtility u : utilities) {
                    if (u.getFrontier() == best.getFrontier()) {
                        badUtilities.add(u);
                    }
                }
                if (best.getAgent() == agent) {
                    agent.addBadFrontier(best.getFrontier()); //only add bad frontiers if they are 'ours'
                }
            } else if (utilities.isEmpty() || (best.getUtility() >= utilities.peek().getUtility())) {
                //as the utility estimate is an optimistic heuristic if the best.utility is better as the peek-utility... go and get it
                //if (!considerOtherAgents || best.getAgent() == agent) {
                if (best.getAgent() == agent) {
                    return best;
                } else {
                    // This robot is assigned to this frontier, so remove all remaining associated utilities (for the agent and for the frontier)
                    badUtilities.add(best);
                    for (FrontierUtility u : utilities) {
                        if (u.getAgent() == best.getAgent() || u.getFrontier() == best.getFrontier()) {
                            badUtilities.add(u);
                        }
                    }
                }
            }

        }
        utilities.removeAll(badUtilities);

        //Return best Frontier (utility) that is calculated for me
        util_iter = utilities.iterator();
        while (util_iter.hasNext()) {
            best = util_iter.next();
            if (best.getAgent() == agent) {
                return best;
            }
        }
        return null;
    }

    /**
     * Calculates the frontiers and stores them in the member 'frontiers'
     */
    protected void calculateFrontiers() {
        // If recalculating frontiers, must set old frontiers dirty for image rendering
        frontiers.stream().forEach((f) -> {
            agent.addDirtyCells(f.getPolygonOutline());
        });

        // 1. Find all Contours
        LinkedList<LinkedList<Point>> contours = ContourTracer.findAllContours(agent.getOccupancyGrid());
        //System.out.println(contours.size());
        //ContourTracer.mergeContours(contours);
        //System.out.println(contours.size());

        frontiers = new PriorityQueue();
        Frontier currFrontier;

        int contourCounter = 0;
        int contoursSmall = 0;
        int contoursBad = 0;

        //2. Make the Contours to Frontiers
        for (LinkedList<Point> currContour : contours) {
            currFrontier = new Frontier(agent.getX(), agent.getY(), currContour);
            //badFrontiers is a list the Agent keeps with Frontiers he already marked as unreachable
            if (!agent.isBadFrontier(currFrontier)) {
                // only consider frontiers that are big enough and reachable
                if (currFrontier.getArea() >= SimConstants.MIN_FRONTIER_SIZE) {
                    {
                        frontiers.add(currFrontier);
                        contourCounter++;
                    }
                } else {
                    contoursSmall++;
                }
            } else {
                contoursBad++;
            }
        }

        if (SimConstants.DEBUG_OUTPUT) {
            System.out.println("retained " + contourCounter + " of them, disregarded due to size " + contoursSmall
                    + ", disregarded as bad " + contoursBad + ".");
        }
    }

    /**
     * Returns the frontiers for display purpose only.
     *
     * @return list of frontiers
     */
    public PriorityQueue<Frontier> getFrontiers() {
        return frontiers;
    }

    private Point processRelay() {
        if (!relay_handling) {
            return agent.getPath().nextPoint();
        }
        if (simConfig.useComStations()) {
            LinkedList<TeammateAgent> relays = new LinkedList<>();
            for (TeammateAgent mate : agent.getAllTeammates().values()) {
                if (mate.isStationary() && mate.getState() == Agent.AgentState.RELAY) {
                    relays.add(mate);
                }
            }
            tmap.update(false);
            HashMap<Integer, TopologicalNode> topoNodes = agent.getTopologicalMap().getJTopologicalNodes(true);
            LinkedList<TopologicalNode> nodesWithRelay = new LinkedList<>();
            int baseNodeId = agent.getTopologicalMap().getTopologicalJArea(agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).getLocation());
            topoNodes.get(baseNodeId).calculateDeadEnd(null);
            nodesWithRelay.add(topoNodes.get(baseNodeId));

            PriorityQueue<Point> needlessRelays = checkForNeedlessRelays(topoNodes, nodesWithRelay, relays);
            if (!needlessRelays.isEmpty()) {
                agent.setPath(agent.calculatePath(needlessRelays.peek().getLocation(), recentEnvError));
                agent.setExploreState(Agent.ExplorationState.GoToRelay);
                return agent.stay();
            }

            for (TeammateAgent mate : agent.getAllTeammates().values()) {
                if (mate.isStationary() && mate.getState() == Agent.AgentState.RELAY && mate.getID() != SimConstants.BASE_STATION_TEAMMATE_ID) {
                    //Is a Relay
                    int nodeid = agent.getTopologicalMap().getTopologicalJArea(mate.getLocation());
                    nodesWithRelay.add(topoNodes.get(nodeid));

                }

            }
            if (!isNeedlessPlace(topoNodes, nodesWithRelay)) {
                switch (relayType) {
                    case Random:
                        if (!agent.comStations.isEmpty() && (Math.random() < simConfig.getComStationDropChance())) {
                            agent.setExploreState(Agent.ExplorationState.SettingRelay);
                            return agent.stay();
                        }

                        TeammateAgent relay = agent.findNearComStation(agent.getSpeed());
                        if (agent.comStations.size() < agent.getComStationLimit() && relay != null && Math.random() < simConfig.getComStationTakeChance()) {
                            agent.setExploreState(Agent.ExplorationState.TakingRelay);
                            return relay.getLocation();
                        }
                        break;
                    case KeyPoints:
                        if (!agent.comStations.isEmpty()) {
                            PriorityQueue<NearRVPoint> tempPoints = new PriorityQueue<>();
                            TeammateAgent base = agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID);
                            for (Point p : tmap.getJunctionPoints()) {
                                for (TeammateAgent mate : relays) {
                                    if (agent.getOccupancyGrid().directLinePossible(mate.getLocation(), p, false, false)) {
                                        tempPoints.add(new NearRVPoint(p.x, p.y, base.getLocation().distance(p)));
                                    }
                                }
                            }
                            if (tempPoints.isEmpty()) {
                                for (Point p : tmap.getKeyPoints()) {
                                    for (TeammateAgent mate : relays) {
                                        if (agent.getOccupancyGrid().directLinePossible(mate.getLocation(), p, false, false)) {
                                            tempPoints.add(new NearRVPoint(p.x, p.y, base.getLocation().distance(p)));
                                        }
                                    }
                                }
                            }

                            Iterator<NearRVPoint> keyP_iter = tempPoints.iterator();
                            while (keyP_iter.hasNext()) {
                                NearRVPoint keyP = keyP_iter.next();
                                TopologicalNode keyN = topoNodes.get(agent.getTopologicalMap().getTopologicalJArea(keyP));
                                //if (noRelay(keyP) && noNearRelay(keyP) && !keyN.calculateDeadEnd((LinkedList<TopologicalNode>) nodesWithRelay.clone())) {
                                if (noRelay(keyP) && noNearRelay(keyP) && !keyN.isDeadEnd()) {
                                    agent.setPath(agent.calculatePath(keyP, recentEnvError));
                                    agent.setExploreState(Agent.ExplorationState.GoToRelay);
                                    return agent.stay();
                                }
                            }
                        }
                        break;
                    case RangeBorder:
                        if (!agent.comStations.isEmpty()) {
                            boolean useful = false;
                            for (TeammateAgent mate : relays) {
                                if (mate.getDirectComLink() >= 5 && mate.getDirectComLink() < (agent.getSpeed() * 2.1)) {
                                    //Is at range-border
                                    useful = true;
                                } else if (mate.getDirectComLink() != 0) {
                                    //Is in strong comrange so don't drop! Stop iterating as this is a definit "not useful"
                                    useful = false;
                                    break;
                                }
                            }
                            if (useful) {
                                agent.setExploreState(Agent.ExplorationState.SettingRelay);
                                return agent.stay();
                            }
                        }
                        break;
                    case BufferRelay:
                    case None:
                    default:
                }
            }

        }

        return agent.getPath().nextPoint();
    }

    public Point takeStep_GoToRelay() {

        if (agent.getLocation().equals(agent.getCurrentGoal())) {
            //Setting state twice to get right previous state
            agent.setExploreState(agent.getPrevExploreState());
            if (noRelay(agent.getLocation())) {
                agent.setExploreState(RealAgent.ExplorationState.SettingRelay);
            } else {
                agent.setExploreState(RealAgent.ExplorationState.TakingRelay);
            }
            return agent.getLocation();
        }
        if (agent.getPath().isValid()) {
            return agent.getPath().nextPoint();
        } else {
            agent.setExploreState(agent.getPrevExploreState());
            return RandomWalk.randomStep(agent, 4);
        }
    }

    /**
     * Checks if current Location is needless for a relay, means it is in a dead end
     *
     * @param topoNodes
     * @param nodesWithRelay
     * @return true if location is in dead end, false otherwise
     */
    private boolean isNeedlessPlace(HashMap<Integer, TopologicalNode> topoNodes, LinkedList<TopologicalNode> nodesWithRelay) {
        TopologicalNode node = topoNodes.get(agent.getTopologicalMap().getTopologicalJArea(agent.getLocation()));
        if (node == null) {
            return true;
        }
        //check for dead ends with all nodesWithRelays as borders except the own node
        LinkedList<TopologicalNode> tempBorder = (LinkedList<TopologicalNode>) nodesWithRelay.clone();
        tempBorder.remove(node);
        return node.isDeadEnd();
    }

    private PriorityQueue<Point> checkForNeedlessRelays(HashMap<Integer, TopologicalNode> topoNodes, LinkedList<TopologicalNode> nodesWithRelay, LinkedList<TeammateAgent> relays) {
        PriorityQueue<Point> needless = new PriorityQueue<>();

        for (TeammateAgent mate : relays) {
            if (mate.getID() == SimConstants.BASE_STATION_TEAMMATE_ID) {
                continue;
            }
            if (!mate.hasBaseComLink()) {
                needless.add(new NearRVPoint(mate.getLocation().x, mate.getLocation().y, agent.getLocation().distance(mate.getLocation()) * -1));
                continue;
            }
            TopologicalNode node = topoNodes.get(agent.getTopologicalMap().getTopologicalJArea(mate.getLocation()));
            //check for dead ends with all nodesWithRelays as borders except the own node
            LinkedList<TopologicalNode> tempBorder = (LinkedList<TopologicalNode>) nodesWithRelay.clone();
            tempBorder.remove(node);
            //boolean deadEnd = node.calculateDeadEnd((LinkedList<TopologicalNode>) tempBorder.clone());
            boolean deadEnd = node.isDeadEnd();
            if (deadEnd) {
                needless.add(new NearRVPoint(mate.getLocation().x, mate.getLocation().y, agent.getLocation().distance(mate.getLocation()) * -1));
            }
        }
        return needless;
    }
}
