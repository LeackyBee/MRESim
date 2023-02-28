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

package simulator;

import agents.Agent;
import agents.ComStation;
import agents.RealAgent;
import agents.TeammateAgent;
import communication.DataMessage;
import communication.DirectLine;
import communication.PropModel1;
import communication.StaticCircle;
import config.RobotConfig;
import config.RobotTeamConfig;
import config.SimConstants;
import config.SimulatorConfig;
import environment.Environment;
import environment.Environment.Status;
import environment.OccupancyGrid;
import gui.MainGUI;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.*;
import java.util.*;
import javax.swing.ImageIcon;
import javax.swing.Timer;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 *
 * @author Julian de Hoog
 */
public class SimulationFramework implements ActionListener {

    boolean pauseSimulation;                   // For stepping through simulation one step at a time

    boolean isBatch;                            // Are we running a batch file
    int runNumber;
    int runNumMax;

    MainGUI mainGUI;                            // Allows simulator to change image, data
    ExplorationImage image;                     // Image of environment
    private Environment env;                     // The environment (walls, obstacles)
    RealAgent[] agent;                           // The agents
    int numRobots;

    SimulatorConfig simConfig;

    Polygon[] agentRange;                       // For visualization of agents' comm ranges

    Timer timer;                                // Drives simulation steps
    Random random;                              // For generating random debris

    int[] debrisTimer;                          // For aisleRoom random debris exercise (AAMAS2010)

    // Communication
    int[][] directCommTable;
    int[][] multihopCommTable;

    // Interesting data
    int timeElapsed;
    int jointAreaKnown;
    double pctAreaKnownTeam;
    int avgCycleTime;
    long simStartTime;
    int totalArea;
    double avgComStationKnowledge;
    double avgAgentKnowledge;
    double avgTimeLastCommand;
    double totalDistanceTraveled;
    int numSwaps;
    int experiment = 0;
    FileOutputStream outputFile;

    RobotTeamConfig robotTeamConfig;

    boolean logging_agent;
    boolean force_full_update;

    public SimulationFramework(MainGUI maingui, RobotTeamConfig newRobotTeamConfig,
            SimulatorConfig newSimConfig, ExplorationImage img) {
        random = new Random();
        mainGUI = maingui;
        image = img;
        simConfig = newSimConfig;
        env = simConfig.getEnvironment();
        robotTeamConfig = newRobotTeamConfig;

        logging_agent = false;
        force_full_update = false;

        reset();
    }

    private void reset() {
        pauseSimulation = false;
        env = simConfig.getEnvironment();

        timeElapsed = 0;
        jointAreaKnown = 1;             // to prevent divide by 0
        pctAreaKnownTeam = 0;
        totalArea = simConfig.getEnvironment().getTotalFreeSpace();
        avgComStationKnowledge = 0;
        avgAgentKnowledge = 0;
        avgTimeLastCommand = 0;
        avgCycleTime = 0;
        totalDistanceTraveled = 0;
        numSwaps = 0;

        createAgents(robotTeamConfig);

        // Initialize Timer
        timer = new Timer(0, this);
        timer.setInitialDelay(SimConstants.INIT_DELAY);
        timer.setCoalesce(true);

        // Initialize Debris timing
        debrisTimer = new int[6];
        for (int i = 0; i < 6; i++) {
            debrisTimer[i] = 0;
        }

        // Initialize Image
        updateImage(true);
    }

    private void createAgents(RobotTeamConfig robotTeamConfig) {
        numRobots = robotTeamConfig.getNumRobots();
        agent = new RealAgent[numRobots];
        TeammateAgent[] teammate = new TeammateAgent[numRobots];
        agentRange = new Polygon[numRobots];

        // Create BaseStation
        agent[0] = new ComStation(env.getColumns(), env.getRows(), robotTeamConfig.getRobotTeam().get(1), simConfig);
        agent[0].setState(Agent.AgentState.RELAY);
        teammate[0] = new TeammateAgent(robotTeamConfig.getRobotTeam().get(1));

        for (int i = 1; i < numRobots; i++) {
            if (!robotTeamConfig.getRobotTeam().get(i + 1).getRole().equals(RobotConfig.roletype.RelayStation)) {
                agent[i] = new RealAgent(env.getColumns(), env.getRows(), robotTeamConfig.getRobotTeam().get(i + 1), simConfig, agent[0]);
            } else {
                agent[i] = new ComStation(env.getColumns(), env.getRows(), robotTeamConfig.getRobotTeam().get(i + 1), simConfig);
                RealAgent realAgent = agent[agent[i].getParent() - 1];
                realAgent.addComStation((ComStation) agent[i]);
            }
            agentRange[i] = null;
            teammate[i] = new TeammateAgent(robotTeamConfig.getRobotTeam().get(i + 1));
            if (teammate[i].getRole() == RobotConfig.roletype.RelayStation) {
                teammate[i].setReference((ComStation) agent[i]);
            }
        }

        for (int i = 1; i < numRobots; i++) {
            agent[i].setSimFramework(this); //for logging only
        }

        // Give each agent its teammates
        for (int i = 0; i < numRobots; i++) {
            for (int j = 0; j < numRobots; j++) {
                if (j != i) {
                    agent[i].addTeammate(teammate[j].copy());
                }
            }
        }
    }

    public int getTotalArea() {
        return totalArea;
    }

    public void setForce_full_update(boolean force_full_update) {
        this.force_full_update = force_full_update;
    }

    public int getTimeElapsed() {
        return timeElapsed;
    }

    //used for checking if area has been double-sensed, for logging stats only
    public boolean hasCellBeenSensedByAnyAgent(int x, int y) {
        for (int i = 1; i < numRobots; i++) {
            if (agent[i].getOccupancyGrid().freeSpaceAt(x, y) || agent[i].getOccupancyGrid().obstacleAt(x, y)) {
                return true;
            }
        }
        return false;
    }

    public boolean simulationCycle() {
        // only run at start
        if (timeElapsed == 0) {
            try {
                outputFile = new FileOutputStream("/home/alec/Documents/Cambridge/Work/dissertation/Test Data/experiment.txt", false);


            } catch (IOException e) {
                throw new RuntimeException(e);
            }
            int goalArea = env.getTotalFreeSpace();
            // tells each agent how much of the map is actually "free space" so it can calculate how much it knows
            // in practice this wouldn't be possible, since to do this we need a map prior to running
            Arrays.stream(agent).forEach((a) -> a.getStats().setGoalArea(goalArea));
        } else if (timeElapsed == 1) {
            simStartTime = System.currentTimeMillis();
        }

        // Communications:

        // detects all 1-hop comm links and uses these to find all multi-hop links
        detectCommunication();

        for (int i = 0; i < numRobots - 1; i++) {
            for (int j = i + 1; j < numRobots; j++) {
                if (multihopCommTable[i][j] >= 1) {
                    // Teammate Agents are the object held by each robot to store information about the other agents
                    // If there is a multi-hop link between i and j, set their connection state to true
                    agent[i].getTeammate(agent[j].getID()).setCommunicationLink(true);
                    agent[j].getTeammate(agent[i].getID()).setCommunicationLink(true);
                }
            }
        }

        Arrays.stream(agent).forEach(RealAgent::flushComms);
        // Simulate map sharing
        simulateCommunication();


        // Move agents
        agentSteps();
        if(SimConstants.ALEC_DEBUG){
            System.out.println();
            System.out.println("Timestep: ".concat(String.valueOf(timeElapsed)));
        }
        agent[0].flushLog();

        // Update data
        if (timeElapsed % SimConstants.UPDATE_AGENT_KNOWLEDGE_INTERVAL == 0) {
            updateAgentKnowledgeData();
        }
        updateGlobalData();         // update data
        updateGUI();    // update GUI
        mainGUI.updateRobotConfig();


        // Logging
        logging();                  // perform logging as required

        robotTeamConfig.getRobotTeam().entrySet().stream().filter((entry) -> (entry.getValue().getLoggingState())).forEach((entry) -> {
            RealAgent a = agent[entry.getValue().getRobotNumber() - 1];
            a.getOccupancyGrid().saveToPNG(SimConstants.DEFAULT_IMAGE_LOG_DIRECTORY + "occuGrid " + a.toString() + timeElapsed + ".png");
            logging_agent = true; //There is a logging-wish
        });
        if (logging_agent) {  //do non-agent-based logging if there is a wish to log for any robot
            image.saveScreenshot(SimConstants.DEFAULT_IMAGE_LOG_DIRECTORY, timeElapsed);
        }
        logging_agent = false; //reset logging-wish for next cycle

        checkPause();               // check whether user wanted to pause
        avgCycleTime = (int) (System.currentTimeMillis() - simStartTime) / timeElapsed;

        // Alec logging code
        try {
            OccupancyGrid total = new OccupancyGrid(env.getColumns(), env.getRows());
            Arrays.stream(agent).forEach(a -> total.mergeGrid(a.getOccupancyGrid(),false));
            double totalKnown = 100*total.getNumFreeCells() / (double) totalArea;
            outputFile.write(String.valueOf(pctAreaKnownTeam)
                            .concat(" , ")
                            .concat(String.valueOf(totalKnown))
                            .concat("\n")
                            .getBytes());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return checkRunFinish(agent, timeElapsed, pctAreaKnownTeam, avgCycleTime);           // for scripting multiple runs, to max number of cycles
    }

    private String readFile(String pathname) throws IOException {
        File file = new File(pathname);
        StringBuilder fileContents = new StringBuilder((int) file.length());
        Scanner scanner = new Scanner(file);
        String lineSeparator = System.getProperty("line.separator");

        try {
            while (scanner.hasNextLine()) {
                fileContents.append(scanner.nextLine()).append(lineSeparator);
            }
            return fileContents.toString();
        } finally {
            scanner.close();
        }
    }

    private void updateRunConfig() {
        // This method can be used to set up batch simulations

        //open JSON file
        try {
            String json = readFile(simConfig.getBatchFilename());
            //parse JSON
            JSONObject obj = new JSONObject(json);
            //set global settings
            String baseDir = obj.getJSONObject("globalSettings").getString("baseDir");
            String envDir = baseDir + obj.getJSONObject("globalSettings").getString("envDir");
            //set runNumMax
            //set appropriate local settings
            JSONArray runs = obj.getJSONArray("runs");
            runNumMax = runs.length();

            int index = runNumber;
            String strategy = runs.getJSONObject(index).getString("strategy");
            switch (strategy) {
                case "greedy":
                    simConfig.setExpAlgorithm(SimulatorConfig.exptype.FrontierExploration);
                    simConfig.setFrontierAlgorithm(SimulatorConfig.frontiertype.ReturnWhenComplete);
                    break;
                case "ratio": {
                    String ratio = runs.getJSONObject(index).getString("ratio");
                    boolean baseRange = Boolean.parseBoolean(runs.getJSONObject(index).getString("baseRange"));
                    simConfig.setBaseRange(baseRange);
                    if (baseRange) {
                        String samplingDensity = runs.getJSONObject(index).getString("samplingDensity");
                        simConfig.setSamplingDensity(Double.parseDouble(samplingDensity));
                    }
                    simConfig.setExpAlgorithm(SimulatorConfig.exptype.FrontierExploration);
                    simConfig.setFrontierAlgorithm(SimulatorConfig.frontiertype.UtilReturn);
                    simConfig.TARGET_INFO_RATIO = Double.parseDouble(ratio);
                    break;
                }
                case "role-based": {
                    boolean multiPoint = Boolean.parseBoolean(runs.getJSONObject(index).getString("multiPoint"));
                    boolean baseRange = Boolean.parseBoolean(runs.getJSONObject(index).getString("baseRange"));
                    simConfig.setBaseRange(baseRange);
                    if (baseRange) {
                        String samplingDensity = runs.getJSONObject(index).getString("samplingDensity");
                        simConfig.setSamplingDensity(Double.parseDouble(samplingDensity));
                    }
                    String relayExplore = runs.getJSONObject(index).getString("relayExplore");
                    String rvcalc = runs.getJSONObject(index).getString("rvcalc");
                    simConfig.setExpAlgorithm(SimulatorConfig.exptype.RoleBasedExploration);
                    simConfig.setRoleSwitchAllowed(true);
                    simConfig.setReplanningAllowed(false);
                    simConfig.setStrictRoleSwitch(false);
                    simConfig.setUseImprovedRendezvous(rvcalc.equals("improved"));
                    simConfig.setRelayExplore(Boolean.parseBoolean(relayExplore));
                    if (multiPoint) {
                        String exploreReplan = runs.getJSONObject(index).getString("exploreReplan");
                        simConfig.setExploreReplan(Boolean.parseBoolean(exploreReplan));
                        String tryToGetToExplorerRV = runs.getJSONObject(index).getString("tryToGetToExplorerRV");
                        simConfig.setTryToGetToExplorerRV(Boolean.parseBoolean(tryToGetToExplorerRV));
                        String useSingleMeetingTime = runs.getJSONObject(index).getString("useSingleMeetingTime");
                        simConfig.setUseSingleMeetingTime(Boolean.parseBoolean(useSingleMeetingTime));
                    } else {

                    }
                    simConfig.setRVThroughWallsEnabled(multiPoint);
                    break;
                }
                default:
                    break;
            }

            String map = runs.getJSONObject(index).getString("map");
            String conf = runs.getJSONObject(index).getString("conf");
            String outputDir = baseDir + runs.getJSONObject(index).getString("outputDir");

            (new File(outputDir)).mkdirs();
            (new File(outputDir + "\\screenshots")).mkdirs();
            simConfig.setLogDataFilename(outputDir + "\\sim.txt");
            simConfig.setLogAgentsFilename(outputDir + "\\loc.txt");
            simConfig.setLogScreenshotsDirname(outputDir + "\\screenshots");
            simConfig.setLogAgents(true);
            simConfig.setLogData(true);
            simConfig.setLogScreenshots(true);

            simConfig.loadEnvironment(envDir + "\\" + map);
            robotTeamConfig.loadConfig(envDir + "\\" + conf);

            if (SimConstants.DEBUG_OUTPUT) {
                System.out.println(simConfig.toString());
            }
            try (PrintWriter out = new PrintWriter(outputDir + "\\config.txt")) {
                out.println(simConfig.toString());
            }

            mainGUI.updateFromRobotTeamConfig();
        } catch (IOException | JSONException | NumberFormatException e) {

        }

        /*String root = "C:\\Users\\Victor\\Documents\\uni\\actual_dphil_thesis\\experiments\\ch3\\automated\\";
        String configs[] = {//"grid_RB_2R", "grid_greedy_2R",
                            "lgrid_util_8r_0.2", "lgrid_util_8r_0.3",
                            "lgrid_util_8r_0.4", "lgrid_util_8r_0.5",
                            "lgrid_util_8r_0.6", "lgrid_util_8r_0.7",
                            "lgrid_util_8r_0.8", "lgrid_util_8r_0.9",};

        {
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.2 + runNumber*0.1;
            (new File(root + configs[runNumber])).mkdirs();
            (new File(root + configs[runNumber] + "\\screenshots")).mkdirs();
            simConfig.setLogDataFilename(root + configs[runNumber] + "\\sim.txt");
            simConfig.setLogAgentsFilename(root + configs[runNumber] + "\\loc.txt");
            simConfig.setLogScreenshotsDirname(root + configs[runNumber] + "\\screenshots");
        }*/
 /*int map = 2;
        if (runNumber == 1)
        {
            simConfig.loadEnvironment(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.RoleBasedExploration);
            simConfig.setRoleSwitchAllowed(true);
            simConfig.setReplanningAllowed(false);
            simConfig.setStrictRoleSwitch(false);
            simConfig.setUseImprovedRendezvous(true);
            robotTeamConfig.loadConfig(root + "configs\\" + configs[0]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[0] + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[0] + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[0])).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[0]);
        }


        mainGUI.updateFromRobotTeamConfig();*/
    }

    public void start() {
        runNumber = 0;
        //Check if we are running batch
        isBatch = simConfig.getExpAlgorithm() == SimulatorConfig.exptype.BatchRun;

        if (isBatch) {
            updateRunConfig(); //this should set runNumMax;
            reset();
        }
        timer.start();
        simStartTime = System.currentTimeMillis();
    }

    private void restart() {
        if (isBatch) {
            updateRunConfig();
        }
        reset();
        simStartTime = System.currentTimeMillis();
        timer.start();
    }

    public void takeOneStep() {
        pauseSimulation = true;
    }

    private void checkPause() {
        if (pauseSimulation || timeElapsed % 15000 == 0) {
            pauseSimulation = false;
            this.pause();
        }
    }

    private boolean baseStationDone() {
        // If the base station knows enough of the map
        return agent[0].isMissionComplete() || (((double) agent[0].getStats().getAreaKnown() / (double) totalArea) >= SimConstants.TERRITORY_PERCENT_EXPLORED_GOAL);
    }

    private boolean checkRunFinish(RealAgent[] agent, int timeElapsed, double pctAreaKnownTeam, int avgCycleTime) {
        boolean allAgentsAtBase = true;

        for (int i = 1; i < agent.length; i++) {
            if (!agent[i].getTeammate(1).hasCommunicationLink() || !agent[i].isMissionComplete()) {
                allAgentsAtBase = false;
            }
        }

        if (timeElapsed >= SimConstants.MAXIMUM_TIME || baseStationDone() || allAgentsAtBase) {
            updateGUI();
            timer.stop();
            runNumber++;
            if (isBatch && (runNumber < runNumMax)) {
                restart();
            } else {
                mainGUI.runComplete(agent, timeElapsed, pctAreaKnownTeam, avgCycleTime);
            }
        }
        return false;
    }

    public void pause() {
        timer.stop();
        if (SimConstants.DEBUG_OUTPUT) {
            System.out.println(this.toString() + "Pausing exploration!");
        }
    }

    public void kill() {
        timer.stop();
        if (SimConstants.DEBUG_OUTPUT) {
            System.out.println(this.toString() + "Resetting exploration!");
        }
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        simulationCycle();
    }

    private void agentSteps() {
        agent[0].flush();

        List<Thread> threads = new ArrayList<Thread>();
        for (RealAgent agent1 : agent) {
            // If the agent is a Comm Station it doesn't move
            if (agent1.getClass().toString().equals(ComStation.class.toString())) {
                continue;
            }
            Runnable task = new AgentStepRunnable(agent1, simConfig, timeElapsed, env, this);
            Thread worker = new Thread(task);
            worker.setName(agent1.toString());
            worker.start();
            threads.add(worker);
        }

        for (int i = 0; i < threads.size(); i++) {
            try {
                threads.get(i).join();
            } catch (Exception e) {
                System.err.println("Thread " + i + " threw exception " + e.getMessage());
            }
        }
    }

    /**
     * Simulates data from laser range finder
     *
     * @param agent the agend sensing something
     * @param nextLoc the location the given agent will be at the moment of sensing
     * @return
     */
    protected double[] findSensorData(RealAgent agent, Point nextLoc) {
        double currRayAngle, heading;
        int prevRayX, prevRayY;
        int currRayX, currRayY;
        double sensorData[] = new double[181];

        if (agent.getLocation().equals(nextLoc)) {
            heading = agent.getHeading();
        } else {
            heading = Math.atan2(nextLoc.y - agent.getY(), nextLoc.x - agent.getX());
        }

        //For every degree
        for (int i = 0; i <= 180; i += 1) {
            prevRayX = nextLoc.x;
            prevRayY = nextLoc.y;

            currRayAngle = heading - Math.PI / 2 + Math.PI / 180 * i;

            for (double m = 1; m <= agent.getSenseRange(); m++) {
                currRayX = nextLoc.x + (int) (m * Math.cos(currRayAngle));
                currRayY = nextLoc.y + (int) (m * Math.sin(currRayAngle));

                if (!env.locationExists(currRayX, currRayY)) {
                    sensorData[i] = nextLoc.distance(prevRayX, prevRayY);
                    break;
                } else if (env.statusAt(currRayX, currRayY).ordinal() >= Environment.Status.obstacle.ordinal()) {
                    sensorData[i] = nextLoc.distance(currRayX, currRayY);
                    break;
                } else if (m >= agent.getSenseRange()) {
                    sensorData[i] = nextLoc.distance(currRayX, currRayY);
                    break;
                } else {
                    prevRayX = currRayX;
                    prevRayY = currRayY;
                }
            }
        }

        return sensorData;
    }

    /**
     * update area known if needed
     *
     */
    private void updateAgentKnowledgeData() {
        if (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.RunFromLog) {
            return; //Nothing to do here
        }
        for (RealAgent agent1 : agent) {
            agent1.updateAreaKnown();
        }

    }

    private void simulateCommunication() {
        // Exchange data
        for (int i = 0; i < numRobots - 1; i++) {
            for (int j = i + 1; j < numRobots; j++) {
                if (multihopCommTable[i][j] >= 1) {
                    DataMessage msgFromFirst = new DataMessage(agent[i], directCommTable[i][j], multihopCommTable[0][i] > 0);
                    DataMessage msgFromSecond = new DataMessage(agent[j], directCommTable[j][i], multihopCommTable[j][0] > 0);

                    agent[i].receiveMessage(msgFromSecond);
                    agent[j].receiveMessage(msgFromFirst);
                }
            }
        }

        // Update post comm
        for (int q = 0; q < numRobots; q++) {
            agent[q].updateAfterCommunication();
        }
    }

    private void detectMultiHopLinks() {
        for (int i = 0; i < directCommTable.length; i++) {
            System.arraycopy(directCommTable[i], 0, multihopCommTable[i], 0, directCommTable.length);
        }
        for (int i = 0; i < multihopCommTable.length; i++) {
            for (int j = 0; j < multihopCommTable[0].length; j++) {
                if (multihopCommTable[i][j] >= 1 || multihopCommTable[j][i] >= 1) {
                    for (int k = 0; k < multihopCommTable.length; k++) {
                        if (multihopCommTable[k][i] >= 1 || multihopCommTable[i][k] >= 1) {
                            multihopCommTable[k][j] = 1;
                            multihopCommTable[j][k] = 1;
                        }
                    }
                }
            }
        }
    }

    private void detectCommunication() {
        directCommTable = new int[numRobots][numRobots];
        multihopCommTable = new int[numRobots][numRobots];

        switch (simConfig.getCommModel()) {
            case StaticCircle:
                directCommTable = StaticCircle.detectCommunication(env, agent);
                for (int i = 0; i < numRobots; i++) {
                    if (mainGUI.getRobotPanel(i).showCommRange()) {
                        agentRange[i] = null;
                    }
                }
                break;
            case DirectLine:
                directCommTable = DirectLine.detectCommunication(env, agent);
                for (int i = 0; i < numRobots; i++) {
                    if (mainGUI.getRobotPanel(i).showCommRange()) {
                        agentRange[i] = null;
                    }
                }
                break;
            case PropModel1:
                directCommTable = PropModel1.detectCommunication(env, agent);
                for (int i = 0; i < numRobots; i++) {
                    if (mainGUI.getRobotPanel(i).showCommRange()) {
                        agentRange[i] = PropModel1.getRange(env, agent[i]);
                    } else {
                        agentRange[i] = null;
                    }
                }
                break;
            default:
                break;
        }
        detectMultiHopLinks();
    }

    private void logging() {
        // Note, logging of data is performed in updateGlobalData, should change to here when i have the time

        // Log screenshot
        if (simConfig.logScreenshots()) {
            logScreenshot();
        }

        if (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.RunFromLog) {
            return; //Nothing to do here
        }
        // Log agent positions
        if (simConfig.logAgents()) {
            logAgents();
        }

    }

    private void logAgents() {
        try (PrintWriter outFile = new PrintWriter(new FileWriter(simConfig.getLogAgentFilename(), true))) {

            outFile.print(timeElapsed + " ");
            for (int i = 0; i < numRobots; i++) {
                outFile.print(agent[i].getX() + " ");
                outFile.print(agent[i].getY() + " ");
                outFile.print(agent[i].getCurrentGoal().getX() + " ");
                outFile.print(agent[i].getCurrentGoal().getY() + " ");
                outFile.print(agent[i].getRole() + " ");
                outFile.print(agent[i].getState() + " ");
                outFile.print(agent[i].totalSpareTime + " ");
            }
            outFile.println();
        } catch (IOException e) {
            System.err.println(this.toString() + "Agent logging - error writing data to file!" + e);
        }
    }

    private void logScreenshot() {
        image.fullUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        image.saveScreenshot(simConfig.getLogScreenshotsDirname(), timeElapsed);
    }

    public void logScreenshot(String dirname) {
        image.fullUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        image.saveScreenshot(dirname, timeElapsed);
    }

    public void simRateChanged(int newSimRate, MainGUI.runMode runmode) {
        if (newSimRate == 0) {
            timer.stop();
        } else {
            if (!timer.isRunning() && !runmode.equals(MainGUI.runMode.paused)) {
                timer.start();
            }
            //newSimRate 1-10, 10 is no delay
            timer.setDelay(10 * SimConstants.TIME_INCREMENT - newSimRate * SimConstants.TIME_INCREMENT);

        }
    }

    private void updateGUI() {
        mainGUI.updateFromData(agent, timeElapsed, pctAreaKnownTeam, avgCycleTime);
        updateImage(false); //was false

    }

    public void updateImage(boolean full) {

        if (full || this.force_full_update) {
            image.fullUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        } else {
            image.dirtyUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        }
        mainGUI.getLabelImageHolder().setIcon(new ImageIcon(image.getImage()));

    }

    public int getTrueJointAreaKnown() {
        int known = 0;
        for (int j = 0; j < env.getColumns(); j++) {
            for (int k = 0; k < env.getRows(); k++) {
                for (int i = 0; i < agent.length; i++) {
                    if ((agent[i].getOccupancyGrid().freeSpaceAt(j, k))
                            && (env.statusAt(j, k).ordinal() < Status.obstacle.ordinal())) {
                        known++; //"true" area known, excluding false empty spaces
                        i = agent.length;
                    }
                }
            }
        }
        return known;
    }

    private void updateGlobalData() {
        timeElapsed++;
        double pctAreaKnownBase = 100 * (double) agent[SimConstants.BASE_STATION_AGENT_ID].getStats().getAreaKnown() / (double) totalArea;
        pctAreaKnownTeam = pctAreaKnownBase;
        if (simConfig.logData()) {
            avgAgentKnowledge = 0;
            avgTimeLastCommand = 0;
            totalDistanceTraveled = 0;
            //jointAreaKnown = 1;

            int maxTeamLatency = 0;
            double avgTeamLatency = 0;
            //todo: same per robot

            //stats below also per agent
            int totalTeamTimeSpentSensing = 0; //sum of individual agent time spent sensing new areas
            int totalTeamTimeSpentDoubleSensing = 0; //sum of individual agent time spent re-sensing areas known by other agents
            int totalTeamTime = timeElapsed * (agent.length - 1); //ignore base station

            int totalRelayingTime = 0; //sum of individual agent time spent in "returning to parent" state.

            if ((timeElapsed % SimConstants.RECALC_JOINT_AREA) == 1) {
                jointAreaKnown = getTrueJointAreaKnown();
            } else {
                jointAreaKnown = Math.max(agent[SimConstants.BASE_STATION_AGENT_ID].getStats().getAreaKnown(), jointAreaKnown);
            }

            jointAreaKnown = Math.max(agent[SimConstants.BASE_STATION_AGENT_ID].getStats().getAreaKnown(), jointAreaKnown);

            for (int i = 1; i < agent.length; i++) {
                avgAgentKnowledge += agent[i].getStats().getAreaKnown();
                avgTimeLastCommand += agent[i].getStats().getTimeLastCentralCommand();
                totalDistanceTraveled += agent[i].getStats().getDistanceTraveled();
                if (agent[i].getStats().getMaxLatency() > maxTeamLatency) {
                    maxTeamLatency = agent[i].getStats().getMaxLatency();
                }
                avgTeamLatency += agent[i].getStats().getAvgLatency();
                totalTeamTimeSpentSensing += agent[i].getStats().getTimeSensing();
                totalTeamTimeSpentDoubleSensing += agent[i].getStats().getTimeDoubleSensing();
                totalRelayingTime += agent[i].getStats().getTimeReturning();
            }
            int totalNotSensingTime = totalTeamTime - totalTeamTimeSpentSensing - totalTeamTimeSpentDoubleSensing;
            avgTeamLatency /= (agent.length - 1);
            avgAgentKnowledge /= (agent.length - 1);  //ComStation not included in calculation
            avgAgentKnowledge = 100 * avgAgentKnowledge / jointAreaKnown;
            avgTimeLastCommand /= (agent.length - 1);

            if (jointAreaKnown == 0) {
                avgComStationKnowledge = 0;
            } else if (timeElapsed < 2) {
                avgComStationKnowledge = 100 * agent[0].getStats().getAreaKnown() / jointAreaKnown;
            } else {
                avgComStationKnowledge = ((timeElapsed - 1) * avgComStationKnowledge
                        + (100 * agent[0].getStats().getAreaKnown() / jointAreaKnown))
                        / timeElapsed;
            }
            pctAreaKnownTeam = 100 * (double) jointAreaKnown / (double) totalArea;

            try (PrintWriter outFile = new PrintWriter(new FileWriter(simConfig.getLogDataFilename(), true))) {

                outFile.print(timeElapsed + " ");
                outFile.print(System.currentTimeMillis() + " ");
                outFile.print(pctAreaKnownTeam + " ");
                outFile.print(pctAreaKnownBase + " ");
                outFile.print(totalArea + " ");
                outFile.print(avgAgentKnowledge + " ");
                outFile.print(avgTimeLastCommand + " ");
                outFile.print((double) totalDistanceTraveled / (double) (agent.length - 1) + " ");
                outFile.print(numSwaps + " ");
                outFile.print(jointAreaKnown + " ");
                outFile.print(agent[0].getStats().getAreaKnown() + " ");
                outFile.print(100 * (double) agent[0].getStats().getAreaKnown() / (double) jointAreaKnown + " ");
                outFile.print(maxTeamLatency + " ");
                outFile.print(avgTeamLatency + " ");
                outFile.print(totalTeamTimeSpentSensing + " ");
                outFile.print(totalTeamTimeSpentDoubleSensing + " ");
                outFile.print(totalRelayingTime + " ");
                outFile.print(totalNotSensingTime + " ");
                outFile.print(totalTeamTime + " ");
                for (int i = 1; i < agent.length; i++) {
                    outFile.print(agent[i].getStats().getAreaKnown() + " ");
                    outFile.print(agent[i].getStats().getNewInfo() + " ");
                    //outFile.print(agent[i].getMaxRateOfInfoGatheringBelief() + " ");
                    outFile.print(agent[i].getStats().getCurrentTotalKnowledgeBelief() + " ");
                    outFile.print(agent[i].getStats().getCurrentBaseKnowledgeBelief() + " ");
                    outFile.print(agent[i].getStats().getMaxLatency() + " ");
                    outFile.print(agent[i].getStats().getAvgLatency() + " ");
                    outFile.print(agent[i].getStats().getTimeSensing() + " ");
                    outFile.print(agent[i].getStats().getTimeDoubleSensing() + " ");
                    outFile.print(agent[i].getStats().getTimeReturning() + " ");
                    outFile.print((timeElapsed - agent[i].getStats().getTimeSensing()
                            - agent[i].getStats().getTimeDoubleSensing()) + " ");
                }
                outFile.println();
            } catch (IOException e) {
                System.err.println(this.toString() + "Error writing data to file!" + e);
            }
        }
    }

    @Override
    public String toString() {
        return ("[Simulator] ");
    }
}
