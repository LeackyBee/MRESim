package exploration;

import agents.Agent;
import agents.RealAgent;
import config.SimConstants;
import environment.ContourTracer;
import environment.Frontier;
import environment.OccupancyGrid;
import path.TopologicalNode;

import java.awt.*;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;

public class HungarianComms {

    private static HungarianComms singleton;

    private Point meetup;
    private RealAgent baseStation;

    private static int count;

    // We use this format instead of a hashmap linking agents to booleans/points so that we maintain an order
    // This is needed here for the hungarian assignment, so we can remember which agent relates to what point
    private final Map<RealAgent, Integer> agentToIndex;
    private final Map<Integer, RealAgent> indexToAgent;
    private final List<Boolean> agentAtRendezvous;
    private final List<Point> agentPoints;
    private static final FileOutputStream outputFile;

    static {
        try {
            outputFile = new FileOutputStream("/home/alec/Documents/Cambridge/Work/dissertation/Test Data/run3.txt", false);

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private List<RealAgent> agents = new ArrayList<>();

    public synchronized static HungarianComms register(RealAgent a){
        if(singleton == null){
            singleton = new HungarianComms();
            singleton.meetup = a.baseStation.getLocation();
            singleton.baseStation = a.baseStation;
        }
        singleton.agents.add(a);
        return singleton;
        /*
        * singleton.agentToIndex.put(a,count);
        singleton.indexToAgent.put(count, a);
        singleton.agentPoints.add(null);
        singleton.agentAtRendezvous.add(Boolean.FALSE);
        count++;
        * */
    }

    private HungarianComms(){
        count = 0;
        agentToIndex = new HashMap<>();
        indexToAgent = new HashMap<>();
        agentAtRendezvous = new ArrayList<>();
        agentPoints = new ArrayList<>();
    }

    public Point getMeetup(){
        if(!sorted){
            sortAgents();
        }
        return meetup;
    }

    public synchronized boolean atRendezvous(RealAgent a){
        if(!sorted){
            sortAgents();
        }
        agentAtRendezvous.set(agentToIndex.get(a), Boolean.TRUE);
        return allAtRendezvous();
    }

    public synchronized boolean allAtRendezvous(){
        if(!sorted){
            sortAgents();
        }
        return !agentAtRendezvous.contains(Boolean.FALSE);
    }

    public synchronized Point getNextTarget(RealAgent a){
        if(!sorted){
            sortAgents();
        }

        if(agentPoints.get(agentToIndex.get(a)) == null){
            assignContours();
        }

        Point target = agentPoints.get(agentToIndex.get(a));
        agentPoints.set(agentToIndex.get(a), null);
        boolean allNull = true;
        for (Point agentPoint : agentPoints) {
            if (agentPoint != null) {
                allNull = false;
                break;
            }
        }

        if(allNull){
            Collections.fill(agentAtRendezvous, Boolean.FALSE);
        }

        return target;
    }

    public static synchronized void writeToDebug(String s){
        try {
            outputFile.write(s.getBytes());
            outputFile.write("\n".getBytes());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private boolean sorted = false;

    private void sortAgents(){
        agents.stream().sorted(Comparator.comparingInt(Agent::getRobotNumber)).forEach(
                a ->{
                    singleton.agentToIndex.put(a,count);
                    singleton.indexToAgent.put(count, a);
                    singleton.agentPoints.add(null);
                    singleton.agentAtRendezvous.add(Boolean.FALSE);
                    count++;
                }
        );
        writeToDebug(agentToIndex.toString());
        sorted = true;
    }

    private void assignContours(){
        RealAgent aT = agents.get(0);
        double max = 0;
        for(RealAgent agent : agents){
            if(agent.getStats().getPercentageKnown() > max){
                max = agent.getStats().getPercentageKnown();
                aT = agent;
            }
        }

        final RealAgent a = aT;

        if(a.getStats().getPercentageKnown() > SimConstants.TERRITORY_PERCENT_EXPLORED_GOAL){
            agentPoints.replaceAll(p -> baseStation.getLocation());
            return;
        }


        LinkedList<LinkedList<Point>> contours = ContourTracer.findAllContours(a.getOccupancyGrid());
        ArrayList<Frontier> frontiers = new ArrayList<>();
        contours.forEach(c -> frontiers.add(new Frontier(a.getX(), a.getY(), c)));
        frontiers.removeIf(frontier ->frontier.getArea() <= SimConstants.MIN_FRONTIER_SIZE);
        frontiers.sort(Frontier::compareTo);

        writeToDebug(frontiers.toString());

        int assigned = 0;
        for(int i = 0; i < Math.min(agentPoints.size(), frontiers.size()); i++){
            agentPoints.set(i, frontiers.get(i).getCentre());
            assigned++;
        }

        writeToDebug(agentPoints.toString());

        /*
        The (no longer necessary) hungarian assignment algorithm

        List<RealAgent> agents = new ArrayList<>();
        for(int i = 0; i < count; i++){
            agents.add(indexToAgent.get(i));
        }

        while(agents.size() < frontiers.size()){
            frontiers.remove(frontiers.size()-1);
        }

        while(agents.size() > frontiers.size()){
            agents.remove(agents.size()-1);
        }

        // Now contours and agents are the same size, so we can do hungarian assignment

        int[][] matrix = new int[frontiers.size()][frontiers.size()];

        for(int i = 0; i < frontiers.size(); i++){
            for(int j = 0; j < agents.size(); j++){
                matrix[j][i] = -(frontiers.get(i).evaluateFrom(agents.get(j).getX(), agents.get(j).getY()));
            }
        }



        HungarianAlgorithm alg = new HungarianAlgorithm(matrix);
        int[][] assignment = alg.findOptimalAssignment();
        int assigned = 0;
        for(int[] pair : assignment){
            if(pair[0] < count){
                agentPoints.set(pair[0], frontiers.get(pair[1]).getCentre());
                assigned++;
            }
        }
         */



        List<TopologicalNode> nodes = a.getTopologicalMap().getTopologicalNodes(true).values()
                .stream().filter(node -> a.getOccupancyGrid().locationExists(node.getPosition().x, node.getPosition().y))
                .collect(Collectors.toList());

        writeToDebug("Topological Nodes:");
        for(TopologicalNode n : nodes){
            writeToDebug(n.toString());
        }

        for(TopologicalNode n : nodes){
            n.removeImpossibleNeighbours(a.getOccupancyGrid());
        }

        if(SimConstants.ALEC_DEBUG){
            System.out.println("Before Prims");
            for(TopologicalNode n1 : nodes){
                for(TopologicalNode n2 : n1.getListOfNeighbours()){
                    // output the graph in a format I can use to visualise in mermaid
                    System.out.println(" ".concat(String.valueOf(n1.getID())).concat(" --> ").concat(String.valueOf(n2.getID())));
                }
            }
        }

        // Now that we have the graph of critical points, we need to get a tree
        // the following is Prim's Algorithm, which finds the minimum spanning tree.

        double[][] graph = new double[nodes.size()][nodes.size()];
        for(int i = 0; i < graph.length; i++){
            for(int j = 0; j < nodes.size(); j++){
                graph[i][j] = nodes.get(i).getLengthToNeighbour(nodes.get(j));
            }
        }

        // We've got the info we want stored in graph, so we clear the neighbours to be able to directly
        // manipulate the nodes
        nodes.forEach(TopologicalNode::clearNeighbours);

        List<Integer> nodesInTree = new ArrayList<>(nodes.size());
        nodesInTree.add(new Random().nextInt(nodes.size())); // Start from the 1st node since the choice doesn't matter

        while(nodesInTree.size() < nodes.size()){
            double minEdge = Double.MAX_VALUE;
            int iMin = 0; // will always be a node in the tree
            int jMin = 0; // will always be the node we want to add

            // find the least edge that connects a node in the tree to a node not in the tree
            for(int i : nodesInTree){
                for(int j = 0; j < graph[i].length; j++){
                    if(graph[i][j] < minEdge && !nodesInTree.contains(j)){
                        minEdge = graph[i][j];
                        iMin = i;
                        jMin = j;
                    }
                }
            }

            nodesInTree.add(jMin);

            // Prims builds an undirected tree, we will add a hierarchy later
            nodes.get(iMin).addNeighbour(nodes.get(jMin), null);
            nodes.get(jMin).addNeighbour(nodes.get(iMin),null);
        }

        // Now, our topological nodes should form a tree

        if(SimConstants.ALEC_DEBUG){
            System.out.println("After Prims");
            for(TopologicalNode n1 : nodes){
                for(TopologicalNode n2 : n1.getListOfNeighbours()){
                    // output the graph in a format I can use to visualise in mermaid
                    System.out.println(" ".concat(String.valueOf(n1.getID())).concat(" --> ").concat(String.valueOf(n2.getID())));
                }
            }
        }

        boolean seen = false;
        TopologicalNode root = null;
        for (TopologicalNode node : nodes) {
            if (!seen || node.getPosition().distance(meetup) < root.getPosition().distance(meetup)) {
                seen = true;
                root = node;
            }
        }

        if(root != null){

            Queue<TopologicalNode> curr = new LinkedList<>();
            Queue<TopologicalNode> next = new LinkedList<>();

            curr.add(root);
            // We now have a tree
            while(!curr.isEmpty()){
                for(TopologicalNode c : curr){
                    List<TopologicalNode> neighbours = c.getListOfNeighbours();
                    for(TopologicalNode n : neighbours){
                        if(n.getParent() == null){
                            n.setParent(c);
                            next.add(n);
                        }
                    }
                }
                curr.clear();
                curr.addAll(next);
                next.clear();
            }

            if(SimConstants.ALEC_DEBUG){
                System.out.println("After tree-ification");
                for(TopologicalNode n1 : nodes){
                    for(TopologicalNode n2 : n1.getListOfNeighbours()){
                        // output the graph in a format I can use to visualise in mermaid
                        System.out.println(" ".concat(String.valueOf(n1.getID())).concat(" --> ").concat(String.valueOf(n2.getID())));
                    }
                }
            }

            // Now we should have imposed some hierarchy onto the Topological Nodes
            // We now assign frontiers to nodes based on distance
            for(Frontier f : frontiers.subList(0,assigned)){
                TopologicalNode min = root;
                double dist = root.getPosition().distance(f.getCentre());
                for(TopologicalNode node : nodes){
                    if(node.getPosition().distance(f.getCentre()) < dist){
                        min = node;
                        dist = node.getPosition().distance(f.getCentre());
                    }
                }
                min.addFrontier(f);
            }

            // After this all frontiers have been assigned to their closest topological node
            // By calling root.skin() we get rid of all nodes which don't lead to frontiers


            root.skin();
            nodes.removeIf(TopologicalNode::wasSkinned);

            // We may have that the root now only has one child and no frontiers
            // if this is the case we keep assigning the root as this child until we either have some frontiers or two children

            while(root.getDegree() == 1 && root.getFrontiers().size() == 0){
                nodes.remove(root);
                root = root.getListOfNeighbours().get(0);
                root.setParent(null);
            }

            if(SimConstants.ALEC_DEBUG){
                System.out.println("After Skinning");
                for(TopologicalNode n1 : nodes){
                    String name1 = String.valueOf(n1.getID()).concat(n1.getFrontiers().isEmpty() ? "" : "!");
                    for(TopologicalNode n2 : n1.getListOfNeighbours()){
                        String name2 = String.valueOf(n2.getID()).concat(n2.getFrontiers().isEmpty() ? "" : "!");
                        // output the graph in a format I can use to visualise in mermaid
                        System.out.println(" ".concat(name1).concat(" --> ").concat(name2));
                    }
                }
            }


            //nodes.forEach(TopologicalNode::thin);
            //nodes.removeIf(TopologicalNode::wasThinned);


            if(SimConstants.ALEC_DEBUG){
                System.out.println("After Thinning");
                for(TopologicalNode n1 : nodes){
                    String name1 = String.valueOf(n1.getID()).concat(n1.getFrontiers().isEmpty() ? "" : "!");
                    for(TopologicalNode n2 : n1.getListOfNeighbours()){
                        String name2 = String.valueOf(n2.getID()).concat(n2.getFrontiers().isEmpty() ? "" : "!");
                        // output the graph in a format I can use to visualise in mermaid
                        System.out.println(" ".concat(name1).concat(" --> ").concat(name2));
                    }
                }
            }

            // After this, we definitely have no straggling useless nodes
            // However, this is a singly linked tree, we would like it to be doubly linked
            nodes.forEach(TopologicalNode::makeDoublyLinked);

            if(SimConstants.ALEC_DEBUG){
                System.out.println("After Making Doubly Linked");
                for(TopologicalNode n1 : nodes){
                    String name1 = String.valueOf(n1.getID()).concat(n1.getFrontiers().isEmpty() ? "" : "!");
                    for(TopologicalNode n2 : n1.getListOfNeighbours()){
                        String name2 = String.valueOf(n2.getID()).concat(n2.getFrontiers().isEmpty() ? "" : "!");
                        // output the graph in a format I can use to visualise in mermaid
                        System.out.println(" ".concat(name1).concat(" --> ").concat(name2));
                    }
                }
            }

            // We choose the next meetup point as the node most central to this tree.
            //System.out.println(nodes);
            //System.out.println("Finding centroid");
            Optional<TopologicalNode> centroid = nodes.stream().min(
                    Comparator.comparingInt(TopologicalNode::centroidRank)
                    .thenComparing(tn -> -Math.hypot(tn.getPosition().x, tn.getPosition().y)));
            centroid.ifPresent(t -> meetup = t.getPosition());

            writeToDebug("New Meetup: ".concat(meetup.toString()));
        }

        while(assigned < count){
            agentPoints.set(assigned, meetup);
            assigned++;
        }

        // Ensure all agents possess the same map
        // This should be done by the simulator, but it's not.
        // This is necessary as sometimes we may select a meetup point that is outside
        // of another agents map due to the buggy sharing.
        OccupancyGrid occ = a.getOccupancyGrid();

        for (RealAgent agent : agentToIndex.keySet()) {
            agent.setOccupancyGrid(occ);
        }

    }

}
