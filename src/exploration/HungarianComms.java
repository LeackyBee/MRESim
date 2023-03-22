package exploration;

import agents.RealAgent;
import environment.ContourTracer;
import environment.Frontier;
import path.TopologicalNode;

import java.awt.*;
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

    public synchronized static HungarianComms register(RealAgent a){
        if(singleton == null){
            singleton = new HungarianComms();
            singleton.meetup = a.baseStation.getLocation();
            singleton.baseStation = a.baseStation;
        }
        singleton.agentToIndex.put(a,count);
        singleton.indexToAgent.put(count, a);
        singleton.agentPoints.add(null);
        singleton.agentAtRendezvous.add(Boolean.FALSE);
        count++;
        return singleton;
    }

    private HungarianComms(){
        count = 0;
        agentToIndex = new HashMap<>();
        indexToAgent = new HashMap<>();
        agentAtRendezvous = new ArrayList<>();
        agentPoints = new ArrayList<>();
    }

    public Point getMeetup(){
        return meetup;
    }

    public synchronized boolean atRendezvous(RealAgent a){
        agentAtRendezvous.set(agentToIndex.get(a), Boolean.TRUE);
        return allAtRendezvous();
    }

    public synchronized boolean allAtRendezvous(){
        return !agentAtRendezvous.contains(Boolean.FALSE);
    }

    public synchronized Point getNextTarget(RealAgent a){
        if(agentPoints.get(agentToIndex.get(a)) == null){
            assignContours(a);
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

    private void assignContours(RealAgent a){

        LinkedList<LinkedList<Point>> contours = ContourTracer.findAllContours(a.getOccupancyGrid());
        ArrayList<Frontier> frontiers = new ArrayList<>();
        contours.forEach(c -> frontiers.add(new Frontier(a.getX(), a.getY(), c)));
        frontiers.sort(Frontier::compareTo);

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

        Collection<TopologicalNode> nodes = a.getTopologicalMap().getTopologicalNodes(true).values()
                .stream().filter(node -> a.getOccupancyGrid().locationExists(node.getPosition().x, node.getPosition().y))
                .collect(Collectors.toList());

        Optional<TopologicalNode> newMeet = nodes.stream().max(Comparator.comparingInt(TopologicalNode::getDegree));
        newMeet.ifPresent(topologicalNode -> meetup = topologicalNode.getPosition());

        while(assigned < count){
            agentPoints.set(assigned, meetup);
            assigned++;
        }
    }

}
