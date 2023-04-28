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
package path;

import config.SimConstants;
import environment.Frontier;
import environment.OccupancyGrid;
import exploration.Fibonacci;

import java.awt.Point;
import java.util.*;

/**
 *
 * @author Victor
 */
public class TopologicalNode {

    private int ID;
    private Point position;
    private LinkedList<TopologicalNode> neighbours;
    private LinkedList<Path> neighbour_paths;
    private LinkedList<Point> cells; //occupancy grid cells allocated to this node
    private boolean deadEnd;

    private TopologicalNode parent;

    private List<Frontier> frontiers;

    private boolean skinned;
    public TopologicalNode(int ID, Point position) {
        this.ID = ID;
        this.position = (Point) position.clone();
        neighbours = new LinkedList<TopologicalNode>();
        neighbour_paths = new LinkedList<Path>();
        cells = new LinkedList<Point>();
        deadEnd = false;
        parent = null;
        frontiers = new ArrayList<>();
        skinned = false;
    }

    public TopologicalNode getParent(){
        return parent;
    }

    public void addFrontier(Frontier f){
        frontiers.add(f);
    }

    public List<Frontier> getFrontiers(){
        return frontiers;
    }

    public void clearNeighbours(){
        neighbours.clear();
    }

    public boolean wasSkinned(){
        return skinned;
    }

    private void setSkinned(){
        skinned = true;
    }

    public boolean isUseless(){
        if(!frontiers.isEmpty()){
            return false;
        }
        if(!neighbours.isEmpty()){
            skin();
        }
        return neighbours.isEmpty() && frontiers.isEmpty();
    }

    private boolean thinned;

    public boolean wasThinned(){
        return thinned;
    }

    public void removeNeighbour(TopologicalNode neighbour){
        neighbours.remove(neighbour);
    }

    public void thin(){
        if(getFrontiers().isEmpty() && parent != null){
            if(getListOfNeighbours().size() == 1){
                parent.addNeighbour(neighbours.get(0), null);
                neighbours.get(0).setParent(parent);
                parent.removeNeighbour(this);
                thinned = true;
            } else if(getListOfNeighbours().size() == 0){
                parent.removeNeighbour(this);
                thinned = true;
            }
        }
    }
    public void skin(){
        List<TopologicalNode> toRemove = new ArrayList<>();
        for(TopologicalNode n : neighbours){
            if(n.isUseless()){
                n.setSkinned();
                toRemove.add(n);
            }
        }
        neighbours.removeAll(toRemove);
    }
    public void makeDoublyLinked(){
        this.neighbours.forEach(n -> n.addNeighbour(this, null));
    }

    int rank = -1;
    public int centroidRank(){
        if(rank != -1){
            return rank;
        }

        Queue<TopologicalNode> next = new LinkedList<>();
        Queue<TopologicalNode> curr = new LinkedList<>(this.neighbours);
        List<TopologicalNode> seen = new LinkedList<>();
        seen.add(this);

        int level = 1;
        int output = 0;
        while(!curr.isEmpty()){
            for(TopologicalNode node : curr){
                seen.add(node);

                output += Math.log(level);

                for(TopologicalNode n : node.neighbours){
                    if(!seen.contains(n)){
                        next.add(n);
                    }
                }
            }
            level++;
            curr.clear();
            curr.addAll(next);
            next.clear();
        }
        rank = output;
        return output;
    }

    public void setParent(TopologicalNode parent){
        this.parent = parent;
        neighbours.remove(parent);
    }

    public int getID() {
        return ID;
    }

    public void setID(int ID) {
        this.ID = ID;
    }

    public Point getPosition() {
        return position;
    }

    public boolean addNeighbour(TopologicalNode neighbour, Path path) {
        if(neighbour.equals(this)){
            return false;
        }
        if (!neighbours.contains(neighbour)) {
            neighbours.add(neighbour);
            if (ID != SimConstants.UNEXPLORED_NODE_ID && neighbour.getID() != SimConstants.UNEXPLORED_NODE_ID && path != null && (path.getStartPoint().distance(this.position) > 10 || !path.testPath(true))) {
                if (!path.getExact()) {
                    path.repairPath();
                }
                if (ID != SimConstants.UNEXPLORED_NODE_ID && neighbour.getID() != SimConstants.UNEXPLORED_NODE_ID && (path.getStartPoint().distance(this.position) > 10 || !path.testPath(true))) {
                    System.err.println("Added invalid path to node, already tried repair");
                    return false;
                }
            }
            neighbour_paths.add(path);
        }
        return true;
    }

    public LinkedList<TopologicalNode> getListOfNeighbours() {
        return neighbours;
    }

    public void addCell(Point p) {
        cells.add(p);
    }

    public int getDegree(){
        return neighbours.size();
    }

    public LinkedList<Point> getCellList() {
        return cells;
    }

    public boolean isDeadEnd() {
        return this.deadEnd;
    }

    /**
     * tests is this node is a dead and, means it has no border to unexplored environment
     *
     * @param border border for the search and list of visited nodes in inner algoritmic usage
     * @return true if this is the border itself or is a dead end considering the given border-nodes
     */
    public boolean calculateDeadEnd(LinkedHashSet<TopologicalNode> border) {
        if (border == null) {
            border = new LinkedHashSet<>();
        }
        border.add(this);
        if (this.getID() == SimConstants.UNEXPLORED_NODE_ID) {
            return false;
        }
        if (this.deadEnd) {
            return true;
        }
        LinkedList<TopologicalNode> pending = (LinkedList<TopologicalNode>) neighbours.clone();

        // iterate all neighbors
        while (!pending.isEmpty()) {
            TopologicalNode current = pending.pop();
            if (current.getID() == SimConstants.UNEXPLORED_NODE_ID) {
                return false;
            }
            if (border.contains(current)) {
                continue;
            }
            border.add(current); //Add this node to checked neighbors without dead end
            if (!current.calculateDeadEnd((LinkedHashSet<TopologicalNode>) border.clone())) {//check if neighbor is no dead end, too
                //If one of these return false there is a way to unexplored node and we need to stop it
                return false;
            }
        }
        //This is not a dead ent itself, no neighbor returned to be a dead end
        deadEnd = true;
        return true;
    }

    @Override
    public String toString() {
        return "TopoNode[" + this.ID + "] at Position " + this.position + " with " + this.neighbours.size() + " neighbors";
    }

    public Path getPathToNeighbour(TopologicalNode neighbour) {
        try {
            int index = neighbours.indexOf(neighbour);
            return neighbour_paths.get(index);
        } catch (IndexOutOfBoundsException ex) {
            return null;
        }
    }

    public double getLengthToNeighbour(TopologicalNode neighbour){
        try {
            int index = neighbours.indexOf(neighbour);
            return neighbour_paths.get(index).getLength();
        } catch (IndexOutOfBoundsException ex) {
            return Double.MAX_VALUE;
        }
    }

    public void removeImpossibleNeighbours(OccupancyGrid g) {
        for(int i = 0; i < neighbours.size(); i++){
            TopologicalNode n = neighbours.get(i);
            if(!g.locationExists(n.getPosition().x, n.getPosition().y)){
                neighbour_paths.remove(i);
                neighbours.remove(i);
                i--;
            }
        }
    }
}
