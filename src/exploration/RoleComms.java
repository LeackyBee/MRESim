package exploration;

import agents.RealAgent;

import java.awt.*;
import java.util.HashMap;
import java.util.Map;

public class RoleComms {

    private static final Map<Integer, RoleComms> commAgents = new HashMap<>();

    private Point rendezvous;
    private RealAgent explorer;
    private RealAgent relay;

    private RoleComms(){

    }

    public static synchronized RoleComms getCommunication(int explorerNumber){
        if(commAgents.containsKey(explorerNumber)){
            return commAgents.get(explorerNumber);
        } else{
            RoleComms comm = new RoleComms();
            commAgents.put(explorerNumber, comm);
            return comm;
        }
    }

    public synchronized void setRendezvous(Point suggested){
        rendezvous = suggested;
    }

    public synchronized void setExplorer(RealAgent explorer){
        this.explorer = explorer;
    }

    public RealAgent getExplorer() {
        return explorer;
    }

    public RealAgent getRelay() {
        return relay;
    }

    public void setRelay(RealAgent relay) {
        this.relay = relay;
    }

    public synchronized Point getRendezvous(){
        return rendezvous;
    }
}
