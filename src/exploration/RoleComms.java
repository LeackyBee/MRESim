package exploration;

import java.awt.*;
import java.util.HashMap;
import java.util.Map;

public class RoleComms {

    private static final Map<Integer, RoleComms> commAgents = new HashMap<>();

    private Point rendezvous;

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


    public synchronized Point getRendezvous(){
        return rendezvous;
    }
}
