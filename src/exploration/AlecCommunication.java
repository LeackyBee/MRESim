package exploration;

import java.awt.*;
import java.util.HashMap;
import java.util.Map;

public class AlecCommunication {

    private static Map<Integer, AlecCommunication> commAgents = new HashMap<>();

    private Point rendezvous;
    private boolean rendezvousChanged = false;

    private AlecCommunication(){

    }

    public static synchronized AlecCommunication getCommunication(int explorerNumber){
        if(commAgents.containsKey(explorerNumber)){
            return commAgents.get(explorerNumber);
        } else{
            AlecCommunication comm = new AlecCommunication();
            commAgents.put(explorerNumber, comm);
            return comm;
        }
    }

    public synchronized void setRendezvous(Point suggested){
        rendezvousChanged = true;
        rendezvous = suggested;
    }

    public boolean isRendezvousChanged(){
        return rendezvousChanged;
    }

    public synchronized void ackRendezvous(){
        rendezvousChanged = false;
    }

    public synchronized Point getRendezvous(){
        return rendezvous;
    }
}
