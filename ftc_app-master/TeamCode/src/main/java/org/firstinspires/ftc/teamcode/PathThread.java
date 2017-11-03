package org.firstinspires.ftc.teamcode;

import java.util.concurrent.LinkedBlockingQueue;

/**
 * Created by Cameron on 10/30/2017.
 */

public class PathThread extends Thread {
    public LinkedBlockingQueue<RoverMessage> pathQueue = new LinkedBlockingQueue<RoverMessage>();
    private LinkedBlockingQueue mainQueue;
    private Node currPos = new Node();
    AStarAlgorithm astar = new AStarAlgorithm();

    public PathThread(LinkedBlockingQueue<RoverMessage> mQueue){
        astar.initialize();
        mainQueue = mQueue;
    }

    @Override
    public void run()
    {
        while(true) {
            RoverMessage msg = new RoverMessage();
            try {
                msg = pathQueue.take();
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            switch(msg.msgType){
                case PathRequest:
                    priorityQueue p;
                    p = astar.AStar(astar.getCell(0, 0), astar.getCell(0, 2));
                    Movement movePath;
                    movePath = astar.movementPath(p, 0.0);
                    RoverMessage message = new RoverMessage();
                    message.msgType = MessageType.MovementPath;
                    message.move = movePath;
                    try{
                        mainQueue.put(message);
                        Thread.sleep(1000);
                    } catch (InterruptedException e){
                        e.printStackTrace();
                    }
                    break;
            }
        }

    }
}
