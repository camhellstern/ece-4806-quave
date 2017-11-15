package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


import android.os.Looper;
import android.os.Handler;
import android.os.Message;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
/**
 * Created by Cameron on 10/26/2017.
 */
public class OpDistanceThread extends Thread {

    LinkedBlockingQueue<RoverMessage> mQueue;
    LinkedBlockingQueue<RoverMessage> myQueue = new LinkedBlockingQueue<RoverMessage>();
    AnalogInput mInput;
    MessageType type;
    public OpDistanceThread(LinkedBlockingQueue<RoverMessage> mainQueue, AnalogInput op, MessageType t){
        mQueue = mainQueue;
        //mDist = op;
        mInput = op;
        type = t;
    }

    @Override
    public void run()
    {
        //odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "sensor_ods");
        while(true) {
            if(type == MessageType.Pixy1) {
                double voltage = mInput.getVoltage();
                if((voltage > 1.64) && (voltage < 1.66)) {
                    RoverMessage msg = new RoverMessage();
                    msg.msgType = MessageType.PixyAligned;
                    msg.PixyCamVoltage = mInput.getVoltage();
                    Message message = Message.obtain();
                    message.obj = msg;
                    try {
                        mQueue.put(msg);
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
            else if(type == MessageType.OpDistance1)
            {
                RoverMessage receive = new RoverMessage();
                try{
                    receive = myQueue.take();
                    sleep(500);
                } catch (InterruptedException e){
                    e.printStackTrace();
                }
                if(receive.msgType == MessageType.DataRequest)
                {
                    RoverMessage n = new RoverMessage();
                    n.opDistVoltage = mInput.getVoltage();
                    n.msgType = MessageType.OpDistance1;
                    Message message = Message.obtain();
                    message.obj = n;
                    try {
                        mQueue.put(n);
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        //Looper.prepare();
    }
}
