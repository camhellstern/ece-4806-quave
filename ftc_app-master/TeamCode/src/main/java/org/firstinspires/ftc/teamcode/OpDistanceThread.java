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

    public Handler sHandler;
    OpticalDistanceSensor odsSensor;
    private LinkedBlockingQueue<RoverMessage> mQueue;
    OpticalDistanceSensor mDist;
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
                RoverMessage msg = new RoverMessage();
                msg.msgType = type;
                if (type == MessageType.OpDistance1)
                    msg.opDistVoltage = mInput.getVoltage();
                else if (type == MessageType.Pixy1)
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
        //Looper.prepare();
    }
}
