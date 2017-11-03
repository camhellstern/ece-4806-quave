/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.content.Intent;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.util.concurrent.LinkedBlockingQueue;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
enum RoverState{
    INIT, MOVE, IDLE, WAIT_FOR_SENSOR, PARKING
}

enum MotorState{
    STAND_STILL, MOVE
}

@Autonomous(name="Pushbot: Senior Design Project", group="Pushbot")
public class SDPOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    AStarAlgorithm          astar   = new AStarAlgorithm();
    priorityQueue           p       = new priorityQueue();
    public LinkedBlockingQueue<RoverMessage> mainQueue = new LinkedBlockingQueue<RoverMessage>();
    private ElapsedTime     runtime = new ElapsedTime();
    Node currenPosition = new Node();
    Movement curPath = new Movement();
    boolean begin = true;
    int pathCount = 0;
    int targetX, targetY;
    RoverState curState = RoverState.INIT;
    MotorState curMotor = MotorState.STAND_STILL;
    double curOp1 = 0;
    double curPixy1 = 0;
    boolean sent = false;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Path0",  "Starting at %7d :%7d", robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
        //telemetry.update();

        AnalogInput dist1 = hardwareMap.analogInput.get("dist1");

        telemetry.addData("%s\n", dist1.getDeviceName());

        AnalogInput pixy1 = hardwareMap.analogInput.get("pixy1");
        telemetry.addData("%s", pixy1.getDeviceName());

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        OpDistanceThread op1 = new OpDistanceThread(mainQueue, dist1, MessageType.OpDistance1);
        op1.start();

        OpDistanceThread pixycam1 = new OpDistanceThread(mainQueue, pixy1, MessageType.Pixy1);
        pixycam1.start();

        PathThread path1 = new PathThread(mainQueue);
        path1.start();

        sleep(1000);


        //telemetry.addData("Starting Thread", "OpDistance Thread");
        //telemetry.update();
        while(true)
        {
            RoverMessage msg = new RoverMessage();
            try {
                msg = mainQueue.take();
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            switch(msg.msgType){
                case OpDistance1:
                    telemetry.addData("Optical Distance Sensor Voltage", "%7f", msg.opDistVoltage);
                    telemetry.update();
                    curOp1 = msg.opDistVoltage;
                    //RoverMessage pathRequest = new RoverMessage();
                    //pathRequest.msgType = MessageType.PathRequest
                    //Intent intent = new Intent(AStarAlgorithm.class);
                    /*if(msg.opDistVoltage > 1.5) {
                        encoderDrive(TURN_SPEED, 6, -6, 5.0);
                        encoderDrive(DRIVE_SPEED, 12, 12, 5.0);
                    }*/
                    break;
                case Pixy1:
                    telemetry.addData("PixyCam Voltage", "%7f", msg.PixyCamVoltage);
                    telemetry.update();
                    curPixy1 = msg.PixyCamVoltage;
                    break;
                case MovementPath:
                    pathCount = msg.move.size;
                    curPath = msg.move;
                    /*for(int l = 0; l < msg.move.size; l++){
                        if(msg.move.moveP[l].movementType == MovementType.FORWARD)
                            encoderDrive(DRIVE_SPEED, msg.move.moveP[l].dist, msg.move.moveP[l].dist, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                        else if(msg.move.moveP[l].movementType == MovementType.TURN_LEFT)
                            encoderDrive(TURN_SPEED, -12, 12, 4.0);
                        else if(msg.move.moveP[l].movementType == MovementType.TURN_RIGHT)
                            encoderDrive(TURN_SPEED, 12, -12, 4.0);
                    }*/
                    break;
            }
            if(curState == RoverState.INIT)
            {
                RoverMessage pathRequest = new RoverMessage();
                pathRequest.msgType = MessageType.PathRequest;

                try {
                    path1.pathQueue.put(pathRequest);
                } catch (InterruptedException e){
                    e.printStackTrace();
                }
                curState = RoverState.IDLE;
            }

            if(pathCount != 0)
            {
                if(sent == false) {
                    if (curPath.moveP[0].movementType == MovementType.FORWARD) {
                        targetX = robot.leftDrive.getCurrentPosition() + (int) (curPath.moveP[0].dist * COUNTS_PER_INCH);
                        targetY = robot.rightDrive.getCurrentPosition() + (int) (curPath.moveP[0].dist * COUNTS_PER_INCH);
                    } else if (curPath.moveP[0].movementType == MovementType.TURN_LEFT) {
                        targetX = robot.leftDrive.getCurrentPosition() + (int) ((-11) * COUNTS_PER_INCH);
                        targetY = robot.rightDrive.getCurrentPosition() + (int) ((11) * COUNTS_PER_INCH);
                    } else if (curPath.moveP[0].movementType == MovementType.TURN_RIGHT) {
                        targetX = robot.leftDrive.getCurrentPosition() + (int) ((11) * COUNTS_PER_INCH);
                        targetY = robot.rightDrive.getCurrentPosition() + (int) ((-11) * COUNTS_PER_INCH);
                    }

                    robot.leftDrive.setTargetPosition(targetX);
                    robot.rightDrive.setTargetPosition(targetY);
                    sent = true;
                }


                encoderDrive(DRIVE_SPEED, targetX, targetY, 5.0);
            }
            else if(pathCount == 0)
            {
                curMotor = MotorState.STAND_STILL;
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sent = false;
            }

            if(curPixy1 > 1.63 && curPixy1 < 1.67 && curState == RoverState.IDLE)
            {
                pathCount = 0;
                curMotor = MotorState.STAND_STILL;
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                curState = RoverState.WAIT_FOR_SENSOR;
            }

            if(curState == RoverState.WAIT_FOR_SENSOR && curOp1 < 0.5)
            {
                Movement turnIn = new Movement();
                MovementNode first = new MovementNode();
                MovementNode second = new MovementNode();
                first.movementType = MovementType.TURN_RIGHT;
                turnIn.moveP[0] = first;
                second.movementType = MovementType.FORWARD;
                second.dist = 12;
                turnIn.moveP[1] = second;
                turnIn.size = 2;
                pathCount = 2;
                curPath = turnIn;
                sent = false;
                curState = RoverState.PARKING;
            }
        }
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /*telemetry.addData("Initializing","A*");
        telemetry.update();

        astar.initialize();
        telemetry.addData("A*","Initialized");
        telemetry.update();
        p = astar.AStar(astar.getCell(0, 0), astar.getCell(2,2));

        Movement movePath = new Movement();
        movePath = astar.movementPath(p, 0.0);

        for(int l = 0; l < movePath.size; l++){
            if(movePath.moveP[l].movementType == MovementType.FORWARD)
                encoderDrive(DRIVE_SPEED, movePath.moveP[l].dist, movePath.moveP[l].dist, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            else if(movePath.moveP[l].movementType == MovementType.TURN_LEFT)
                encoderDrive(TURN_SPEED, -12, 12, 4.0);
            else if(movePath.moveP[l].movementType == MovementType.TURN_RIGHT)
                encoderDrive(TURN_SPEED, 12, -12, 4.0);
        }*/
        //sleep(1000);
        /*if(p.elements[1].c.x == 1 && p.elements[1].c.y == 0) {
            encoderDrive(DRIVE_SPEED, 48, 48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

            sleep(1000);     // pause for servos to move

            telemetry.addData("A*", "Passed");
            telemetry.update();
        }
        else
        {
            encoderDrive(DRIVE_SPEED, 10, 10, 5.0);
            telemetry.addData("A*", "Failed");
            telemetry.update();
        }*/
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            switch(curMotor){
                case STAND_STILL:
                    robot.leftDrive.setPower(0);
                    robot.rightDrive.setPower(0);

                    // Turn off RUN_TO_POSITION
                    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if(robot.leftDrive.getCurrentPosition() != robot.leftDrive.getTargetPosition() || robot.rightDrive.getCurrentPosition() != robot.rightDrive.getTargetPosition())
                    {
                        curMotor = MotorState.MOVE;
                        runtime.reset();
                    }
                    break;
                case MOVE:
                    robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftDrive.setPower(Math.abs(speed));
                    robot.rightDrive.setPower(Math.abs(speed));
                    if(!robot.leftDrive.isBusy() || !robot.rightDrive.isBusy())
                    {
                        curMotor = MotorState.STAND_STILL;
                        for(int k=0;k<pathCount - 1;k++)
                        {
                            curPath.moveP[k] = curPath.moveP[k+1];
                        }
                        pathCount--;
                        sent = false;
                    }
                    break;
            }
            // Determine new target position, and pass to motor controller
            /*newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d", robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
                //telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

            //  sleep(250);   // optional pause after each move
        }
    }
}
