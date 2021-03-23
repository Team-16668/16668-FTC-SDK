package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Jacob on 3/22/2021.
 */
public class BackgroundThread implements Runnable{
    //All the hardware objects that need to exist
    private DcMotor intake, wobbleArm;
    private DcMotorEx shooter;
    private Servo wobbleClaw, wobbleClaw2, backPlate, flicker, ringKnocker, wobbleLifter;
    private CRServo intakeServo;
    private TouchSensor wobbleTouch1, wobbleTouch2;
    private RevBlinkinLedDriver lights, lights2;

    //Thread run condition
    private boolean isRunning = true;

    private int sleepTime;

    /**
     * Constructor for the Background Thread
     * @param shooter A DcMotorEx object for the ring shooter
     * @param intake A DcMotor object for the intake
     * @param wobbleArm A DcMotor object for the wobble arm motor
     * @param wobbleClaw A Servo object for the first wobble claw
     * @param wobbleClaw2 A Servo object for the second wobble claw
     * @param backPlate A Servo object for the backplate of the RTM
     * @param flicker A Servo object for the flicker of the RTM
     * @param ringKnocker A Servo object for the 3D printed mechanism to knock over a stack of rings
     * @param wobbleLifter A Servo object for the secondary wobble lifter that is used in auto
     * @param intakeServo A CRServo object for the continuous rotation servo that runs when intaking rings
     * @param wobbleTouch1 A TouchSensor object that senses whether the wobble arm has hit the touch sensor yet
     * @param wobbleTouch2 A TouchSensor object that senses whether the wobble arm has hit the touch sensor yet
     * @param lights A RevBlinkinLedDriver for the first string of lights on the robot
     * @param lights2 A RevBlinkinLedDrive for the second string of lights on the robot
     * @param threadSleepDelay A parameter for the time that the thread should sleep between loops (in milliseconds)
     */
    public BackgroundThread(DcMotorEx shooter, DcMotor intake, DcMotor wobbleArm, Servo wobbleClaw,
                            Servo wobbleClaw2, Servo backPlate, Servo flicker, Servo ringKnocker, Servo wobbleLifter,
                            CRServo intakeServo, TouchSensor wobbleTouch1, TouchSensor wobbleTouch2,
                            RevBlinkinLedDriver lights, RevBlinkinLedDriver lights2, int threadSleepDelay){
        this.intake = intake;
        this.wobbleArm = wobbleArm;
        this.shooter = shooter;
        this.wobbleClaw = wobbleClaw;
        this.wobbleClaw2 = wobbleClaw2;
        this.backPlate = backPlate;
        this.flicker = flicker;
        this.ringKnocker = ringKnocker;
        this.wobbleLifter = wobbleLifter;
        this.intakeServo = intakeServo;
        this.wobbleTouch1 = wobbleTouch1;
        this.wobbleTouch2 = wobbleTouch2;
        this.lights = lights;
        this.lights2 = lights2;
        sleepTime = threadSleepDelay;
    }

    /**
     * This is the method that controls the active tasks (i.e. stopping the wobble arm once it's reached the ground, etc)
     */
    private void primaryTask(){
        //TODO: Create functions that can actually be used here
    }

    public void init() {
        //TODO: Set all motors, servos, sensors, lights, etc. to their proper modes/states
    }

    /**
     * Stops the position thread
     */
    public void stop(){ isRunning = false; }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            primaryTask();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
