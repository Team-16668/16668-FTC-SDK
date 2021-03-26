package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.concurrent.TimeUnit;

import static java.lang.Thread.sleep;

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

    //Variables for different background tasks
    boolean checkOnWobbleArm = false;
    public boolean flicking = false;
    double flickerStartTime = 0;
    double totalFlickerTime = 1;

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
        if(checkOnWobbleArm) {
            boolean condition1 = wobbleTouch2.isPressed() && wobbleArm.getPower() > 0;
            boolean condition2 = wobbleTouch1.isPressed() && wobbleArm.getPower() < 0;
            if(condition1 || condition2) {
                wobbleArm.setPower(0);
                checkOnWobbleArm = false;
            }
        }

        if(flicking) {
            if((System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1) >= totalFlickerTime / 2) {
                flicker.setPosition(1);
            }
            if((System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1) <= totalFlickerTime) {
                flicking = false;
            }
        }
    }

    //Discrete Functions I need while Not Moving
    public void Flick() {
        flicking = true;
        flicker.setPosition(0);
        flickerStartTime = System.nanoTime();
    }

    public void putWobbleArmDown() {
        wobbleArm.setPower(-0.45);
        checkOnWobbleArm = true;
    }

    public void putWobbleArmUp() {
        wobbleArm.setPower(0.37);
        checkOnWobbleArm = true;
    }

    public void setShooterRPM(double RPM) {
        shooter.setVelocity((RPM/60)*28);
    }

    public void runIntakeForward() {
        intake.setPower(1);
        intakeServo.setPower(1);
    }

    public void runIntakeBackward() {
        intake.setPower(-1);
        intakeServo.setPower(-1);
    }

    public void stopIntake() {
        intake.setPower(0);
        intakeServo.setPower(0);
    }

    public void liftBackplate() {
        backPlate.setPosition(0);
    }

    public void lowerBackplate() {
        backPlate.setPosition(1);
    }

    public void releaseWobble() {
        wobbleClaw.setPosition(0);
        wobbleClaw2.setPosition(0);
    }

    public void grabWobble() {
        wobbleClaw.setPosition(1);
        wobbleClaw2.setPosition(1);
    }

    public void putWobbleLifterDown() {
        wobbleLifter.setPosition(0.85);
    }

    public void liftWobble() {
        wobbleLifter.setPosition(0.75);
    }

    public void putWobbleLifterUp() {
        wobbleLifter.setPosition(0.66);
    }

    public void putDownRingKnocker() {
        ringKnocker.setPosition(1);
    }

    public void liftUpRingKnocker() {
        ringKnocker.setPosition(0);
    }

    public void setLightPattern(int lightSet, RevBlinkinLedDriver.BlinkinPattern pattern) {
        if(lightSet == 1) {
            lights.setPattern(pattern);
        } else if (lightSet == 2) {
            lights2.setPattern(pattern);
        }
    }

    public void init() {
        //Set motor directions
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set servo states
        wobbleClaw.setPosition(1);
        wobbleClaw2.setPosition(1);
        backPlate.setPosition(0);
        flicker.setPosition(1);

        ringKnocker.setPosition(0);
        wobbleLifter.setPosition(0.66);
        //wobbleLifter.setPosition(0.85);
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
                sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
