package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.parts.WobbleTurret.TurretState.*;

public class WobbleTurret {
    public DcMotorEx wobbleRotaryMotor, wobbleLinearMotor;
    public Servo claw;
    public CRServo clawLift;
    public TouchSensor touchForwardRotary, touchBackwardRotary, touchOutLinear, touchInLinear, touchUpLift, touchDownLift;

    public boolean firstTime = true;

    public boolean busy = false;

    public TurretState turretState;

    public double clawClosed = 0.5;
    public double clawOpen = 1;
    double dropTime = 0.3;
    double upPower = -0.5;
    double downPower = upPower * -1;
    double linearPower = 0.7;
    double linearPowerBack = linearPower * -1;
    double rotaryPower = 0.7;
    double rotaryPowerBack = rotaryPower * -1;

    public double saveTime;

    public ElapsedTime timer;

    public WobbleTurret(HardwareMap hMap, TurretState startingState, double rotarySpeed) {
        wobbleRotaryMotor = hMap.get(DcMotorEx.class, "wobble_rotary");
        wobbleLinearMotor = hMap.get(DcMotorEx.class, "wobble_linear");

        claw = hMap.servo.get("claw");
        clawLift = hMap.get(CRServo.class, "claw_lift");

        touchForwardRotary = hMap.touchSensor.get("rotary_forward");
        touchBackwardRotary = hMap.touchSensor.get("rotary_back");
        touchOutLinear = hMap.touchSensor.get("linear_out");
        touchInLinear = hMap.touchSensor.get("linear_in");
        touchUpLift = hMap.touchSensor.get("lift_up");
        touchDownLift = hMap.touchSensor.get("lift_down");

        turretState = startingState;

        wobbleRotaryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();

        this.rotaryPower = rotarySpeed;
        this.rotaryPowerBack = -rotarySpeed;
    }

    public WobbleTurret(HardwareMap hMap, TurretState startingState) {
        wobbleRotaryMotor = hMap.get(DcMotorEx.class, "wobble_rotary");
        wobbleLinearMotor = hMap.get(DcMotorEx.class, "wobble_linear");

        claw = hMap.servo.get("claw");
        clawLift = hMap.get(CRServo.class, "claw_lift");

        touchForwardRotary = hMap.touchSensor.get("rotary_forward");
        touchBackwardRotary = hMap.touchSensor.get("rotary_back");
        touchOutLinear = hMap.touchSensor.get("linear_out");
        touchInLinear = hMap.touchSensor.get("linear_in");
        touchUpLift = hMap.touchSensor.get("lift_up");
        touchDownLift = hMap.touchSensor.get("lift_down");

        turretState = startingState;

        wobbleRotaryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();

        this.rotaryPower = 0.7;
    }

    public void init() {
        if(turretState == STOWED) {
            claw.setPosition(clawClosed);
        } else if(turretState == FORWARD_OPEN || turretState== BACK_OPEN) {
            claw.setPosition(clawOpen);
        } else if(turretState == FORWARD_CLOSED || turretState == BACK_CLOSED) {
            claw.setPosition(clawClosed);
        } else if(turretState == FORWARD_STOWED || turretState == BACK_STOWED) {
            claw.setPosition(clawClosed);
        }
    }

    public void advanceStateMachineTeleOp() {
        if(turretState == STOWED) {
            turretState = FORWARD_OPEN;
        } else if(turretState == FORWARD_OPEN) {
            turretState = BACK_CLOSED;
        } else if(turretState == BACK_CLOSED) {
            turretState = DROP_BACK;
        } else if(turretState == DROP_BACK) {
            turretState = FORWARD_OPEN;
        }

        stopAll();

        beginAction(turretState);
    }

    public void setState(TurretState state) {
        turretState = state;

        stopAll();

        beginAction(state);
    }

    public void update() {
        if(turretState == STOWED) {
            linearInCheck();
            rotaryForwardCheck();
            liftUpCheck();
            if(timer.seconds() > 0.5) {
                tryLinearBackward();
            }
        }else if(turretState == FORWARD_OPEN) {
            boolean linearCheck = linearOutCheck();
            rotaryForwardCheck();
            if(linearCheck && firstTime) {
                tryLiftDown();
                firstTime = false;
            }
            liftDownCheck();
        }else if(turretState == FORWARD_CLOSED) {
            boolean linearCheck = linearOutCheck();
            rotaryForwardCheck();
            if(linearCheck) {
                tryLiftDown();
            }
            liftDownCheck();
            if(timer.seconds() > saveTime + 2) {
                clawLift.setPower(0);
                busy = false;
            }
        }else if(turretState == BACK_STOWED) {
            if(timer.seconds() > 0.5 && firstTime) {
                tryLinearBackward();
                tryRotaryBackward();
                tryLiftUp();
                firstTime = false;
            }
            linearInCheck();
            rotaryBackwardCheck();
            liftUpCheck();
        }else if(turretState == BACK_CLOSED) {
            if(timer.seconds() > 0.5 && firstTime) {
                boolean linearCheck = linearOutCheck();
                tryLiftUp();
                tryLinearForward();
                tryRotaryBackward();
                if(linearCheck) {
                    tryLiftUp();
                }
                firstTime = false;
            }
            liftUpCheck();
            rotaryBackwardCheck();
        } else if(turretState == FORWARD_STOWED) {
            if(timer.seconds() > 0.5) {
                tryLinearBackward();
                tryRotaryForward();
                tryLiftUp();
            }
            linearInCheck();
            rotaryForwardCheck();
            liftUpCheck();
        }else if(turretState == BACK_OPEN) {
            boolean linearCheck = linearOutCheck();
            rotaryBackwardCheck();
            if(linearCheck) {
                tryLiftDown();
            }
            liftDownCheck();
        }else if(turretState == DROP_BACK || turretState == DROP_FORWARD) {
            if(timer.seconds() > dropTime) {
                clawLift.setPower(0);
                claw.setPosition(clawOpen);
            }
        }

        if(wobbleLinearMotor.getPower() == 0 && wobbleRotaryMotor.getPower() == 0) {
            busy = false;
        }
    }

    public void beginAction(TurretState state) {
        firstTime = true;
        busy = true;
        if(state == STOWED) {
            tryRotaryForward();
            //tryLinearBackward();
            claw.setPosition(clawClosed);
            tryLiftUp();
            timer.reset();
        } else if(state == FORWARD_OPEN) {
            tryRotaryForward();
            tryLinearForward();
            claw.setPosition(clawOpen);
        } else if(state == FORWARD_CLOSED) {
            tryRotaryForward();
            tryLinearForward();
            claw.setPosition(clawClosed);
            saveTime = timer.seconds();
        }else if(state == FORWARD_STOWED) {
            claw.setPosition(clawClosed);
            timer.reset();
        }else if(state == BACK_OPEN) {
            tryRotaryBackward();
            tryLinearForward();
            claw.setPosition(clawOpen);
        } else if(state == BACK_CLOSED) {
            claw.setPosition(clawClosed);
            timer.reset();
        } else if(state == BACK_STOWED) {
            claw.setPosition(clawClosed);
            timer.reset();
        } else if(state == DROP_BACK || state == DROP_FORWARD) {
            tryLiftDown();
            timer.reset();
        }
    }

    public void tryRotaryForward() {
        if(!touchForwardRotary.isPressed()) {
            wobbleRotaryMotor.setPower(this.rotaryPower);
        }
    }

    public void tryRotaryBackward() {
        if(!touchBackwardRotary.isPressed()) {
            wobbleRotaryMotor.setPower(this.rotaryPowerBack);
        }
    }

    public void tryLinearForward() {
        if(!touchOutLinear.isPressed()) {
            wobbleLinearMotor.setPower(linearPower);
        }
    }

    public void tryLinearBackward() {
        if(!touchInLinear.isPressed()) {
            wobbleLinearMotor.setPower(linearPowerBack);
        }
    }

    public boolean rotaryForwardCheck() {
        if(touchForwardRotary.isPressed() && wobbleRotaryMotor.getPower() == this.rotaryPower) {
            wobbleRotaryMotor.setPower(0);
            return true;
        }else if(touchForwardRotary.isPressed()) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean rotaryBackwardCheck() {
        if(touchBackwardRotary.isPressed() && wobbleRotaryMotor.getPower() == this.rotaryPowerBack) {
            wobbleRotaryMotor.setPower(0);
            return true;
        }else if(touchBackwardRotary.isPressed()) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean linearOutCheck() {
        if(touchOutLinear.isPressed() && wobbleLinearMotor.getPower() == linearPower) {
            wobbleLinearMotor.setPower(0);
            return true;
        }
        else if(touchOutLinear.isPressed()) {
            return true;
        }
        else{
            return false;
        }
    }

    public boolean linearInCheck() {
        if(touchInLinear.isPressed() && wobbleLinearMotor.getPower()== linearPowerBack) {
            wobbleLinearMotor.setPower(0);
            return true;
        }
        else if(touchInLinear.isPressed()) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean liftUpCheck() {
        if(touchUpLift.isPressed()) {
            clawLift.setPower(0);
            return true;
        }else {
            return false;
        }
    }

    public boolean liftDownCheck() {
        if(touchDownLift.isPressed()) {
            clawLift.setPower(0);
            return true;
        }else {
            return false;
        }
    }

    public void tryLiftUp() {
        if(!touchUpLift.isPressed()) {
            clawLift.setPower(upPower);
        }
    }

    public void tryLiftDown() {
        if(!touchDownLift.isPressed()) {
            clawLift.setPower(downPower);
        }
    }

    public void stopAll() {
        wobbleLinearMotor.setPower(0);
        wobbleRotaryMotor.setPower(0);
        clawLift.setPower(0);
    }

    public TurretState getState() {
        return turretState;
    }


    public enum TurretState {
        STOWED, FORWARD_OPEN, FORWARD_CLOSED, FORWARD_STOWED, DROP_FORWARD, BACK_OPEN, BACK_CLOSED, BACK_STOWED, DROP_BACK
    }
}
