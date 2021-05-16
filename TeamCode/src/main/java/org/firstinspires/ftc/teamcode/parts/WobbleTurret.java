package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.parts.WobbleTurret.MovementState.*;
import static org.firstinspires.ftc.teamcode.parts.WobbleTurret.TurretState.*;

public class WobbleTurret {
    public DcMotorEx wobbleRotaryMotor, wobbleLinearMotor;
    public Servo claw;
    public CRServo clawLift;
    public TouchSensor touchForwardRotary, touchBackwardRotary, touchOutLinear, touchInLinear, touchUpLift, touchDownLift;

    private TurretState turretState;
    private MovementState movementState;

    double clawClosed = 1;
    double clawOpen = 0;
    double dropTime = 0.5;
    double upPower = 1;
    double downPower = upPower * -1;
    double linearPower = 1;
    double linearPowerBack = linearPower * -1;
    double rotaryPower = 0.5;
    double rotaryPowerBack = rotaryPower * -1;

    ElapsedTime timer;

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
            turretState = BACK_STOWED;
        } else if(turretState == BACK_STOWED) {
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
        }else if(turretState == FORWARD_OPEN) {
            boolean linearCheck = linearOutCheck();
            rotaryForwardCheck();
            if(linearCheck) {
                clawLift.setPower(downPower);
            }
            liftDownCheck();
        }else if(turretState == FORWARD_CLOSED) {
            boolean linearCheck = linearOutCheck();
            rotaryForwardCheck();
            if(linearCheck) {
                clawLift.setPower(downPower);
            }
            liftDownCheck();
        }else if(turretState == BACK_STOWED) {
            if(timer.seconds() > 0.5) {
                tryLinearBackward();
                tryRotaryBackward();
                clawLift.setPower(upPower);
            }
            linearInCheck();
            rotaryBackwardCheck();
            liftUpCheck();
        }else if(turretState == BACK_CLOSED) {
            boolean linearCheck = linearOutCheck();
            rotaryBackwardCheck();
            if(linearCheck) {
                clawLift.setPower(downPower);
            }
            liftDownCheck();
        } else if(turretState == FORWARD_STOWED) {
            if(timer.seconds() > 0.5) {
                tryLinearBackward();
                tryRotaryForward();
                clawLift.setPower(upPower);
            }
            linearInCheck();
            rotaryForwardCheck();
            liftUpCheck();
        }else if(turretState == BACK_OPEN) {
            boolean linearCheck = linearOutCheck();
            rotaryBackwardCheck();
            if(linearCheck) {
                clawLift.setPower(downPower);
            }
            liftDownCheck();
        }else if(turretState == DROP_BACK || turretState == DROP_FORWARD) {
            if(timer.seconds() > dropTime) {
                clawLift.setPower(0);
                claw.setPosition(clawOpen);
            }
        }
    }

    public void beginAction(TurretState state) {
        movementState = BUSY;
        if(state == STOWED) {
            tryRotaryForward();
            tryLinearBackward();
            claw.setPosition(clawClosed);
            clawLift.setPower(upPower);
        } else if(state == FORWARD_OPEN) {
            tryRotaryForward();
            tryLinearForward();
            claw.setPosition(clawOpen);
        } else if(state == FORWARD_CLOSED) {
            tryRotaryForward();
            tryLinearForward();
            claw.setPosition(clawClosed);
        }else if(state == FORWARD_STOWED) {
            claw.setPosition(clawClosed);
            timer.reset();
        }else if(state == BACK_OPEN) {
            tryRotaryBackward();
            tryLinearForward();
            claw.setPosition(clawOpen);
        } else if(state == BACK_CLOSED) {
            tryRotaryBackward();
            tryLinearForward();
            claw.setPosition(clawClosed);
        } else if(state == BACK_STOWED) {
            claw.setPosition(clawClosed);
            timer.reset();
        } else if(state == DROP_BACK || state == DROP_FORWARD) {
            clawLift.setPower(downPower);
            timer.reset();
        }
    }

    private void tryRotaryForward() {
        if(!touchForwardRotary.isPressed()) {
            wobbleRotaryMotor.setPower(rotaryPower);
        }
    }

    private void tryRotaryBackward() {
        if(!touchBackwardRotary.isPressed()) {
            wobbleRotaryMotor.setPower(rotaryPowerBack);
        }
    }

    private void tryLinearForward() {
        if(!touchOutLinear.isPressed()) {
            wobbleLinearMotor.setPower(linearPower);
        }
    }

    private void tryLinearBackward() {
        if(!touchInLinear.isPressed()) {
            wobbleLinearMotor.setPower(linearPowerBack);
        }
    }

    private boolean rotaryForwardCheck() {
        if(touchForwardRotary.isPressed() && wobbleRotaryMotor.getPower() == rotaryPower) {
            wobbleRotaryMotor.setPower(0);
            return true;
        }
        else {
            return false;
        }
    }

    private boolean rotaryBackwardCheck() {
        if(touchBackwardRotary.isPressed() && wobbleRotaryMotor.getPower() == rotaryPowerBack) {
            wobbleRotaryMotor.setPower(0);
            return true;
        }
        else {
            return false;
        }
    }

    private boolean linearOutCheck() {
        if(touchOutLinear.isPressed() && wobbleLinearMotor.getPower() == linearPower) {
            wobbleLinearMotor.setPower(0);
            return true;
        }
        else {
            return false;
        }
    }

    private boolean linearInCheck() {
        if(touchInLinear.isPressed() && wobbleLinearMotor.getPower()== linearPowerBack) {
            wobbleLinearMotor.setPower(0);
            return true;
        }
        else {
            return false;
        }
    }

    private boolean liftUpCheck() {
        if(touchUpLift.isPressed() && clawLift.getPower() == upPower) {
            clawLift.setPower(0);
            return true;
        } else {
            return false;
        }
    }

    private boolean liftDownCheck() {
        if(touchDownLift.isPressed() && clawLift.getPower() == downPower) {
            clawLift.setPower(0);
            return true;
        } else {
            return false;
        }
    }

    private void stopAll() {
        wobbleLinearMotor.setPower(0);
        wobbleRotaryMotor.setPower(0);
        clawLift.setPower(0);
    }

    public enum TurretState {
        STOWED, FORWARD_OPEN, FORWARD_CLOSED, FORWARD_STOWED, DROP_FORWARD, BACK_OPEN, BACK_CLOSED, BACK_STOWED, DROP_BACK
    }

    public enum MovementState {
        IDLE, BUSY
    }
}
