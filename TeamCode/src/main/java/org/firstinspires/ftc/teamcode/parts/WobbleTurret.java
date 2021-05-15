package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.parts.WobbleTurret.MovementState.*;
import static org.firstinspires.ftc.teamcode.parts.WobbleTurret.TurretState.*;

public class WobbleTurret {
    private DcMotorEx wobbleRotaryMotor, wobbleLinearMotor;
    private Servo claw, clawLift;
    private TouchSensor touchForwardRotary, touchBackwardRotary, touchOutLinear, touchInLinear;

    private TurretState turretState;
    private MovementState movementState;

    double clawClosed = 1;
    double clawOpen = 0;
    double armUp = 0;
    double armDown = 0;
    double armDrop = 0.5;
    double linearPower = 1;
    double linearPowerBack = linearPower * -1;
    double rotaryPower = 0.5;
    double rotaryPowerBack = rotaryPower * -1;

    ElapsedTime timer;

    public WobbleTurret(HardwareMap hMap, TurretState startingState) {
        wobbleRotaryMotor = hMap.get(DcMotorEx.class, "wobble_rotary");
        wobbleLinearMotor = hMap.get(DcMotorEx.class, "wobble_linear");

        claw = hMap.servo.get("claw");
        clawLift = hMap.servo.get("clawLift");

        touchForwardRotary = hMap.touchSensor.get("rotary_forward");
        touchBackwardRotary = hMap.touchSensor.get("rotary_back");
        touchOutLinear = hMap.touchSensor.get("linear_out");
        touchInLinear = hMap.touchSensor.get("linear_in");

        turretState = startingState;

        wobbleRotaryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();
    }

    public void init() {
        if(turretState == STOWED) {
            claw.setPosition(clawClosed);
            clawLift.setPosition(armUp);
        } else if(turretState == FORWARD_OPEN || turretState== BACK_OPEN) {
            claw.setPosition(clawOpen);
            clawLift.setPosition(armDown);
        } else if(turretState == FORWARD_CLOSED || turretState == BACK_CLOSED) {
            claw.setPosition(clawClosed);
            clawLift.setPosition(armDown);
        } else if(turretState == FORWARD_STOWED || turretState == BACK_STOWED) {
            claw.setPosition(clawClosed);
            clawLift.setPosition(armUp);
        }
    }

    public void advanceStateMachineTeleOp() {
        if(turretState == STOWED) {
            turretState = FORWARD_OPEN;
        } else if(turretState == FORWARD_OPEN) {
            turretState = BACK_DROP_POS;
        } else if(turretState == BACK_DROP_POS) {
            turretState = DROP_BACK;
        } else if(turretState == DROP_BACK) {
            turretState = FORWARD_OPEN;
        }

        beginAction(turretState);
    }

    public void setState(TurretState state) {
        turretState = state;

        beginAction(state);
    }

    public void update() {
        if(turretState == STOWED) {
            linearInCheck();
            rotaryForwardCheck();
        }else if(turretState == FORWARD_OPEN) {
            boolean linearCheck = linearOutCheck();
            rotaryForwardCheck();
            if(linearCheck) {
                clawLift.setPosition(armDown);
            }
        }else if(turretState == FORWARD_CLOSED) {
            boolean linearCheck = linearOutCheck();
            rotaryForwardCheck();
            if(linearCheck) {
                clawLift.setPosition(armDown);
            }
        }else if(turretState == FORWARD_DROP_POS) {
            linearInCheck();
            rotaryForwardCheck();
        }else if(turretState == BACK_STOWED) {
            if(timer.seconds() > 0.5) {
                tryLinearBackward();
                tryRotaryBackward();
                clawLift.setPosition(armUp);
            }
            linearInCheck();
            rotaryBackwardCheck();
        } else if(turretState == BACK_DROP_POS) {
            if (timer.seconds() > 0.5) {
                tryLinearBackward();
                tryRotaryBackward();
                clawLift.setPosition(armDrop);
            }
            linearInCheck();
            rotaryBackwardCheck();
        }else if(turretState == BACK_CLOSED) {
            boolean linearCheck = linearOutCheck();
            rotaryBackwardCheck();
            if(linearCheck) {
                clawLift.setPosition(armDown);
            }
        }
        else if(turretState == FORWARD_DROP_POS) {
            if (timer.seconds() > 0.5) {
                tryLinearBackward();
                tryRotaryForward();
                clawLift.setPosition(armDrop);
            }
            linearInCheck();
            rotaryForwardCheck();
        } else if(turretState == FORWARD_STOWED) {
            if(timer.seconds() > 0.5) {
                tryLinearBackward();
                tryRotaryForward();
                clawLift.setPosition(armUp);
            }
            linearInCheck();
            rotaryForwardCheck();
        }else if(turretState == BACK_OPEN) {
            boolean linearCheck = linearOutCheck();
            rotaryBackwardCheck();
            if(linearCheck) {
                clawLift.setPosition(armDown);
            }
        }
    }

    public void beginAction(TurretState state) {
        movementState = BUSY;
        if(state == STOWED) {
            tryRotaryForward();
            tryLinearBackward();
            claw.setPosition(clawClosed);
            clawLift.setPosition(armUp);
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
        }else if(state == FORWARD_DROP_POS) {
            claw.setPosition(clawClosed);
            timer.reset();
        } else if(state == BACK_OPEN) {
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
        } else if(state == BACK_DROP_POS) {
            claw.setPosition(clawClosed);
            timer.reset();
        } else if(state == DROP_BACK || state == DROP_FORWARD) {
            claw.setPosition(armDrop);
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

    public enum TurretState {
        STOWED, FORWARD_OPEN, FORWARD_CLOSED, FORWARD_STOWED, FORWARD_DROP_POS, DROP_FORWARD, BACK_OPEN, BACK_CLOSED, BACK_STOWED, BACK_DROP_POS, DROP_BACK
    }

    public enum MovementState {
        IDLE, BUSY
    }
}
