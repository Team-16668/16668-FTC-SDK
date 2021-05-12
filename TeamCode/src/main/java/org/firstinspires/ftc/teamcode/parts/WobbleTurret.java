package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static org.firstinspires.ftc.teamcode.parts.WobbleTurret.MovementState.*;
import static org.firstinspires.ftc.teamcode.parts.WobbleTurret.TurretState.*;

public class WobbleTurret {
    private DcMotorEx wobbleRotaryMotor, wobbleLinearMotor;
    private Servo claw, clawLift;
    private TouchSensor rotaryForwardSensor, rotaryBackSensor, linearOutSensor, linearInSensor;

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

    public WobbleTurret(HardwareMap hMap, TurretState startingState) {
        wobbleRotaryMotor = hMap.get(DcMotorEx.class, "wobble_rotary");
        wobbleLinearMotor = hMap.get(DcMotorEx.class, "wobble_linear");

        claw = hMap.servo.get("claw");
        clawLift = hMap.servo.get("clawLift");

        rotaryForwardSensor = hMap.touchSensor.get("rotary_forward");
        rotaryBackSensor = hMap.touchSensor.get("rotary_back");
        linearOutSensor = hMap.touchSensor.get("linear_out");
        linearInSensor = hMap.touchSensor.get("linear_in");

        turretState = startingState;
    }

    public void update() {
        if(movementState == BUSY) {
            
        }
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
            turretState = BACK_STOWED;
        } else if(turretState == BACK_STOWED) {
            turretState = BACK_DROP;
        } else if(turretState == BACK_DROP) {
            turretState = FORWARD_OPEN;
        }

        beginAction(turretState);
    }

    public void setState(TurretState state) {
        turretState = state;

        beginAction(state);
    }

    public void beginAction(TurretState state) {
        movementState = BUSY;
        if(state == STOWED) {
            tryRotaryForward();
            tryLinearForward();
            claw.setPosition(clawClosed);
            clawLift.setPosition(armUp);
        } else if(state == FORWARD_OPEN) {
            tryRotaryForward();
            tryLinearForward();
            claw.setPosition(clawOpen);
            clawLift.setPosition(armDown);
        } else if(state == FORWARD_CLOSED) {
            tryRotaryForward();
            tryLinearForward();
            claw.setPosition(clawClosed);
            clawLift.setPosition(armDown);
        }else if(state == FORWARD_STOWED) {
            tryRotaryForward();
            tryLinearForward();
            claw.setPosition(clawClosed);
            clawLift.setPosition(armUp);
        }else if(state == FORWARD_DROP) {
            tryRotaryForward();
            tryLinearForward();
            claw.setPosition(clawClosed);
            clawLift.setPosition(armDrop);
        } else if(state == BACK_OPEN) {
            tryRotaryBackward();
            tryLinearForward();
            claw.setPosition(clawOpen);
            clawLift.setPosition(armDown);
        } else if(state == BACK_CLOSED) {
            tryRotaryBackward();
            tryLinearForward();
            claw.setPosition(clawClosed);
            clawLift.setPosition(armDown);
        } else if(state == BACK_STOWED) {
            tryRotaryBackward();
            tryLinearForward();
            claw.setPosition(clawClosed);
            clawLift.setPosition(armUp);
        } else if(state == BACK_DROP) {
            tryRotaryBackward();
            tryLinearForward();
            claw.setPosition(clawClosed);
            clawLift.setPosition(armDrop);
        }
    }

    public void tryRotaryForward() {
        if(!rotaryForwardSensor.isPressed()) {
            wobbleRotaryMotor.setPower(rotaryPower);
        }
    }

    public void tryRotaryBackward() {
        if(!rotaryBackSensor.isPressed()) {
            wobbleRotaryMotor.setPower(rotaryPowerBack);
        }
    }

    public void tryLinearForward() {
        if(!linearOutSensor.isPressed()) {
            wobbleLinearMotor.setPower(linearPower);
        }
    }

    public void tryLinearBackward() {
        if(!linearInSensor.isPressed()) {
            wobbleLinearMotor.setPower(linearPowerBack);
        }
    }

    public enum TurretState {
        STOWED, FORWARD_OPEN, FORWARD_CLOSED, FORWARD_STOWED, FORWARD_DROP, BACK_OPEN, BACK_CLOSED, BACK_STOWED, BACK_DROP
    }

    public enum MovementState {
        IDLE, BUSY
    }


}
