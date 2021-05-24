package org.firstinspires.ftc.teamcode.ui;

public class Cursor {
    private int x;
    private int y;

    private double speed;


    /**
     * This is the cursor for the autonomous teleop that will move around the dashboard.
     * @param x X for start position of cursor
     * @param y Y for start position of cursor
     * @param speed Speed of the cursor. The controls can be inverted by making it negative
     */
    public Cursor(int x, int y, double speed) {
        this.x = x;
        this.y = y;
        this.speed = speed;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public void setX(int x) {
        this.x = x;
    }

    public void setY(int y) {
        this.y = y;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }

    /**
     * This applies the cursors movement for the current loop of the program.
     * @param up These are all just booleans for the controls on the gamepad.
     * @param down
     * @param left
     * @param right
     * @param leftBumper
     * @param rightBumper
     */
    public void applyMovement(boolean up, boolean down, boolean left, boolean right, boolean leftBumper, boolean rightBumper) {
        double speedMultiplier = 1;
        if(leftBumper) {
            speedMultiplier /= 2;
        } else if(rightBumper) {
            speedMultiplier *= 2;
        }

        double adjustedSpeed = this.speed * speedMultiplier;

        if(up) {
            this.x += adjustedSpeed;
        }else if(down) {
            this.x -= adjustedSpeed;
        }

        if(left) {
            this.y += adjustedSpeed;
        }else if(right) {
            this.y -= adjustedSpeed;
        }


    }

}
