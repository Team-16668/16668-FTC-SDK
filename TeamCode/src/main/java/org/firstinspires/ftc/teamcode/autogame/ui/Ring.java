package org.firstinspires.ftc.teamcode.autogame.ui;

public class Ring {
    private double x;
    private double y;
    private String color;
    private boolean currentlyTargeted;

    public Ring(double x, double y, String color, boolean currentlyTargeted) {
        this.x = x;
        this.y = y;
        this.color = color;
        this.currentlyTargeted = currentlyTargeted;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setColor(String color) {
        this.color = color;
    }

    public String getColor() {
        return color;
    }

    public boolean isCurrentlyTargeted() {
        return currentlyTargeted;
    }

    public void setCurrentlyTargeted(boolean currentlyTargeted) {
        this.currentlyTargeted = currentlyTargeted;
    }
}
