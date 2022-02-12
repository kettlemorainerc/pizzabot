package org.usfirst.frc.team2077.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Crosshairs extends SubsystemBase {
    private int width_;
    private int height_;
    private double[] crosshairs = new double[] {0, 0};

    //width and height are width and height constraints of the image
    public Crosshairs(int width, int height) {
        width_ = width;
        height_ = height;
    }

    public void set(double x, double y) {
        boolean heightLim = y >= height_; 
        boolean widthLim = x >= width_;
        crosshairs[0] = x;
        crosshairs[1] = y;
        if (Math.abs(y) >= height_) {
            crosshairs[1] = Math.copySign(height_, y);
        }
        if (Math.abs(x) >= width_) {
            crosshairs[0] = Math.copySign(width_, x);

        }
        
    }

    public double[] get() {
        return new double[] {crosshairs[0], crosshairs[1]};
    }

    public double getAzimuth() {
        // TODO: Convert crosshair to degrees using lens function.
        return 0;
    }

    public double getElevation() {
        // TODO: Convert crosshair to degrees using lens function.
        return 0;
    }

    @Override
    public String toString() {
        return "(" + Math.round(crosshairs[0]) + "," + Math.round(crosshairs[1]) + "}";
    }
}