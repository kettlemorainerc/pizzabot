package org.usfirst.frc.team2077.drivetrain;

import static org.usfirst.frc.team2077.Robot.*;

public class MecanumChassis extends AbstractChassis {

    private final MecanumMath mecanumMath_;

    // Velocity setpoint.
    private double north_ = 0;
    private double east_ = 0;
    private double clockwise_ = 0;

    private final double translationFactor_;
    private final double rotationFactor_;

    private final double topSpeed_;

    public MecanumChassis(DriveModule[] driveModule, double wheelbase, double trackWidth, double wheelRadius) {
        super(driveModule, wheelbase, trackWidth, wheelRadius);

        mecanumMath_ = new MecanumMath(wheelbase_, trackWidth_, wheelRadius_);

        // velocity and rotation conversion from input to MecanumMath units
        double tf0
            = wheelRadius_ /* inches */
            / 1 /* radian */ ;
        double rf0 // not right!
            = (2*Math.PI*wheelRadius_) /* one wheel rotation in inches */
            / (Math.sqrt(wheelbase_*wheelbase_+trackWidth_*trackWidth_)) /* one robot rotation in inches */ ;
        double tf1 = 1 / mecanumMath_.inverse(new double[] {1, 0, 0})[0];
        double rf1 = 1 / mecanumMath_.inverse(new double[] {0, 0, -1})[0];
        double tf2 = mecanumMath_.forward(new double[]{1, 1, 1, 1})[0];
        double rf2 = mecanumMath_.forward(new double[]{-1, -1, 1, 1})[2];
        translationFactor_ = tf1;
        rotationFactor_ = rf2;

        // wheel velocity conversion from 0-1 range to DriveModule maximum
        topSpeed_ = robot_.constants_.TOP_SPEED;

        System.out.println("T=" + tf0 + " T1=" + tf1 + "T2=" + tf2 + " R=" + rf0 + " R1=" + rf1 + " R2=" + rf2);
    }

    @Override
    public void setVelocity01(double north, double east, double clockwise) {
        north_ = Math.max(-1.0, Math.min(1.0, north));
        east_ = Math.max(-1.0, Math.min(1.0, east));
        clockwise_ = Math.max(-1.0 ,Math.min(1.0, clockwise));

        updateMotorSpeeds();
    }

    @Override
    public void setVelocity01(double north, double east) {
        north_ = Math.max(-1.0, Math.min(1.0, north));
        east_ = Math.max(-1.0, Math.min(1.0, east));

        updateMotorSpeeds();
     }

    @Override
    public void setRotation01(double clockwise) {
        clockwise_ = Math.max(-1.0, Math.min(1.0, clockwise));

        updateMotorSpeeds();
    }

    @Override
    public void stop() {
        setVelocity01(0, 0, 0);
    }

    @Override
    public double[] getVelocitySet() {
        return new double[] {north_, east_, clockwise_};
    }

    private void updateMotorSpeeds() {
        double[] v = getVelocitySet();
        v[0] *= translationFactor_;
        v[1] *= translationFactor_;
        v[2] *= rotationFactor_;
        //System.out.println("MECANUM INPUT: " + v[0] + " " + v[1] + " " + v[2]);
        double[] w = mecanumMath_.inverse(v);
        double max = 1;
        for (double ws : w) {
            max = Math.max(max, Math.abs(ws));
        }
        System.out.print("DRIVE MODULE INPUT:");
        for (int i = 0; i < w.length; i++) {
            double ws = w[i] / max;
            ws *= topSpeed_;
            System.out.print(" " + Math.round(ws));
            driveModule_[i].setVelocity(ws);
            System.out.print("(" + Math.round(driveModule_[i].getVelocity()) + ")");
        }
        // System.out.println();
    }
}