package org.usfirst.frc.team2077.drivetrain;

import static org.usfirst.frc.team2077.Robot.*;

public class DifferentialChassis extends AbstractChassis {

    private final DifferentialMath differentialMath_;

    // Velocity setpoint.
    private double north_ = 0;
    private double east_ = 0;
    private double clockwise_ = 0;

    private final double translationFactor_;
    private final double rotationFactor_;

    private final double translationFactor01_;
    private final double rotationFactor01_;

    private final double topSpeed_;

    /**
     * 
     * @param driveModule [2]
     * @param trackWidth
     * @param wheelRadius
     */
    public DifferentialChassis(DriveModule[] driveModule, double wheelbase, double trackWidth, double wheelRadius) {
        super(driveModule, wheelbase, trackWidth, wheelRadius);

        differentialMath_ = new DifferentialMath(trackWidth_, wheelRadius_);

        // velocity and rotation conversion from input to MecanumMath units
        double tf0 = 0;
        double rf0 = 0;
        double tf1 = 1 / differentialMath_.inverse(new double[] {1, 0})[0];
        double rf1 = 1 / differentialMath_.inverse(new double[] {0, -1})[0];
        double tf2 = differentialMath_.forward(new double[]{1, 1})[0];
        double rf2 = differentialMath_.forward(new double[]{-1, 1})[1];
        translationFactor_ = tf1;
        rotationFactor_ = rf1;

        // wheel velocity conversion from 0-1 range to DriveModule maximum
        topSpeed_ = robot_.constants_.TOP_SPEED;

        translationFactor01_ = topSpeed_;
        rotationFactor01_ = 180/Math.PI;

        System.out.println("T=" + tf0 + " T1=" + tf1 + "T2=" + tf2 + " R=" + rf0 + " R1=" + rf1 + " R2=" + rf2);
    }

    @Override
    public void setVelocity(double north, double east, double clockwise) {
        setVelocity01(north/translationFactor01_, east/translationFactor01_, clockwise/rotationFactor01_);
    }

    @Override
    public void setVelocity(double north, double east) {
        setVelocity01(north/translationFactor01_, east/translationFactor01_);
    }

    @Override
    public void setRotation(double clockwise) {
        setRotation01(clockwise/rotationFactor01_);
    }

    @Override
    public void setVelocity01(double north, double east, double clockwise) {
        north_ = Math.max(-1.0, Math.min(1.0, north));
        east_ = 0;
        clockwise_ = Math.max(-1.0 ,Math.min(1.0, clockwise));

        updateMotorSpeeds();
    }

    @Override
    public void setVelocity01(double north, double east) {
        north_ = Math.max(-1.0, Math.min(1.0, north));
        east_ = 0;

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
        return new double[] {north_, clockwise_};
    }

    private void updateMotorSpeeds() {
        double[] v = getVelocitySet();
        v[0] *= translationFactor_;
        v[1] *= rotationFactor_;
        //System.out.println("DIFFERENTIAL INPUT: " + v[0] + " " + v[1]);
        double[] w = differentialMath_.inverse(v);
        double max = 1;
        for (double ws : w) {
            max = Math.max(max, Math.abs(ws));
        }
        //System.out.print("DRIVE MODULE INPUT:");
        for (int i = 0; i < w.length; i++) {
            double ws = w[i] / max;
            ws *= topSpeed_;
            //System.out.print(" " + Math.round(ws));
            driveModule_[i].setVelocity(ws);
            //System.out.print("(" + Math.round(driveModule_[i].getVelocity()) + ")");
        }
        // System.out.println();
    }

    /**
     * Simplified MecanumMath knockoff.
     */ 
    private static class DifferentialMath {

        private final double chassisRadius_;

        private final double wheelRadius_;

        /**
         * Calls to {@link #inverse(double[])} or {@link #forward(double[])} will calculate translation motions
         * using the same length units as width and wheel radius, and robot and wheel rotations in radians.
         * @param width Distance between wheel contact points in the E/W direction, in any distance unit.
         * @param wheelRadius Radius of each wheel, in the same unit.
         */
        public DifferentialMath(double width, double wheelRadius) {
            chassisRadius_ = width/2;
            wheelRadius_ = wheelRadius;
        }

        /***
         * @param V <b>[V]</b>: [north/south translation, rotation], in length units and radians respectively.
         * @return The wheel speed vector <b>[&#x03A9;]</b>: [E, W] in radians.
         */
        public final double[] inverse(double[] V) {

            double translationWheelRadians = V[0] / wheelRadius_;
            double rotationWheelRadians_ = V[1] * (chassisRadius_/wheelRadius_);

        	return new double[] {translationWheelRadians - rotationWheelRadians_, translationWheelRadians + rotationWheelRadians_};
        }

        /***
         * @param O A wheel speed vector <b>[&#x03A9;]</b>: [E, W] in radians.
         * @return [2] robot motion velocities [north/south translation, rotation] in length units and radians respectively.
         */
        public final double[] forward(double[] O) {

            double translationWheelRadians = (O[0] + O[1]) / 2;
            double rotationWheelRadians = (O[1] - O[0]) / 2;
 
            return new double[] {translationWheelRadians * wheelRadius_, rotationWheelRadians / (chassisRadius_/wheelRadius_)};
        }
    }
}