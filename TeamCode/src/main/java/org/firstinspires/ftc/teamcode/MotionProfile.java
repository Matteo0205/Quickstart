package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

public class MotionProfile {

    private final double maxVelocity;
    private final double maxAcceleration;
    private final double maxDeceleration;

    public enum State {
        FINISHED,
        WAITING,
        ACCELERATING,
        CRUISING,
        DECELERATING
    }

    public static class Params {
        public double maxVelocity = Double.MAX_VALUE;
        public double maxAcceleration;
        public double maxDeceleration;

        public Params(double maxAcceleration) {
            this.maxAcceleration = maxAcceleration;
            this.maxDeceleration = maxAcceleration;
        }

        public Params(double maxVelocity, double maxAcceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxDeceleration = maxAcceleration;
        }
        public Params(double maxVelocity, double maxAcceleration, double maxDeceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxDeceleration = maxDeceleration;
        }
    }

    private State state = State.FINISHED;
    private final Map<Double, Precalculation> precalculations = new HashMap<>();
    private long startedTimestamp = -1L;

    public MotionProfile(double maxVelocity, double maxAcceleration, double maxDeceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
    }

    public MotionProfile(Params params) {
        this(params.maxVelocity, params.maxAcceleration, params.maxDeceleration);
    }

    public State getState() {
        return state;
    }

    public boolean isTransitioning() {
        return state == State.ACCELERATING || state == State.CRUISING || state == State.DECELERATING;
    }

    public void init(double... distances) {
        for (double distance : distances) {
            preCalculate(distance);
        }
    }

    public void reset() {
        startedTimestamp = -1L;
        state = State.WAITING;
    }

    public Precalculation getCalculation(double distance) {
        return precalculations.computeIfAbsent(distance, this::preCalculate);
    }

    public void initAllDistances(double... distances) {
        precalculations.clear();
        for (double distance1 : distances) {
            for (double distance2 : distances) {
                double delta = Math.abs(distance2 - distance1);
                init(delta);
            }
        }
    }

    public double calculate(double distance) {
        if (distance == 0.0) {
            state = State.FINISHED;
            return 0.0;
        }

        if (startedTimestamp == -1L) {
            startedTimestamp = System.currentTimeMillis();
        }

        double elapsedTime = (System.currentTimeMillis() - startedTimestamp) / 1000.0;
        Precalculation calc = getCalculation(distance);

        if (elapsedTime > calc.totalTime) {
            state = State.FINISHED;
            return calc.distance;
        } else if (elapsedTime <= calc.accelTime) {
            state = State.ACCELERATING;
            return calc.calculateAcceleration(elapsedTime);
        } else if (elapsedTime >= calc.totalTime - calc.decelTime) {
            state = State.DECELERATING;
            return calc.calculateDeceleration(elapsedTime);
        } else {
            state = State.CRUISING;
            return calc.calculateCruise(elapsedTime);
        }
    }

    private Precalculation preCalculate(double distance) {
        if (distance == 0.0) {
            return new Precalculation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        double halfDistance = distance / 2;
        double accelTime = Math.min(maxVelocity / maxAcceleration,
                Math.sqrt(halfDistance / (0.5 * maxAcceleration)));
        double accelDistance = 0.5 * maxAcceleration * Math.pow(accelTime, 2);
        double actualMaxVelocity = maxAcceleration * accelTime;

        double decelTime = actualMaxVelocity / maxDeceleration;
        double decelDistance = 0.5 * maxDeceleration * Math.pow(decelTime, 2);

        if (accelDistance + decelDistance > distance) {
            actualMaxVelocity = Math.sqrt(2 * distance * (maxAcceleration * maxDeceleration) /
                    (maxAcceleration + maxDeceleration));
            accelTime = actualMaxVelocity / maxAcceleration;
            accelDistance = 0.5 * maxAcceleration * Math.pow(accelTime, 2);

            decelTime = actualMaxVelocity / maxDeceleration;
            decelDistance = 0.5 * maxDeceleration * Math.pow(decelTime, 2);
        }

        double cruiseDistance = distance - accelDistance - decelDistance;
        double cruiseTime = cruiseDistance > 0 ? cruiseDistance / actualMaxVelocity : 0.0;
        double totalTime = accelTime + cruiseTime + decelTime;

        Precalculation p = new Precalculation(distance, accelTime, accelDistance,
                decelTime, decelDistance, cruiseTime, cruiseDistance,
                actualMaxVelocity, totalTime);
        precalculations.put(distance, p);
        return p;
    }

    public class Precalculation {
        public final double distance;
        public final double accelTime;
        public final double accelDistance;
        public final double decelTime;
        public final double decelDistance;
        public final double cruiseTime;
        public final double cruiseDistance;
        public final double actualMaxVelocity;
        public final double totalTime;

        public Precalculation(double distance, double accelTime, double accelDistance,
                              double decelTime, double decelDistance, double cruiseTime,
                              double cruiseDistance, double actualMaxVelocity, double totalTime) {
            this.distance = distance;
            this.accelTime = accelTime;
            this.accelDistance = accelDistance;
            this.decelTime = decelTime;
            this.decelDistance = decelDistance;
            this.cruiseTime = cruiseTime;
            this.cruiseDistance = cruiseDistance;
            this.actualMaxVelocity = actualMaxVelocity;
            this.totalTime = totalTime;
        }

        public double calculateAcceleration(double elapsedTime) {
            return 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        }

        public double calculateCruise(double elapsedTime) {
            double timeInCruise = elapsedTime - accelTime;
            return accelDistance + actualMaxVelocity * timeInCruise;
        }

        public double calculateDeceleration(double elapsedTime) {
            double timeInDecel = elapsedTime - (accelTime + cruiseTime);
            return accelDistance + cruiseDistance +
                    (actualMaxVelocity * timeInDecel - 0.5 * maxDeceleration * Math.pow(timeInDecel, 2));
        }
    }
}