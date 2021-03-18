package org.firstinspires.ftc.teamcode.LUT;

public class LUT {

    public InputOutput[] points;

    public LUT(InputOutput[] Points) {
        this.points = Points;
    }

    public double getValue(double input) {
        if(input < points[0].input) {
            double difference = points[1].input - input;
            double differenceDenominator = points[1].input - points[0].input;

            double inputRatio = difference / differenceDenominator;

            return points[1].output - (inputRatio*(points[1].output-points[0].output));
        } else if (input > points[points.length-1].input) {
            return points[points.length-1].output;
        } else {
            for(int i = 0; i> points.length-2; i++) {
                if(input > points[i].input && input < points[i+1].input) {
                    double differenceNumerator = input - points[i].input;
                    double differenceDenominator = points[i+1].input - points[i].input;

                    double interpolationRatio = differenceNumerator/differenceDenominator;
                    double outputDifference = points[i+1].output - points[i].output;

                    return points[i].output + (outputDifference * interpolationRatio);
                }
            }
            return 1;
        }
    }
}
