package org.firstinspires.ftc.teamcode.pedroPathing;

import java.util.TreeMap;
import java.util.Map;

public class MonotonicCubicSpline {
    private double[] x, y, m;

    public MonotonicCubicSpline(TreeMap<Double, Double> table) {
        int n = table.size();
        x = new double[n];
        y = new double[n];
        m = new double[n];

        int i = 0;
        for (Map.Entry<Double, Double> entry : table.entrySet()) {
            x[i] = entry.getKey();
            y[i] = entry.getValue();
            i++;
        }

        // 1. Calculate Secant Slopes
        double[] secants = new double[n - 1];
        for (i = 0; i < n - 1; i++) {
            secants[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
        }

        // 2. Initial Tangents (Average of secants)
        m[0] = secants[0];
        for (i = 1; i < n - 1; i++) {
            m[i] = (secants[i - 1] + secants[i]) / 2.0;
        }
        m[n - 1] = secants[n - 2];

        // 3. Fritsch-Carlson Monotonicity Adjustment
        for (i = 0; i < n - 1; i++) {
            if (secants[i] == 0) {
                m[i] = 0;
                m[i + 1] = 0;
            } else {
                double a = m[i] / secants[i];
                double b = m[i + 1] / secants[i];
                double h = Math.hypot(a, b);
                if (h > 3.0) {
                    double t = 3.0 / h;
                    m[i] = t * a * secants[i];
                    m[i + 1] = t * b * secants[i];
                }
            }
        }
    }

    public double interpolate(double val) {
        // Handle boundaries
        if (val <= x[0]) return y[0];
        if (val >= x[x.length - 1]) return y[y.length - 1];

        // Find the interval
        int i = 0;
        while (val >= x[i + 1]) i++;

        double h = x[i + 1] - x[i];
        double t = (val - x[i]) / h;

        // Hermite Basis Functions
        double h00 = (1 + 2 * t) * (1 - t) * (1 - t);
        double h10 = t * (1 - t) * (1 - t);
        double h01 = t * t * (3 - 2 * t);
        double h11 = t * t * (t - 1);

        return h00 * y[i] + h10 * h * m[i] + h01 * y[i + 1] + h11 * h * m[i + 1];
    }
}