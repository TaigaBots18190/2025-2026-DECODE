package org.firstinspires.ftc.teamcode.pedroPathing;

import java.util.TreeMap;
import java.util.Map;


public class MonotonicCubicSpline {
    //arrays to store our three values x,y,m(tangents)
    private double[] x, y, m;

   //this is a constructor which creates a basis to do the interpolation
    public MonotonicCubicSpline(TreeMap<Double, Double> table) {
        int n = table.size();
        x = new double[n];
        y = new double[n];
        m = new double[n];

        // sort the data from the tree map for efficiency
        int i = 0;
        for (Map.Entry<Double, Double> entry : table.entrySet()) {
            x[i] = entry.getKey(); // stores the distance
            y[i] = entry.getValue(); // stores the y value
            i++; // moves on to the next index
        }

        // Calculate Secant Slopes
        double[] secants = new double[n - 1]; // always one less "gap" than there is number of points
        for (i = 0; i < n - 1; i++) {
            secants[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
        }

        // Initialize Tangents
        // Estimate the derivative (tangent) at each point.
        // Internal tangents are initialized as the average of the two adjacent secants.
        m[0] = secants[0]; // the first point just takes the first slope
        for (i = 1; i < n - 1; i++) {  // for the ones in the middle average the one before and after
            m[i] = (secants[i - 1] + secants[i]) / 2.0;
        }
        m[n - 1] = secants[n - 2]; // the last one take the last possible slope

        // Fritsch-Carlson Monotonicity Adjustment
        // Modify tangents to ensure they are monotonic
        for (i = 0; i < n - 1; i++) {
            if (secants[i] == 0) {
                // If the secant slope is zero, the tangents at both endpoints must be zero
                // to maintain a constant value across the interval.
                m[i] = 0;
                m[i + 1] = 0;
            } else {
                // Calculate the ratio of the tangents to the secant slope.
                double a = m[i] / secants[i]; // ratio of the start tangent to the slope
                double b = m[i + 1] / secants[i]; // ratio of the end tangent to the slope

                // If the vector sum of these ratios exceeds 3.0, the spline may not be monotonic.
                double h = Math.hypot(a, b);
                if (h > 3.0) {
                    // Rescale the tangents to make sure they stay monotonic
                    double t = 3.0 / h; // calculate the scaling value
                    m[i] = t * a * secants[i]; // scale this down
                    m[i + 1] = t * b * secants[i]; // scale this down
                }
            }
        }
    }

    //performs the hermite interpolation for a given value
    public double interpolate(double val) {
        // Handle boundary conditions by clamping to the first or last known ordinate.
        if (val <= x[0]) return y[0];
        if (val >= x[x.length - 1]) return y[y.length - 1];

        // Locate the interval that contains the input value.
        int i = 0;
        while (val >= x[i + 1]) i++;

        // Calculate the interval width and the normalized distance (t) within the interval.
        double h = x[i + 1] - x[i];
        double t = (val - x[i]) / h;

        // --- Cubic Hermite Basis Functions ---
        // These polynomials define the influence of the endpoint values and tangents
        // on the interpolated value.
        double h00 = (1 + 2 * t) * (1 - t) * (1 - t); // influence of the last point
        double h10 = t * (1 - t) * (1 - t);           // influence of the last point's slope
        double h01 = t * t * (3 - 2 * t);             // influence of the point above
        double h11 = t * t * (t - 1);                 // influence of the point above's slope

        // Return the interpolated value
        return h00 * y[i] + h10 * h * m[i] + h01 * y[i + 1] + h11 * h * m[i + 1];
    }
}