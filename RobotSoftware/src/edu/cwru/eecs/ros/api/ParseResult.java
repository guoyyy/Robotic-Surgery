package edu.cwru.eecs.ros.api;
//Parse the matlab return value
public class ParseResult {

    public static double[][] formatResult(Object result, int n) {
        if (result == null) {
            return null;
        }
        double[] array = (double[]) result;
        int m = array.length / n;
        double[][] mat = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                mat[i][j] = array[i * n + j];
            }
        }
        return mat;

    }
}
