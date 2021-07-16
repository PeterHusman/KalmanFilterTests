using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;

using System;
using System.Collections.Generic;
using System.Text;

namespace KalmanFilters
{
    public class UnscentedKalmanFilter : IKalmanFilter
    {
        public Func<Vector, Vector> StateTransitionModel;
        public Matrix ProcessNoiseCovariance;
        public Func<Vector, Vector> ObservationModel;
        public Matrix MeasurementCovariance;
        public Vector StateEstimate { get; private set; }
        public Matrix EstimateCovariance { get; private set; }
        private (Vector<double> sigmaPoint, double weight)[] sigmas;

        private Vector<double> predictedMeasurements;
        private Matrix predictedMeasurementCovariance;

        private Matrix<double> kalmanGain;

        public double Kappa;

        public UnscentedKalmanFilter(Func<Vector, Vector> stateTransitionModel, Matrix processNoiseCovariance, Func<Vector, Vector> observationModel, Matrix measurementNoiseCovariance, Vector stateEstimate, Matrix estimateCovariance, double kappa)
        {
            StateTransitionModel = stateTransitionModel;
            ObservationModel = observationModel;
            StateEstimate = stateEstimate;
            EstimateCovariance = estimateCovariance;
            ProcessNoiseCovariance = processNoiseCovariance;
            MeasurementCovariance = measurementNoiseCovariance;
            Kappa = kappa;
        }

        public void Predict()
        {
            sigmas = GetSigmaPoints(StateEstimate, EstimateCovariance, Kappa);
            var sigmaCopy = new (Vector<double>, double)[sigmas.Length];
            for (int i = 0; i < sigmas.Length; i++)
            {
                sigmaCopy[i] = (StateTransitionModel((Vector)sigmas[i].sigmaPoint), sigmas[i].weight);
            }
            // THIS STEP is to blame.
            // Why??
            // Replacing EstimateCovariance with _ (discard) works wonders and eliminates the covariance explosion.
            // Obviously gotta fix the issue tho
            //
            // Wait what if the error isn't here but is instead in the kalman gain or smthn overfavoring the model?
            // and not decreasing the covariances enough?
            // basically just
            // underfavoring the sensor data
            //
            // Also now im leaning towards covariance isnt high *enough*
            // Because variance ends up like on the order of 10^4 or 10^5
            // But the error is like 10^10 or 10^20 or smthn
            (StateEstimate, EstimateCovariance) = UnscentedTransform(sigmaCopy, ProcessNoiseCovariance);

            // Maybe put this here? I'm not 100%
            // sigmas = GetSigmaPoints(StateEstimate, EstimateCovariance, Kappa);

            var sigmaCopy2 = new (Vector<double>, double)[sigmas.Length];
            for (int i = 0; i < sigmas.Length; i++)
            {
                sigmaCopy2[i] = (ObservationModel((Vector)sigmas[i].sigmaPoint), sigmas[i].weight);
            }
            (predictedMeasurements, predictedMeasurementCovariance) = UnscentedTransform(sigmaCopy2, MeasurementCovariance);

            Matrix<double> stateAndPredictedMeasurementCovariance = new DenseMatrix(StateEstimate.Count, predictedMeasurements.Count);
            for (int i = 0; i < sigmas.Length; i++)
            {
                stateAndPredictedMeasurementCovariance += sigmas[i].weight * ((sigmaCopy[i].Item1 - StateEstimate).ToColumnMatrix() /*.OuterProduct*/ * (sigmaCopy2[i].Item1 - predictedMeasurements).ToRowMatrix());
            }

            kalmanGain = stateAndPredictedMeasurementCovariance * predictedMeasurementCovariance.Inverse();
        }

        public void Update(Vector measurements)
        {
            StateEstimate = (Vector)(StateEstimate + kalmanGain * (measurements - predictedMeasurements));
            EstimateCovariance = (Matrix)(EstimateCovariance - kalmanGain * predictedMeasurementCovariance * kalmanGain.Transpose());
        }


        public static (Vector<double> sigmaPoint, double weight)[] GetSigmaPoints(Vector mean, Matrix covariance, double kappa)
        {
            int n = mean.Count;
            var chol = ((n + kappa) * covariance).Cholesky().Factor;

            
            double nPlusKappa = n + kappa;

            //chol.Row(i)
            List<(Vector<double> sigma, double weight)> points = new List<(Vector<double> sigma, double weight)>(mean.Count * 2 + 1);

            points.Add((mean, kappa /*or is it just 1?*/ / nPlusKappa));

            double weight = 1 / (2 * nPlusKappa);

            for (int i = 0; i < n; i++)
            {
                var row = chol.Row(i);
                points.Add((mean + row, weight));
                points.Add((mean - row, weight));
            }

            return points.ToArray();
        }

        public static (Vector mean, Matrix covariance) UnscentedTransform((Vector<double> sigmaPoint, double weight)[] sigmaPointsWithWeights, Matrix noiseCovariance)
        {
            Vector<double> mean = new DenseVector(sigmaPointsWithWeights[0].sigmaPoint.Count);
            foreach ((Vector<double> sigma, double weight) in sigmaPointsWithWeights)
            {
                mean += sigma * weight;
            }
            
            Matrix<double> covariance = new DenseMatrix(noiseCovariance.RowCount);

            foreach ((Vector<double> sigma, double weight) in sigmaPointsWithWeights)
            {
                covariance += weight * (sigma - mean).ToColumnMatrix() * (sigma - mean).ToRowMatrix();
            }

            // Testing
            var sum = 0.0;
            for (int i = 0; i < sigmaPointsWithWeights.Length; i++)
            {
                sum += sigmaPointsWithWeights[i].weight;
            }

            if (sum != 1)
            {
                ;
            }

            covariance += noiseCovariance;
            return ((Vector)mean, (Matrix)covariance);
        }
    }
}
