using MathNet.Numerics.LinearAlgebra.Double;

using System;

namespace KalmanFilters
{
    public class KalmanFilter : IKalmanFilter
    {
        public Matrix StateTransitionModel;
        public Matrix ObservationModel;
        public Matrix ProcessNoise;
        public Matrix ControlInputModel;
        public Matrix MeasurementCovariance;

        public Vector StateEstimate { get; private set; }
        public Matrix EstimateCovariance { get; private set; }

        private Vector aPrioriState;
        private Matrix aPrioriCovariance;
        
        public KalmanFilter(Matrix stateTransition, Matrix processNoiseCovariance, Matrix measurementCovariance, Matrix observationModel, Matrix controlInputModel, Vector stateEstimate, Matrix estimateCovariance)
        {
            StateTransitionModel = stateTransition;
            ProcessNoise = processNoiseCovariance;
            ObservationModel = observationModel;
            ControlInputModel = controlInputModel;
            MeasurementCovariance = measurementCovariance;
            StateEstimate = stateEstimate;
            EstimateCovariance = estimateCovariance;
        }

        public void Predict(Vector control)
        {
            aPrioriState = (Vector)(StateTransitionModel * StateEstimate + ControlInputModel * control);
            aPrioriCovariance = (Matrix)(StateTransitionModel * EstimateCovariance * StateTransitionModel.Transpose() + ProcessNoise);
        }

        public void Predict()
        {
            aPrioriState = (Vector)(StateTransitionModel * StateEstimate);
            aPrioriCovariance = (Matrix)(StateTransitionModel * EstimateCovariance * StateTransitionModel.Transpose() + ProcessNoise);
        }

        public void Update(Vector measurements)
        {
            var residual = measurements - ObservationModel * aPrioriState;

            var a = ObservationModel * aPrioriCovariance;
            var b = a * ObservationModel.Transpose();

            var covariance = b + MeasurementCovariance;// ObservationModel * aPrioriCovariance * ObservationModel.Transpose() + MeasurementCovariance;

            var kalmanGain = aPrioriCovariance * ObservationModel.Transpose() * covariance.Inverse();

            StateEstimate = (Vector)(aPrioriState + kalmanGain * residual);

            EstimateCovariance = (Matrix)(aPrioriCovariance - kalmanGain * ObservationModel * aPrioriCovariance);
        }
    }
}
