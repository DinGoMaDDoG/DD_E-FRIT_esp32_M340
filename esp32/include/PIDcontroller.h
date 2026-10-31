/*
 * I-PD speed controller with database-driven gain scheduling (DD-E-FRIT)
 *
 * compute() runs one step of a discrete I-PD law in velocity form:
 *
 *   du(t) = Ki*e(t)*dt - Kp*dy(t) - Kd*d2y(t)/dt,   u(t) = u(t-1) + du(t)
 *
 * The integral term acts on the error e = r - y, while the proportional and
 * derivative terms act on the measured speed y only, so a setpoint step does
 * not kick the output. y is low-pass filtered first (EMA, alpha = 0.2), and
 * u is clamped to [outputMin, outputMax].
 *
 * updateWeightsandPID() chooses Kp, Ki, Kd for the current operating point:
 * it compares the query vector
 *
 *   [r(t), r(t-1), y(t), y(t-1), u(t-1), u(t-2)]
 *
 * with every record of the database in dataset.h (range-normalised L1
 * distance), keeps the k = 20 nearest records, and averages their gains with
 * weights exp(-(d - d_min)), normalised to sum to 1.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include "dataset.h"

class PIDController {
public:
    float setPoint, outputMin, outputMax;
    float lastRPM, lastDeltaRPM;
    float kp, ki, kd;
    float currentOutput;

    // Past values used to build the query vector.
    float past_sp;
    float past_rpm;
    float past_pwm;
    float past2_pwm;


    // Range (max - min) of one column of the database; never zero.
    template<size_t Rows, size_t Cols>
    float col_range(const float (&matrix)[Rows][Cols], int col_index){
        float col_min = matrix[0][col_index];
        float col_max = matrix[0][col_index];

        for (size_t row=1; row<Rows; ++row){
            if (matrix[row][col_index] < col_min) col_min=matrix[row][col_index];
            if (matrix[row][col_index] > col_max) col_max=matrix[row][col_index];
        }

        float range=col_max-col_min;

        return (range==0.0f) ? 1e-6f : range;
    }

    // 1/range for each element of the query vector, computed once.
    const float inv_ranges[6]={
        1.0f/col_range(infoMatrix, 0),  // r(t)
        1.0f/col_range(infoMatrix, 0),  // r(t-1)
        1.0f/col_range(infoMatrix, 1),  // y(t)
        1.0f/col_range(infoMatrix, 1),  // y(t-1)
        1.0f/col_range(infoMatrix, 2),  // u(t-1)
        1.0f/col_range(infoMatrix, 2)   // u(t-2)
    };


    PIDController(float p, float i, float d, float min, float max) {
        kp=p; ki=i; kd=d;
        outputMin=min; outputMax=max;
        reset();

    }

    void reset() {
        currentOutput=0;
        lastRPM=0;
        lastDeltaRPM=0;
        setPoint=0;

        past_sp=0;
        past_pwm=0;
        past_rpm=0;
        past2_pwm=0;
        filteredRPM=0;
    }

    // KNN lookup of the gains for the current operating point (see top of file).
    void updateWeightsandPID (float current_sp, float current_rpm){
        const int k=20;
        float top_distances[k];
        int distance_indices[k];

        for(int i=0; i<k; i++){
            top_distances[i]=3.4028235E+38f; // FLT_MAX
        }

        // Records 0 and 1 have no complete history, so the search starts at 2.
        for(int i=2; i<dataset_size; i++){
            float d=0;
            d+=fabs(current_sp-infoMatrix[i][0])*inv_ranges[0];
            d+=fabs(past_sp-infoMatrix[i-1][0])*inv_ranges[1];
            d+=fabs(current_rpm-infoMatrix[i][1])*inv_ranges[2];
            d+=fabs(past_rpm-infoMatrix[i-1][1])*inv_ranges[3];
            d+=fabs(past_pwm-infoMatrix[i-1][2])*inv_ranges[4];
            d+=fabs(past2_pwm-infoMatrix[i-2][2])*inv_ranges[5];

            // Insert into the sorted list of the k smallest distances.
            if (d < top_distances[k-1]){
                int j=k-1;
                while (j>0 && d<top_distances[j-1]){
                    top_distances[j]=top_distances[j-1];
                    distance_indices[j]=distance_indices[j-1];
                    j--;
                }
                top_distances[j]=d;
                distance_indices[j]=i;
            }
        }

        // Exponential weights; subtracting the smallest distance keeps exp() in range.
        float weights[k];
        float total_exp=0.0f;
        float min_dist=top_distances[0];

        for(int i=0; i<k; i++){
            weights[i]=exp(-(top_distances[i]-min_dist));
            total_exp+=weights[i];
        }

        float weights_kp=0.0f;
        float weights_ki=0.0f;
        float weights_kd=0.0f;

        if (total_exp > 0) {
            for (int i=0; i<k; i++) {
                float normalized_weight=weights[i]/total_exp;
                int idx=distance_indices[i];

                weights_kp+=normalized_weight*pidMatrix[idx][0];
                weights_ki+=normalized_weight*pidMatrix[idx][1];
                weights_kd+=normalized_weight*pidMatrix[idx][2];
            }

            kp=weights_kp;
            ki=weights_ki;
            kd=weights_kd;
        }

        // Shift the history for the next query.
        past2_pwm=past_pwm;
        past_pwm=currentOutput;
        past_sp=current_sp;
        past_rpm=current_rpm;
    }

    float filteredRPM = 0;

    // One step of the I-PD law (see top of file). Returns the new PWM duty.
    float compute(float rawRPM, float dt) {
        float alpha=0.2;
        filteredRPM=(alpha*rawRPM)+((1.0-alpha)*filteredRPM);

        float deltaRPM=filteredRPM-lastRPM;
        float delta2RPM=deltaRPM-lastDeltaRPM;

        float error=setPoint-filteredRPM;
        float deltaU=(ki*error*dt)-(kp*deltaRPM)-(kd*delta2RPM/dt);

        currentOutput+=deltaU;

        if (currentOutput>outputMax) currentOutput=outputMax;
        else if (currentOutput<outputMin) currentOutput=outputMin;

        lastRPM=filteredRPM;
        lastDeltaRPM=deltaRPM;

        return currentOutput;
    }

};

#endif
