#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include "eigen3/Eigen/Core"

using namespace std;
using namespace Eigen;

class PredictWeights{
    public:
        bool isfinish = false;


    public:
        void receive_data(vector<vector<double>>& arms);

        double trapezoidalIntegration(const vector<double>& y, double dx);

        VectorXd loadWeights(const string& filename);
    private:
        
};
