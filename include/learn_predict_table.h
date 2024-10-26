#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include "eigen3/Eigen/Core"

using namespace std;
using namespace Eigen;

struct ClampingArmData {
    vector<double> currentData;
    double weight;
};

struct TrapzAreas {
    double veicleWeight;
    double veicleWeightCurrentAreas;
};

class Linear_regression{
    public:
        Linear_regression(){};
        ~Linear_regression(){};

        void readCSV(const string& filename, vector<ClampingArmData>& data);

        double trapezoidalIntegration(const vector<double>& y, double dx);

        void calculateTrapzAreas(const vector<ClampingArmData>& data, vector<TrapzAreas>& currentAreas);

        VectorXd trainModel(const vector<double>& weights, const vector<double>& areas);

        void saveWeights(const VectorXd& w, const string& filename);

        VectorXd loadWeights(const string& filename);
    private:
};

