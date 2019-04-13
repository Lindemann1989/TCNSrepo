#ifndef BC_HPP
#define BC_HPP

#include <math.h>
#include <armadillo> 
#include <string>
#include <stdexcept>
#include <algorithm>
#include "formula_parser.hpp"


class BC{
    std::vector<double> X_std;

    int robot_id;
    double W;
	double L;
	double t_init;

    arma::vec u_max;

    std::vector<std::string> s0barrier;
    std::vector<std::string> s0dbarrier_t;
    std::vector<std::vector<std::string>> s0dbarrier_x;
    std::vector<std::string> s1barrier;
    std::vector<std::string> s1dbarrier_t;
    std::vector<std::vector<std::string>> s1dbarrier_x;
    std::vector<std::string> s2barrier;
    std::vector<std::string> s2dbarrier_t;
    std::vector<std::vector<std::string>> s2dbarrier_x;
    std::vector<std::string> s3barrier;
    std::vector<std::string> s3dbarrier_t;
    std::vector<std::vector<std::string>> s3dbarrier_x;
    std::vector<std::string> s4barrier;
    std::vector<std::string> s4dbarrier_t;
    std::vector<std::vector<std::string>> s4dbarrier_x;
    std::vector<std::string> s5barrier;
    std::vector<std::string> s5dbarrier_t;
    std::vector<std::vector<std::string>> s5dbarrier_x;

    std::vector<int> V;
    std::vector<int> sequence;
    
    int sec_counter;
    
    
    // Formulas parsers
    FormulaParser<double> bf_;
    std::vector<FormulaParser<double>> bf_x_;
    std::vector<FormulaParser<double>> bf_x_all_;
    FormulaParser<double> bf_t_;
    

public:
    BC(int robot_id,
    	double W,
    	double L,
        std::vector<std::string> s0barrier, 
        std::vector<std::string> s0dbarrier_t, 
        std::vector<std::vector<std::string>> s0dbarrier_x,
        std::vector<std::string> s1barrier, 
        std::vector<std::string> s1dbarrier_t, 
        std::vector<std::vector<std::string>> s1dbarrier_x,
        std::vector<std::string> s2barrier, 
        std::vector<std::string> s2dbarrier_t, 
        std::vector<std::vector<std::string>> s2dbarrier_x,
        std::vector<std::string> s3barrier, 
        std::vector<std::string> s3dbarrier_t, 
        std::vector<std::vector<std::string>> s3dbarrier_x,
        std::vector<std::string> s4barrier, 
        std::vector<std::string> s4dbarrier_t, 
        std::vector<std::vector<std::string>> s4dbarrier_x,
        std::vector<std::string> s5barrier, 
        std::vector<std::string> s5dbarrier_t, 
        std::vector<std::vector<std::string>> s5dbarrier_x,
        arma::vec u_max,
        std::vector<int> V,
        std::vector<int> sequence);

    void init(double t_0, arma::vec x, arma::vec X);

    arma::vec u(std::vector<double> X, std::vector<double> x, double t);

    
    double bf(std::vector<double> X);
    double t(double t);
    
    

private:
    arma::mat g(arma::vec x);
    arma::vec f(arma::vec x);
	arma::mat Q(arma::vec x);
	arma::vec u1_hist;
	arma::vec u2_hist;

    double gamma(double t);
    
    // Output values, function to be used
    arma::vec bf_x(std::vector<double> X, int n);
    arma::vec bf_x_all(std::vector<double> X, int n);
    double bf_t(std::vector<double> X);
    
    void setFormulaParsers(int robot_id, int n, int t_frac);
    
    void setQPParams();

	void rotate_buffer(float u1, float u2);
};
    
#endif   
