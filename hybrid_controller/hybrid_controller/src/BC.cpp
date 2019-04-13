#include "BC.hpp"

using namespace std;
extern "C" {
#include "solver.h"
}

Vars vars;
Params params;
Workspace work;
Settings settings;

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define ABS(a) ((a)>0?(a):(-a))

BC::BC(int robot_id, 
	double W, double L,
    std::vector<std::string> s0barrier, std::vector<std::string> s0dbarrier_t, std::vector<std::vector<std::string>> s0dbarrier_x, 
    std::vector<std::string> s1barrier, std::vector<std::string> s1dbarrier_t, std::vector<std::vector<std::string>> s1dbarrier_x,
    std::vector<std::string> s2barrier, std::vector<std::string> s2dbarrier_t, std::vector<std::vector<std::string>> s2dbarrier_x,
    std::vector<std::string> s3barrier, std::vector<std::string> s3dbarrier_t, std::vector<std::vector<std::string>> s3dbarrier_x,
    std::vector<std::string> s4barrier, std::vector<std::string> s4dbarrier_t, std::vector<std::vector<std::string>> s4dbarrier_x,
    std::vector<std::string> s5barrier, std::vector<std::string> s5dbarrier_t, std::vector<std::vector<std::string>> s5dbarrier_x,
    arma::vec u_max, 
    std::vector<int> V,
    std::vector<int> sequence): 
        robot_id(robot_id), 
        W(W), L(L),
        s0barrier(s0barrier), s0dbarrier_t(s0dbarrier_t), s0dbarrier_x(s0dbarrier_x),
        s1barrier(s1barrier), s1dbarrier_t(s1dbarrier_t), s1dbarrier_x(s1dbarrier_x),
        s2barrier(s2barrier), s2dbarrier_t(s2dbarrier_t), s2dbarrier_x(s2dbarrier_x),
        s3barrier(s3barrier), s3dbarrier_t(s3dbarrier_t), s3dbarrier_x(s3dbarrier_x),
        s4barrier(s4barrier), s4dbarrier_t(s4dbarrier_t), s4dbarrier_x(s4dbarrier_x),
        s5barrier(s5barrier), s5dbarrier_t(s5dbarrier_t), s5dbarrier_x(s5dbarrier_x),
        bf_x_(std::vector<FormulaParser<double>>(3)), 
        bf_x_all_(std::vector<FormulaParser<double>>(6)),
        u_max(u_max), V(V), sequence(sequence){
        
        
}

void BC::init(double t_0, arma::vec x, arma::vec X){

	std::vector<double> X_inter = arma::conv_to<std::vector<double>>::from(X);
	X_inter.insert(X_inter.end(),0);
	this->X_std = X_inter;
	t_init = t_0;
	sec_counter = 1; // counter indicating to start with the first time interval
	
    //this->X_std = arma::conv_to<std::vector<double>>::from(X);
	
    setFormulaParsers(robot_id, x.n_elem,sec_counter-1);
    
    setQPParams();
    
    u1_hist.zeros(7);
    u2_hist.zeros(7);
    
}

arma::mat BC::g(arma::vec x){arma::vec u1_hist(10);
    return arma::eye<arma::mat>(x.n_elem, x.n_elem);
}

arma::mat BC::Q(arma::vec x){
    return arma::eye<arma::mat>(x.n_elem, x.n_elem);
}

arma::vec BC::f(arma::vec x){
    return arma::zeros<arma::vec>(x.n_elem);
}


double BC::gamma(double t){
	return 1;
}

arma::vec BC::u(std::vector<double> X, std::vector<double> x, double t){
	
	// Switch barriers if needed
	if((t-t_init)>=sequence[sec_counter] && sequence.size()>(sec_counter+1)){
		sec_counter += 1;
		setFormulaParsers(robot_id, x.size(), sec_counter-1);
	}
	
	// calculate control input
	arma::vec u_;
	if((t-t_init)<sequence[sequence.size()-1]){

	    int robots_in_cluster = V.size();
	    int n = x.size();
	    
		X.insert(X.end(),t-t_init); //add time as state
	    arma::vec dbx_ = bf_x(X, n);
	    for(int i=0; i<n; i++){
	    	params.dbx[i] = dbx_(i);
		}
		params.norm_dbx[0] = arma::norm(dbx_);
	  	
	  	arma::vec dbx_all = bf_x_all(X, n);
	  	arma::vec norm_agent(n);
	  	double sum_db = 0;
	  	for(int i=0; i<robots_in_cluster; i++){
	  		norm_agent.zeros();
	  		for(int j=0; j<n; j++){
	  			norm_agent[j] = dbx_all[i*n+j];
	  		}
	  		sum_db += arma::norm(norm_agent);
	  	}
		params.N[0] = arma::norm(dbx_)/sum_db;
	  	
	  	params.dbt[0] = bf_t(X);
	  	if (bf(X)>0){
	  		params.alpha[0] = 0.25*bf(X);
	  	}
	  	else {
	  		params.alpha[0] = 5*bf(X);
	  	}
	  	params.L[0] = L;
	  	params.W[0] = W;
	  			
	  	int num_iters;
	  	set_defaults();
	  	setup_indexing();
	  	settings.verbose = 0;
	  	num_iters = solve();
		
		u_.zeros(3);
		u_(0) = vars.u[0];
		u_(1) = vars.u[1];
		u_(2) = vars.u[2];
		
		if(arma::norm(u_(arma::span(0,1))) > u_max[0]){
				u_(arma::span(0,1)) = u_(arma::span(0,1))/arma::norm(u_)*u_max[0];
		}
		
		//rotate_buffer(u_(0),u_(1));	
		//u_(0) = arma::mean(u1_hist);
		//u_(1) = arma::mean(u2_hist);
	
		if (robot_id==0){
			//printf("R1 time %.3f, and bf: %.3f, ux:%.3f, dbx: %.8f, uy: %.3f, dby: %.8f, norm db_i: %.3f, norm db: %.3f, dbt: %.3f \n",X[9],bf(X),u_(0),params.dbx[0],u_(1),params.dbx[1],params.norm_dbx[0],params.N[0],params.dbt[0]);
		}
		
		if (robot_id==1){
			//u_.zeros(3);
			//printf("R1 time %.3f, and bf: %.3f, ux:%.3f, dbx: %.8f, uy: %.3f, dby: %.8f, norm db_i: %.3f, norm db: %.3f, dbt: %.3f \n",X[9],bf(X),u_(0),params.dbx[0],u_(1),params.dbx[1],params.norm_dbx[0],params.N[0],params.dbt[0]);
		}
		
		if (robot_id==2){
			//u_.zeros(3);
			//printf("R3 time %.3f, and bf: %.3f, ux:%.3f, dbx: %.8f, uy: %.3f, dby: %.8f, norm db_i: %.3f, norm db: %.3f, dbt: %.3f \n",X[9],bf(X),u_(0),params.dbx[0],u_(1),params.dbx[1],params.norm_dbx[0],params.N[0],params.dbt[0]);
		}
	}
	else{
		u_.zeros(3);
	}
		

	
    return u_;
}


double BC::bf(std::vector<double> X){
    return bf_.value(X);
}

double BC::t(double t){
    return t-t_init;
}

arma::vec BC::bf_x(std::vector<double> X, int n){
    arma::vec drho_(n);
    
    for(int i=0; i<n; i++){
        drho_(i) = bf_x_[i].value(X);
    }
    if(!drho_.is_finite()){
        drho_.zeros();
    }
    else{
        //drho_ /= arma::norm(drho_);  //why do we take the norm???
    }
    
    return drho_;
}

arma::vec BC::bf_x_all(std::vector<double> X, int n){
    int robots_in_cluster = V.size();
  	arma::vec drho_(n*robots_in_cluster);
  
    for(int i=0; i<n*robots_in_cluster; i++){
        drho_(i) = bf_x_all_[i].value(X);
    }

    if(!drho_.is_finite()){
        drho_.zeros();
    }
    else{
        //drho_ /= arma::norm(drho_);  //why do we take the norm???
    }

    return drho_;
}


double BC::bf_t(std::vector<double> X){
	return bf_t_.value(X);
}


void BC::setFormulaParsers(int robot_id, int n, int t_frac){

	int robots_in_cluster = V.size();
	
	if (t_frac==0){
		printf("Sequence 0 \n");
		bf_ = FormulaParser<double>(s0barrier[robot_id], "x", X_std);
	    
	    for(int i=0; i<n; i++){
	      	bf_x_[i] = FormulaParser<double>(s0dbarrier_x[robot_id][n*robot_id+i], "x", X_std);
	    }
	
	    for(int i=0; i<n*robots_in_cluster; i++){
	      	bf_x_all_[i] = FormulaParser<double>(s0dbarrier_x[robot_id][i], "x", X_std);
	    }
	
	    bf_t_ = FormulaParser<double>(s0dbarrier_t[robot_id], "x", X_std);
	}
	else if (t_frac==1){
		printf("Sequence 1 \n");
		bf_ = FormulaParser<double>(s1barrier[robot_id], "x", X_std);
	    
	    for(int i=0; i<n; i++){
	      	bf_x_[i] = FormulaParser<double>(s1dbarrier_x[robot_id][n*robot_id+i], "x", X_std);
	    }
	
	    for(int i=0; i<n*robots_in_cluster; i++){
	      	bf_x_all_[i] = FormulaParser<double>(s1dbarrier_x[robot_id][i], "x", X_std);
	    }
	
	    bf_t_ = FormulaParser<double>(s1dbarrier_t[robot_id], "x", X_std);
	}
	else if (t_frac==2){
		printf("Sequence 2 \n");
		bf_ = FormulaParser<double>(s2barrier[robot_id], "x", X_std);
	    
	    for(int i=0; i<n; i++){
	      	bf_x_[i] = FormulaParser<double>(s2dbarrier_x[robot_id][n*robot_id+i], "x", X_std);
	    }
	
	    for(int i=0; i<n*robots_in_cluster; i++){
	      	bf_x_all_[i] = FormulaParser<double>(s2dbarrier_x[robot_id][i], "x", X_std);
	    }
	
	    bf_t_ = FormulaParser<double>(s2dbarrier_t[robot_id], "x", X_std);
	}
    else if (t_frac==3){
    	printf("Sequence 3 \n");
		bf_ = FormulaParser<double>(s3barrier[robot_id], "x", X_std);
	    
	    for(int i=0; i<n; i++){
	      	bf_x_[i] = FormulaParser<double>(s3dbarrier_x[robot_id][n*robot_id+i], "x", X_std);
	    }
	
	    for(int i=0; i<n*robots_in_cluster; i++){
	      	bf_x_all_[i] = FormulaParser<double>(s3dbarrier_x[robot_id][i], "x", X_std);
	    }
	
	    bf_t_ = FormulaParser<double>(s3dbarrier_t[robot_id], "x", X_std);
	}
	else if (t_frac==4){
    	printf("Sequence 4 \n");
		bf_ = FormulaParser<double>(s4barrier[robot_id], "x", X_std);
	    
	    for(int i=0; i<n; i++){
	      	bf_x_[i] = FormulaParser<double>(s4dbarrier_x[robot_id][n*robot_id+i], "x", X_std);
	    }
	
	    for(int i=0; i<n*robots_in_cluster; i++){
	      	bf_x_all_[i] = FormulaParser<double>(s4dbarrier_x[robot_id][i], "x", X_std);
	    }
	
	    bf_t_ = FormulaParser<double>(s4dbarrier_t[robot_id], "x", X_std);
	}
	else if (t_frac==5){
    	printf("Sequence 5 \n");
		bf_ = FormulaParser<double>(s5barrier[robot_id], "x", X_std);
	    
	    for(int i=0; i<n; i++){
	      	bf_x_[i] = FormulaParser<double>(s5dbarrier_x[robot_id][n*robot_id+i], "x", X_std);
	    }
	
	    for(int i=0; i<n*robots_in_cluster; i++){
	      	bf_x_all_[i] = FormulaParser<double>(s5dbarrier_x[robot_id][i], "x", X_std);
	    }
	
	    bf_t_ = FormulaParser<double>(s5dbarrier_t[robot_id], "x", X_std);
	}
    
}

void BC::rotate_buffer(float u1, float u2){
	
	for (int i=0; i<6; i++){
		u1_hist(i) = 0.025*u1_hist(i+1);
		u2_hist(i) = 0.025*u2_hist(i+1);
	}

	u1_hist(6) = u1;
	u2_hist(6) = u2;

}




void BC::setQPParams(){
	// Solve this in a nicer way by using the explicit functions for f and g from above
	params.f[0] = 0;
  	params.f[1] = 0;
  	params.f[2] = 0;
  	params.Q[0] = 1;
  	params.Q[1] = 0;
  	params.Q[2] = 0;
  	params.Q[3] = 0;
  	params.Q[4] = 1;
  	params.Q[5] = 0;
  	params.Q[6] = 0;
  	params.Q[7] = 0;
  	params.Q[8] = 1;
  	params.g[0] = 1;
  	params.g[1] = 0;
  	params.g[2] = 0;
  	params.g[3] = 0;
  	params.g[4] = 1;
  	params.g[5] = 0;
  	params.g[6] = 0;
  	params.g[7] = 0;
  	params.g[8] = 1;
  	params.W[0] = 0;
  	params.L[0] = 0;
}