#include "twiddle.h"
#include "PID.h"
#include <cmath>

using namespace std;

/*
 * TODO: Complete the PID class.
 */
bool Twiddle::m_ready = false;
bool Twiddle::m_processed = false;
bool Twiddle::m_reset_sim = false;

Twiddle::Twiddle() {

	m_cte =0;
	m_steer_value = 0;

}

Twiddle::~Twiddle() {

}
void Twiddle::run() {
	cout << "start fine tuning PID gains" << endl;
	double tol=0.002;
	double p[3] = {0, 0, 0};
	double dp[3] = {1, 1, 1};
	int it = 0;

	double best_err = run_twiddle_iteration(p[0],p[1], p[2]);

	double sum_dp = dp[0] + dp[1] + dp[2];
	while (sum_dp > tol){
		cout <<"cycle "<< it <<", best error = "<<best_err<<endl;
		for(int i = 0; i< 3;i++){
			p[i] += dp[i];
			double err = run_twiddle_iteration(p[0],p[1], p[2]);
			if (err < best_err){
				best_err = err;
				dp[i] *= 1.1;
				cout <<"best err "<< best_err <<" kp, "<<p[0]<<"ki, "<<p[1]<<"kd, "<<p[2]<<endl;
				continue;
			}
			p[i] -= 2 * dp[i];
			err = run_twiddle_iteration(p[0],p[1], p[2]);
			if (err < best_err){
				best_err = err;
				dp[i] *= 1.1;
				cout <<"best err "<< best_err <<" kp, "<<p[0]<<"ki, "<<p[1]<<"kd, "<<p[2]<<endl;
				continue;
			}
			p[i] += dp[i];
			dp[i] *= 0.9;
		}
		it += 1;
		sum_dp = dp[0] + dp[1] + dp[2];
	}
	cout << "#######Best Param: "<<"kp, "<<p[0]<<"ki, "<<p[1]<<"kd, "<<p[2] <<",Best err: "<<best_err<< endl;

}



double Twiddle::process_cte(double cte, double &reset_sim){
	std::cout << "ready "<<cte << endl;

	{
		std::lock_guard<std::mutex> lk(m_m);
		m_cte = cte;
		m_ready = true;
		m_processed = false;
	}
	m_cv.notify_one();
	// wait for the worker
	{
		std::unique_lock<std::mutex> lk(m_m);
		m_cv.wait(lk, []{return m_processed;});
	}
	reset_sim = m_reset_sim;
//	std::cout << "processed  " << m_steer_value <<endl;
	return m_steer_value;
}

double Twiddle::run_twiddle_iteration(double kp,double ki, double kd){

	PID pid = PID();
	pid.Init(kp,ki,kd);
	int count = 6000;
	cout<<"#######run_twiddle_iteration start "<<kp<<"," << ki<<"," <<kd<<endl;
	for(int i=1; i<= count; i++){
		// Wait until main() sends data
		std::unique_lock<std::mutex> lk(m_m);
		m_cv.wait(lk, []{return m_ready;});

//		cout<<"twiddle iteration "<< i<< " out of "<< count <<endl;
		pid.UpdateError(m_cte);
		m_steer_value = pid.m_steer_value;

		// Send data back to main()
		if(i==1){
			Twiddle::m_reset_sim = false;
		}
		if(i == count){
			Twiddle::m_reset_sim = true;
		}
		bool exit_this_param = false;
		if(abs(m_cte)> 2.0){
			//car already crashed, no need to further collect its total error
			Twiddle::m_reset_sim = true;
			exit_this_param = true;
			pid.m_total_err = pid.m_total_err + (count - i)*9.0;
		}
		m_processed = true;
		m_ready = false;
//		cout<<"twiddle iteration end "<< count <<endl;

		// Manual unlocking is done before notifying, to avoid waking up
		// the waiting thread only to block again (see notify_one for details)
		lk.unlock();
		m_cv.notify_one();

		if(exit_this_param){
			break;
		}
	}
	std::cout << "########run_twiddle_iteration end " << pid.m_total_err << std::endl;
	return pid.m_total_err;

}
