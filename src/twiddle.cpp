#include "twiddle.h"
#include "PID.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */
bool Twiddle::m_ready = false;
bool Twiddle::m_processed = false;

Twiddle::Twiddle() {

	m_cte =0;
	m_steer_value = 0;

}

Twiddle::~Twiddle() {

}
void Twiddle::run() {
	cout << "start fine tuning PID gains" << endl;

}



double Twiddle::process_cte(double cte){

	{
		std::lock_guard<std::mutex> lk(m_m);
		m_cte = cte;
		m_ready = true;
		std::cout << "main() signals data ready for processing\n";
	}
	m_cv.notify_one();
	// wait for the worker
	{
		std::unique_lock<std::mutex> lk(m_m);
		m_cv.wait(lk, []{return m_processed;});
	}
	std::cout << "Back in main(), data = " << m_steer_value << '\n';
	return m_steer_value;
}

double Twiddle::run_twiddle_iteration(double kp,double ki, double kd){
	PID pid = PID();
	pid.Init(kp,ki,kd);
	for(int i=0; i< 1000; i++){
		// Wait until main() sends data
		std::unique_lock<std::mutex> lk(m_m);
		m_cv.wait(lk, []{return m_ready;});
		pid.UpdateError(m_cte);
		m_steer_value = pid.m_steer_value;

		// Send data back to main()
		m_processed = true;
		std::cout << "Worker thread signals data processing completed\n";

		// Manual unlocking is done before notifying, to avoid waking up
		// the waiting thread only to block again (see notify_one for details)
		lk.unlock();
		m_cv.notify_one();

	}
	return pid.m_total_err;

}
