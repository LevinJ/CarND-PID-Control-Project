#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

class Twiddle {
public:

	std::mutex m_m;
	std::condition_variable m_cv;
	static bool m_ready;
	static bool m_processed;
	static bool m_reset_sim;

	double m_cte;
	double m_steer_value;

	double m_max_step;
	/*
	 * Constructor
	 */
	Twiddle();

	/*
	 * Destructor.
	 */
	virtual ~Twiddle();
	void run();
	std::thread launch_twiddle() {

	    return std::thread(&Twiddle::run, this);

	};

	double process_cte(double cte, double &reset_sim);
	double run_twiddle_iteration(double kp,double ki, double kd);




};

#endif /* TWIDDLE_H */
