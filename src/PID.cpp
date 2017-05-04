#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	m_firstcte = true;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	m_Kp = Kp;
	m_Ki = Ki;
	m_Kd = Kd;

}

void PID::UpdateError(double cte) {
	if (m_firstcte){
		//if this is the first cte signal received
		m_prev_cte = cte;
		m_int_cte = 0;
		m_total_err = 0;
		m_cnt = 0;

		m_firstcte = false;
	}

	//calculate the steer value
	double diff_cte = cte - m_prev_cte;
	m_prev_cte = cte;
	m_int_cte += cte;
	m_steer_value = -m_Kp * cte - m_Kd * diff_cte - m_Ki * m_int_cte;
	m_cnt++;
	m_total_err += cte * cte;
//	if(m_cnt > 100){
//		m_total_err += cte * cte;
//	}
}

double PID::TotalError() {
	return m_total_err;
}

