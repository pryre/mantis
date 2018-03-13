#include <pidController/pidController.h>

#include <math.h>
#include <assert.h>

pidController::pidController() :
	kp_( 0.0f ),
	ki_( 0.0f ),
	kd_( 0.0f ),
	tau_( 0.0f ),
	x_( 0.0f ),
	x_dot_( 0.0f ),
	sp_( 0.0f ),
	control_output_( 0.0f ) {

	setOutputMinMax( -1.0f, 1.0f );

	this->reset();
}

pidController::pidController( double initial_x, double initial_x_dot, double initial_setpoint, double initial_output ) :
	kp_( 0.0f ),
	ki_( 0.0f ),
	kd_( 0.0f ),
	tau_( 0.0f ),
	x_( initial_x ),
	x_dot_( initial_x_dot ),
	sp_( initial_setpoint ),
	control_output_( initial_output ) {

	setOutputMinMax( -1.0f, 1.0f );

	this->reset();
}

pidController::pidController( double initial_x, double initial_x_dot, double initial_setpoint, double initial_output, double Kp, double Ki, double Kd, double tau, double min_output, double max_output ) :
	kp_( Kp ),
	ki_( Ki ),
	kd_( Kd ),
	tau_( tau ),
	x_( initial_x ),
	x_dot_( initial_x_dot ),
	sp_( initial_setpoint ),
	control_output_( initial_output ) {

	setOutputMinMax( min_output, max_output );

	this->reset();
}

pidController::~pidController() {

}

void pidController::reset() {
	this->reset( x_ );
}

void pidController::reset( double x_prev ) {
	integrator_ = 0.0f;
	x_prev_ = x_prev;
}

void pidController::setKp( double Kp ) {
	kp_ = Kp;
}

void pidController::setKi( double Ki ) {
	ki_ = Ki;
}

void pidController::setKd( double Kd ) {
	kd_ = Kd;
}

void pidController::setTau( double tau ) {
	tau_ = tau;
}

void pidController::setGains( double Kp, double Ki, double Kd, double tau ) {
	this->setKp( Kp );
	this->setKi( Ki );
	this->setKd( Kd );
	this->setTau( tau );
}

void pidController::setOutputMinMax( double min, double max ) {
	assert( min < max );

	output_min_ = min;
	output_max_ = max;
}

//Calculate x_dot
double pidController::step( double dt, double sp, double x ) {
	//Check to make sure the controller hasn't gone stale
	if( dt > 1.0f ) {
		this->reset( x );
		dt = 0.0f;
	}

	//Calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
	//The dirty derivative is a sort of low-pass filtered version of the derivative.
	double x_dot = ( ( 2.0f * tau_ - dt ) / ( 2.0f * tau_ + dt ) * x_dot_ ) + ( 2.0f / ( 2.0f * tau_ + dt ) * ( x - x_prev_ ) );

	double output = this->step( dt, sp, x, x_dot );

	//Save last state
	x_prev_ = x;

	return output;
}

//Calculate control step with known x_dot
//	time_now: Current time in seconds
double pidController::step( double dt, double sp, double x, double x_dot ) {
	x_dot_ = x_dot;

	//Check to make sure the controller hasn't gone stale
	if( dt > 1.0f ) {
		this->reset( x );
		dt = 0.0f;
	}

	x_ = x;
	sp_ = sp;

	//Calculate error
	double error = sp_ - x_;

	//Initialize Terms
	double p_term = error * kp_;
	double i_term = 0.0f;
	double d_term = 0.0f;

	//If it is a stale controller, just skip this section
	if( dt > 0.0f ) {
		d_term = kd_ * x_dot_;

		//Integrate over dt
		integrator_ += error * dt;

		//Calculate I term
		i_term = ki_ * integrator_;
	}

	//Sum three terms: u = p_term + i_term - d_term
	double u = p_term + i_term - d_term;

	//Output Saturation
	double u_sat = ( u > output_max_ ) ? output_max_ : ( (u < output_min_ ) ? output_min_ : u );

	//Integrator anti-windup
	//If the pid controller has saturated and if the integrator is the cause
	if( ( u != u_sat ) && ( fabs( i_term ) > fabs( u - p_term - d_term ) ) )
			integrator_ = ( u_sat - p_term - d_term ) / ki_;	//Trim the integrator to what it should currently be to only just hit the maximum

	//Set output
	control_output_ = u_sat;

	return control_output_;
}

double pidController::getKp() {
	return kp_;
}

double pidController::getKi() {
	return ki_;
}

double pidController::getKd() {
	return kd_;
}

double pidController::getTau() {
	return tau_;
}

double pidController::getOutputMin() {
	return output_min_;
}

double pidController::getOutputMax() {
	return output_max_;
}

double pidController::getOutput() {
	return control_output_;
}
