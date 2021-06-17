/* 
* CPid2.h
*
* Created: 26/07/2020 13:49:08
* Author: philg
*/


#ifndef __CPID2_H__
#define __CPID2_H__

#define AC_PID_TFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PID_EFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PID_DFILT_HZ_DEFAULT  20.0f   // default input filter frequency

class CPid_PID
{
//variables
public:
	float m_Output;
 	float m_DebugValue;
protected:
    // parameters
    float m_Kp;
    float m_Ki;
    float m_Kd;
    float m_Kff;
    float m_KiMax;
    float m_Filt_T_Hz;         // PID target filter frequency in Hz
    float m_Filt_E_Hz;         // PID error filter frequency in Hz
    float Filt_D_Hz;         // PID derivative filter frequency in Hz

    // internal variables
    float m_dt;                // timestep in seconds
    float m_Integrator;        // integrator value
    float m_Error;             // error value to enable filtering
   float m_Target;            // target value to enable filtering
    float m_Derivative;        // derivative value to enable filtering
	float _steer_accel_max;
private:
	uint32_t m_LastUpdateTime =0;
//functions
public:
	CPid_PID();
	CPid_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz, float dt);
	~CPid_PID();

	float GetError();
	void GetValues(float &Error,float &Target, float &Meas);
   // set_dt - set time step in seconds
   void set_dt(float dt);

   //  update_all - set target and measured inputs to PID controller and calculate outputs  target and error are filtered
   //  the derivative is then calculated and filtered  the integral is then updated based on the setting of the limit flag
	float update_all(float target, float measurement, bool limit);
   
   //  update_error - set error input to PID controller and calculate outputs target is set to zero and error is set and filtered  the derivative then is calculated and filtered
   //  the integral is then updated based on the setting of the limit flag  Target and Measured must be set manually for logging purposes.
   float update_error(float error, bool limit = false);

   //  update_i - update the integral
   //  if the limit flag is set the integral is only allowed to shrink
   void UpdateI(bool limit);

	void ResetAll();
	void SetValues( float Kp,float Ki,float Kd);
   // get_pid - get results from pid controller
   float get_pid() const;
   float get_pi() const;
   float GetP() const;
   float GetI() const;
   float GetD() const;
   float GetFF();


   // reset_I - reset the integrator
   void ResetI();

   // reset_filter - input filter will be reset to the next value provided to set_input()
   void ResetFilter() 
   {
	   m_Flags.ResetFilter = true;
   }



   /// operator function call for easy initialisation
   void operator()(float p_val, float i_val, float d_val,float initial_ff, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dt);

   // get accessors
   float &kP() { return m_Kp; }
   float &kI() { return m_Ki; }
   float &kD() { return m_Kd; }
 //  float &ff() { return m_Kff;}
   float &Filt_T_hz() { return m_Filt_T_Hz; }
   float &Filt_E_hz() { return m_Filt_E_Hz; }
   float &Filt_D_hz() { return Filt_D_Hz; }
   float imax() const { return m_KiMax; }
   float GetFiltAlpha(float filt_hz) const;
	float GetFilt_T_Alpha() const;
   float GetFilt_E_Alpha() const;
   float GetFilt_D_Alpha() const;

   // set accessors
   void kP(const float v) { m_Kp = v; }
   void kI(const float v) { m_Ki = v; }
   void kD(const float v) { m_Kd = v; }
 //  void ff(const float v) { m_Kff = v; }
   void imax(const float v) { m_KiMax = fabsf(v); }
   void Filt_T_hz(const float v);
   void Filt_E_hz(const float v);
   void Filt_D_hz(const float v);

//    // set the desired and actual rates (for logging purposes)
//    void set_target_rate(float target) { _pid_info.target = target; }
//    void set_actual_rate(float actual) { _pid_info.actual = actual; }

   // integrator setting functions
   void SetIntegrator(float target, float measurement, float i);
   void SetIntegrator(float error, float i);
   void SetIntegrator(float i);

//   const AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

   // parameter var table
 //  static const struct AP_Param::GroupInfo var_info[];

	CPid_PID( const CPid_PID &c );
protected:

    // flags
    struct pid_flags 
	{
	    bool ResetFilter :1; // true when input filter should be reset during next call to set_input
    } m_Flags;



private:
	CPid_PID& operator=( const CPid_PID &c );

}; //CPid2

#endif //__CPID2_H__
