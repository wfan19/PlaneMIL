class PIDFF
{

public:

    PIDFF();
    ~PIDFF();

    // Initialize the PIDFF with coefficients and limits
    void PIDFF::init(double kp, double ki, double kd, double kff, double imin, double imax, double min, double max);

    // Update PID with new error and elapsed time
    double PIDFF::update(double target, double error, double dt);

    // Reset the integrator
    void PIDFF::resetIntegrator();

private:
    // PID params
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double kff{0.0};
    double imin{0.0};
    double imax{0.0};
    double min{0.0};
    double max{0.0};

    // Integral for past values
    double integrator{0.0};

    // Track last error for derivative term
    double lastError{0.0};

public:

    // Setters and getters
    void PIDFF::setKP(double kp){
        this->kp = kp;
    }

    void PIDFF::setKI(double ki){
        this->ki = ki;
    }

    void PIDFF::setKD(double kd){
        this->kd = kd;
    }

    void PIDFF::setKFF(double kff){
        this->kff = kff;
    }

    void PIDFF::setIMin(double imin){
        this->imin = imin;
    }

    void PIDFF::setIMax(double imax){
        this->imax = imax;
    }

    void PIDFF::setMin(double min){
        this->min = min;
    }

    void PIDFF::setMax(double max){
        this->max = max;
    }

    double PIDFF::getKP(){
        return kp;
    }

    double PIDFF::getKI(){
        return ki;
    }

    double PIDFF::getKD(){
        return kd;
    }

    double PIDFF::getKFF(){
        return kff;
    }

    double PIDFF::getIMin(){
        return imin;
    }

    double PIDFF::getIMax(){
        return imax;
    }

    double PIDFF::getMin(){
        return min;
    }

    double PIDFF::getMax(){
        return max;
    }
};