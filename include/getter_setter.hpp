#ifndef GETTER_SETTER_HPP
#define GETTER_SETTER_HPP

/**
 * @class GetterSetter
 * @brief A generic class for managing getter and setter functions for PID parameters.
 */
class GetterSetter {
protected:
    double Kp, Ki, Kd, dt;

public:

    // Default constructor
    GetterSetter() : Kp(0), Ki(0), Kd(0), dt(0) {} 
    /**
     * @brief Getter for Proportional gain.
     * @return The Proportional gain (Kp).
     */
    double getKp() const { return Kp; }

    /**
     * @brief Getter for Integral gain.
     * @return The Integral gain (Ki).
     */
    double getKi() const { return Ki; }

    /**
     * @brief Getter for Derivative gain.
     * @return The Derivative gain (Kd).
     */
    double getKd() const { return Kd; }

    /**
     * @brief Getter for time step (dt).
     * @return The time step (dt).
     */
    double getdt() const { return dt; }

    /**
     * @brief Setter for Proportional gain.
     * @param Kp The new Proportional gain.
     */
    void setKp(double Kp) { this->Kp = Kp; }

    /**
     * @brief Setter for Integral gain.
     * @param Ki The new Integral gain.
     */
    void setKi(double Ki) { this->Ki = Ki; }

    /**
     * @brief Setter for Derivative gain.
     * @param Kd The new Derivative gain.
     */
    void setKd(double Kd) { this->Kd = Kd; }

    /**
     * @brief Setter for time step (dt).
     * @param dt The new time step.
     */
    void setdt(double dt) { this->dt = dt; }
};

#endif // GETTER_SETTER_HPP
