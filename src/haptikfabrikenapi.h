#ifndef HAPTIKFABRIKENAPI_H
#define HAPTIKFABRIKENAPI_H

/*
 *  Haptikfabriken API
 *  Methods to interact with our haptic devices
 *  e.g. Polhem and WoodenHaptics.
 *
 *  This library is licenced under LGPL (attribution and share-alike)
 *  This header file is licenced under BSD (do what you want)
 *  (C) Forsslund Systems AB 2019
 *
 */
#include <string>
#include <bitset>
#include <iostream>

namespace haptikfabriken {

struct fsVec3d {
    double m_x,m_y,m_z;
    fsVec3d(double x, double y, double z):m_x(x),m_y(y),m_z(z){}
    fsVec3d():m_x(0),m_y(0),m_z(0){}
    double x(){ return m_x; }
    double y(){ return m_y; }
    double z(){ return m_z; }
    void zero(){m_x=0;m_y=0;m_z=0;}
    double& operator[](int i) { if(i==0) return m_x; if(i==1) return m_y; return m_z;}
};
inline fsVec3d operator*(const double& d, const fsVec3d& v){ return fsVec3d(v.m_x*d,v.m_y*d,v.m_z*d); }
inline fsVec3d operator*(const fsVec3d& v, const double& d){ return fsVec3d(v.m_x*d,v.m_y*d,v.m_z*d); }
inline fsVec3d operator+(const fsVec3d& a, const fsVec3d& b){ return fsVec3d(a.m_x+b.m_x, a.m_y+b.m_y, a.m_z+b.m_z); }
inline fsVec3d operator-(const fsVec3d& a, const fsVec3d& b){ return fsVec3d(a.m_x-b.m_x, a.m_y-b.m_y, a.m_z-b.m_z); }

struct fsRot {
    double m[3][3];
    fsRot(){identity();}
    fsRot(double m[3][3]){set(m);}
    inline void set(double m[3][3]){
        for(int i=0;i<3;++i)
            for(int j=0;j<3;++j)
                this->m[i][j] = m[i][j];
    }
    fsRot(std::initializer_list<double> list){   // To initalize with list e.g.
        auto iter = list.begin();                //    fsRot r{1,2,3,
        for(int i=0;i<3;++i)                     //            4,5,6,
            for(int j=0;j<3;++j){                //            7,8,9};
                m[i][j]=*iter;
                iter++;
            }
    }
    void identity();
    void rot_x(double t);
    void rot_y(double t);
    void rot_z(double t);
    fsRot transpose();
};
fsVec3d operator*(const fsRot& m, const fsVec3d& v);
inline fsRot operator*(const fsRot& a, const fsRot& b) {
    fsRot c;
    int i,j,m;
    for(i=0;i<3;i++) {
      for(j=0;j<3;j++) {
        c.m[i][j] = 0;
        for(m=0;m<3;m++)
          c.m[i][j] += a.m[i][m]*b.m[m][j];
      }
    }
    return c;
}
inline fsRot operator*(const fsRot& a, const double& b) {
    fsRot c;
    int i,j;
    for(i=0;i<3;i++) {
      for(j=0;j<3;j++) {
         c.m[i][j] = a.m[i][j]*b;
      }
    }
    return c;
}
std::string toString(const fsVec3d& r);
std::string toString(const fsRot& r);


typedef fsRot fsMatrix3;

/*
static const char* json_polhem_v2 = R"({
    "variant":                  3,
    "diameter_capstan_a":       0.0288,
    "diameter_capstan_b":       0.0077,
    "diameter_capstan_c":       0.01,
    "length_body_a":            0.1,
    "length_body_b":            0.165,
    "length_body_c":            0.1308,
    "diameter_body_a":          0.175,
    "diameter_body_b":          0.1,
    "diameter_body_c":          0.1,
    "workspace_origin_x":       -0.137,
    "workspace_origin_y":       0.182,
    "workspace_origin_z":       -0.0217,
    "workspace_radius":         0.15,
    "torque_constant_motor_a":  0.321,
    "torque_constant_motor_b":  0.0538,
    "torque_constant_motor_c":  0.0538,
    "current_for_10_v_signal":  3,
    "cpr_encoder_a":            4096,
    "cpr_encoder_b":            4096,
    "cpr_encoder_c":            4096,
    "cpr_encoder_d":            1024,
    "cpr_encoder_e":            1024,
    "cpr_encoder_f":            1024,
    "max_linear_force":         5,
    "max_linear_stiffness":     2000,
    "max_linear_damping":       1,
    "mass_body_b":              0,
    "mass_body_c":              0,
    "length_cm_body_b":         0,
    "length_cm_body_c":         0,
    "g_constant":               0,
    "calibrate_enc_a":          8327,
    "calibrate_enc_b":          -18447,
    "calibrate_enc_c":          27140,
    "calibrate_enc_d":          0,
    "calibrate_enc_e":          30,
    "calibrate_enc_f":          0,
    "motor_and_body_aligned_a": false,
    "motor_and_body_aligned_b": true,
    "motor_and_body_aligned_c": false,
    "enc_and_body_aligned_d":   false,
    "enc_and_body_aligned_e":   false,
    "enc_and_body_aligned_f":   false
})";

static const char* json_polhem_v1 = R"({
    "variant":                  2,
    "diameter_capstan_a":       0.01,
    "diameter_capstan_b":       0.01,
    "diameter_capstan_c":       0.01,
    "length_body_a":            0.058,
    "length_body_b":            0.174,
    "length_body_c":            0.133,
    "diameter_body_a":          0.18,
    "diameter_body_b":          0.1,
    "diameter_body_c":          0.1,
    "workspace_origin_x":       0.14,
    "workspace_origin_y":       0,
    "workspace_origin_z":       0.1,
    "workspace_radius":         0.1,
    "torque_constant_motor_a":  0.0538,
    "torque_constant_motor_b":  0.0538,
    "torque_constant_motor_c":  0.0538,
    "current_for_10_v_signal":  3,
    "cpr_encoder_a":            2000,
    "cpr_encoder_b":            2000,
    "cpr_encoder_c":            2000,
    "cpr_encoder_d":            1024,
    "cpr_encoder_e":            1024,
    "cpr_encoder_f":            1024,
    "max_linear_force":         5,
    "max_linear_stiffness":     800,
    "max_linear_damping":       8,
    "mass_body_b":              0.08,
    "mass_body_c":              0.08,
    "length_cm_body_b":         0.04,
    "length_cm_body_c":         0.07,
    "g_constant":               0,
    "calibrate_enc_a":          0,
    "calibrate_enc_b":          0,
    "calibrate_enc_c":          -5000,
    "calibrate_enc_d":          0,
    "calibrate_enc_e":          0,
    "calibrate_enc_f":          0,
    "motor_and_body_aligned_a": true,
    "motor_and_body_aligned_b": true,
    "motor_and_body_aligned_c": true,
    "enc_and_body_aligned_d":   false,
    "enc_and_body_aligned_e":   true,
    "enc_and_body_aligned_f":   false
})";
*/

class Kinematics
{
public:

    fsVec3d computePosition(int ch_a, int ch_b, int ch_c){ int enc[3] = {ch_a,ch_b,ch_c}; return computePosition(enc); }
    fsVec3d computePosition(const int* encoderValues); // encoders[3]
    fsVec3d computeBodyAngles(const int* encoderValues); // encoders[3]
    fsVec3d computeMotorAmps(fsVec3d force, const int* encoderValues);
    fsRot computeRotation(const int* encBase, const int* encRot);




    //! A collection of variables that can be set in ~/wooden_haptics.json
    struct configuration {
        double variant;                 // 0=WoodenHaptics default, 1=AluHaptics
        double diameter_capstan_a;      // m
        double diameter_capstan_b;      // m
        double diameter_capstan_c;      // m
        double length_body_a;           // m
        double length_body_b;           // m
        double length_body_c;           // m
        double diameter_body_a;         // m
        double diameter_body_b;         // m
        double diameter_body_c;         // m
        double workspace_origin_x;      // m
        double workspace_origin_y;      // m
        double workspace_origin_z;      // m
        double workspace_radius;        // m (for application information)
        double torque_constant_motor_a; // Nm/A
        double torque_constant_motor_b; // Nm/A
        double torque_constant_motor_c; // Nm/A
        double current_for_10_v_signal; // A
        double cpr_encoder_a;           // quadrupled counts per revolution
        double cpr_encoder_b;           // quadrupled counts per revolution
        double cpr_encoder_c;           // quadrupled counts per revolution
        double cpr_encoder_d;           // quadrupled counts per revolution
        double cpr_encoder_e;           // quadrupled counts per revolution
        double cpr_encoder_f;           // quadrupled counts per revolution
        double max_linear_force;        // N
        double max_linear_stiffness;    // N/m
        double max_linear_damping;      // N/(m/s)
        double mass_body_b;             // Kg
        double mass_body_c;             // Kg
        double length_cm_body_b;        // m     distance to center of mass
        double length_cm_body_c;        // m     from previous body
        double g_constant;              // m/s^2 usually 9.81 or 0 to
                                        //       disable gravity compensation
        double calibrate_enc_a;
        double calibrate_enc_b;
        double calibrate_enc_c;
        double calibrate_enc_d;
        double calibrate_enc_e;
        double calibrate_enc_f;

        // Look at each body and put your right hand fingers in the direction of
        // where its angle increases. Is the motor pointing in the
        // direction of your thumb?
        double motor_and_body_aligned_a;  // true or false (0 or 1)
        double motor_and_body_aligned_b;
        double motor_and_body_aligned_c;

        // Look at each body and put your right hand fingers in the direction
        // where its angle increases. Is the output from the
        // encoder increasing in the same direction?
        double enc_and_body_aligned_d;  // true or false (0 or 1)
        double enc_and_body_aligned_e;
        double enc_and_body_aligned_f;

        std::string name;

        // Set values
        configuration(const double* k, std::string name="unnamed variant"):
          variant(k[0]),
          diameter_capstan_a(k[1]), diameter_capstan_b(k[2]), diameter_capstan_c(k[3]),
          length_body_a(k[4]), length_body_b(k[5]), length_body_c(k[6]),
          diameter_body_a(k[7]), diameter_body_b(k[8]), diameter_body_c(k[9]),
          workspace_origin_x(k[10]), workspace_origin_y(k[11]), workspace_origin_z(k[12]),
          workspace_radius(k[13]), torque_constant_motor_a(k[14]),
          torque_constant_motor_b(k[15]), torque_constant_motor_c(k[16]),
          current_for_10_v_signal(k[17]), cpr_encoder_a(k[18]), cpr_encoder_b(k[19]),
          cpr_encoder_c(k[20]), cpr_encoder_d(k[21]), cpr_encoder_e(k[22]),
          cpr_encoder_f(k[23]),
          max_linear_force(k[24]), max_linear_stiffness(k[25]),
          max_linear_damping(k[26]), mass_body_b(k[27]), mass_body_c(k[28]),
          length_cm_body_b(k[29]), length_cm_body_c(k[30]), g_constant(k[31]),
          calibrate_enc_a(k[32]),calibrate_enc_b(k[33]),calibrate_enc_c(k[34]),
          calibrate_enc_d(k[35]),calibrate_enc_e(k[36]),calibrate_enc_f(k[37]),
          motor_and_body_aligned_a(k[38]), motor_and_body_aligned_b(k[39]),
          motor_and_body_aligned_c(k[40]), enc_and_body_aligned_d(k[41]),
          enc_and_body_aligned_e(k[42]),enc_and_body_aligned_f(k[43]),
          name(name){}

        configuration(){}




        static configuration default_woody(){
            double data[] = { 0, 0.010, 0.010, 0.010,
                              0.080, 0.205, 0.245,
                              0.160, 0.120, 0.120,
                              0.220, 0.000, 0.080, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,0,0,0,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,0,0,0,0,
                              0,1,0,0,0,0};
            return Kinematics::configuration(data);
        }

        // Some configurations (won't require to use the filesystem)
        static configuration woodenhaptics_v2019() {
            double data[] = { 0, 0.010, 0.010, 0.010,
                              0.080, 0.205, 0.245,
                              0.160, 0.120, 0.120,
                              0.220, 0.000, 0.080, 0.100,
                              0.321, 0.0603, 0.0603, 3.0, 2000, 2000, 2000,0,0,0,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,6000,0,0,0,
                              0,0,0,0,0,0};
            return Kinematics::configuration(data,"woodenhaptics_v2019 hardcoded");
        }

        // Some configurations (won't require to use the filesystem)
        static configuration woodenhaptics_v2015() {
            double data[] = { 0, 0.010, 0.010, 0.010,
                              0.080, 0.205, 0.245,
                              0.160, 0.120, 0.120,
                              0.220, 0.000, 0.080, 0.100,
                              0.0259, 0.0259, 0.0259, 6.0, 2000, 2000, 2000,0,0,0,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,6000,0,0,0,
                              0,0,0,0,0,0};
            return Kinematics::configuration(data,"woodenhaptics_v2015 hardcoded");
        }

        static configuration polhem_v1() {
            double data[] = { 2, 0.010, 0.010, 0.010,
                              0.058, 0.174, 0.133,
                              0.180, 0.100, 0.100,
                              0.140, 0.000, 0.100, 0.100,
                              0.0538, 0.0538, 0.0538, 3.0, 2000, 2000, 2000,1024,1024,1024,
                              5.0, 800.0, 8.0,
                              0.080, 0.080, 0.040, 0.070, 0,
                              0,0,-5000,0,0,0,
                              1,1,1,0,1,0};
            return Kinematics::configuration(data,"polhem_v1 hardcoded");
        }


        static configuration polhem_v2() {
            double data[] = { 3, 0.0288, 0.0130, 0.010, // 3, 0.0288,0.0077,0.010 originally
                              0.1, 0.165, 0.1308,
                              0.175, 0.100, 0.100,
//                              -0.137, 0.182, -0.0217, 0.150,
                              0.137, 0, 0, 0.150,
                              0.321, 0.0538, 0.0538, 3.0, 4096, 4096, 4096,1024,1024,1024,
                              5.0, 5000.0, 1.0,
                              0.0, 0.0, 0.0, 0.0, 0,
                              8327,-10926,27140,0,30,0, // second was -18477
                              0,1,0,0,0,0};
            return Kinematics::configuration(data,"polhem_v2 hardcoded");
        }

        static configuration polhem_v3() {
            double data[] = { 3, 0.0288, 0.0130, 0.014, // 0.010->0.014 2020-01-30
                              0.1, 0.165, 0.1308,
                              0.175, 0.100, 0.100,
                              0.2045, 0.0, 0.0325, 0.150, // 0.137 0 0 0.150
                              0.0603, 0.0538, 0.0538, 3.0, 4096, 4096, 4096,1024,1024,1024, // 0.321
                              5.0, 2000.0, 1.0,
                              0.0, 0.0, 0.0, 0.0, 0,
                              8327,-10926,19386,0,30,0, // thrid was 27140
                              0,1,0,0,0,0};
            return Kinematics::configuration(data,"polhem_v3 hardcoded");
        }

        static configuration aluhaptics_v2() {
            double data[] = { 1, 0.0138, 0.0098, 0.0098,
                              0.111, 0.140, 0.111,
                              0.116, 0.076, 0.076,
                              0.140, 0.000, 0.000, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,0,0,0,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,0,0,0,0,
                              0,0,0,0,0,0};
            return Kinematics::configuration(data,"aluhaptics_v2 hardcoded");
        }

        static configuration vintage() {
            double data[] = { 1, 0.013, 0.010, 0.010,
                              0.056, 0.138, 0.112,
                              0.117, 0.077, 0.077,
                              0.140, 0.000, 0.000, 0.100,
                              0.0163, 0.0163, 0.0163, 3.0, 4000, 4000, 4000,0,0,0,
                              5.0, 800.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0,
                              0,0,0,0,0,0,
                              0,0,0,0,1,1};
            return Kinematics::configuration(data, "vintage hardcoded");
        }
    };

    Kinematics();
    Kinematics(configuration c):m_config(c) {}

    fsVec3d debugTorques[5];
    fsMatrix3 debugJacobians[5];


    const configuration m_config;
};

std::string toJSON(const Kinematics::configuration& c);
Kinematics::configuration fromJSON(std::string json);

// Forward declare
class FsHapticDeviceThread;
class HaptikfabrikenInterface;

// Haptic values as passed to event handlers
struct HapticValues {
    fsVec3d position;
    fsMatrix3 orientation;
    fsVec3d currentForce;
    fsVec3d nextForce; // Will be output to device
};

/*
 *  Class for haptic event listener
 *  Example:
 *    class MyClass : public HapticListener {
 *           positionEvent(HapticValues hv){
 *               fsVec3d f = -100 * hv.position;
 *               hfab->setForce(f);
 *           }
 *
 *           MyClass(){
 *               HaptikfabrikenInterface* hfab = new HaptikfabrikenInterface();
 *               hfab->open();
 *               hfab->addEventListener(this);
 *           }
 *    }
 *
 */
class HapticListener {
public:
    virtual void positionEvent(HapticValues&) = 0;
    virtual ~HapticListener(){}
    int maxHapticListenerFrequency{15000};
};

class HaptikfabrikenInterface {
public:
    enum Protocol {DAQ,UDP,USB};

    static std::string serialport_name;
    static unsigned int findUSBSerialDevices();

    HaptikfabrikenInterface(Kinematics::configuration c=Kinematics::configuration::polhem_v3(),
                            Protocol protocol=DAQ);

    // Deprecated constructor. Signature kept for backwards compatability.
    HaptikfabrikenInterface(bool, Kinematics::configuration c=Kinematics::configuration::polhem_v3(),
                            Protocol protocol=DAQ);

    ~HaptikfabrikenInterface();


    int open();    // Returns 0 if success, otherwise a error number
    void close();

    // If error, get error message here
    std::string getErrorCode();


    // Get/set methods (thread safe, gets latest known values and sets next to be commanded)
    void getEnc(int a[]);  // Array of 6 elements will be filled with signed encoder values
    fsVec3d getBodyAngles();
    void getLatestCommandedMilliamps(int ma[]); // Array of 3 elements
    int getNumSentMessages(); // usb communications statistics
    int getNumReceivedMessages();

    fsVec3d getPos(bool blocking=false);              // Get cartesian coords xyz using kineamtics model
    fsRot getRot();                // Get orientaion of manipulandum
    fsVec3d getCurrentForce();              // Get last applied force
    void setForce(fsVec3d f);      // Set force using kineamtics model, e.g. in xyz
    void setCurrent(fsVec3d amps); // Set the 3 motor amps directly (not cartesian coords.)
    std::bitset<5> getSwitchesState();

    void calibrate();              // Sets encoders (or offset if read-only, e.g. usb)
                                   // to the defined values in kinematic model as home position.

    void addEventListener(HapticListener* listener);
    void removeEventListener(HapticListener* listener);

    const Kinematics::configuration kinematicModel;


    int max_milliamps = 2000;      // Cap the current provided to motors

private:
    FsHapticDeviceThread *fsthread;
};

} // namespace haptikfabriken

#endif // HAPTIKFABRIKENAPI_H
