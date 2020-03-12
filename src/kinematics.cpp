#include "haptikfabrikenapi.h"
#include <string>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>

#ifdef SUPPORT_POLHEMV2
#include "src/polhem.h"
#endif


#ifdef UNIX
//namespace unix {
    // Following includes are only used for reading/writing config file and to find
    // the user's home directory (where the config file will be stored)
    #include <unistd.h>
    #include <sys/types.h>
    #include <pwd.h>
//}
#endif

constexpr double pi_d{3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647};
constexpr long double pi{3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647L};

namespace haptikfabriken {

void fsRot::identity() {
    double a[3][3] = { {1, 0, 0 },
                       {0, 1, 0 },
                       {0, 0, 1 }};
    set(a);
}
void fsRot::rot_x(double t){
    double a[3][3] = { {1,   0,       0    },
                       {0, cos(t), -sin(t) },
                       {0, sin(t), cos(t)  }};
    set(a);
}
void fsRot::rot_y(double t){
    double a[3][3] = { {cos(t),  0, sin(t) },
                       {   0,    1,   0    },
                       {-sin(t), 0, cos(t) }};
    set(a);
}
void fsRot::rot_z(double t){
    double a[3][3] = { {cos(t), -sin(t), 0 },
                       {sin(t), cos(t), 0 },
                       {0, 0, 1 }};
    set(a);
}

fsRot fsRot::transpose()
{
    double a[3][3] = { {m[0][0], m[1][0], m[2][0] },
                       {m[0][1], m[1][1], m[2][1] },
                       {m[0][2], m[1][2], m[2][2] }};
    fsRot r;
    r.set(a);
    return r;
}

fsVec3d operator*(const fsRot &m, const fsVec3d &v)
{
    fsVec3d r;

    r.m_x = m.m[0][0]*v.m_x + m.m[0][1]*v.m_y + m.m[0][2]*v.m_z;
    r.m_y = m.m[1][0]*v.m_x + m.m[1][1]*v.m_y + m.m[1][2]*v.m_z;
    r.m_z = m.m[2][0]*v.m_x + m.m[2][1]*v.m_y + m.m[2][2]*v.m_z;

    return r;
}


//==============================================================================
// Helper functions for getPosition & setForce
//==============================================================================
struct pose {
    double Ln;
    double Lb;
    double Lc;
    double tA;  // angle of body A (theta_A)
    double tB;  // angle of body B (theta_B)
    double tC;  // angle of body C (theta_C)
};

pose calculate_pose(const Kinematics::configuration& c, const int* encoder_values) {
    pose p;
    double motorAngle[] = { (2.0*pi_d*double(encoder_values[0])/c.cpr_encoder_a),
                            (2.0*pi_d*double(encoder_values[1])/c.cpr_encoder_b),
                            (2.0*pi_d*double(encoder_values[2])/c.cpr_encoder_c)};

    //std::cout << encoder_values[0] << "  cpr: "  << c.cpr_encoder_a << "  gives motor angle: " << motorAngle[0] << "\n";


    // Calculate dof angles (theta) for each body
    p.Ln = c.length_body_a;
    p.Lb = c.length_body_b;
    p.Lc = c.length_body_c;

    // Different directions of motor on different devices
    double dir[] = {bool(c.motor_and_body_aligned_a)?-1.0:1.0,
                    bool(c.motor_and_body_aligned_b)?-1.0:1.0,
                    bool(c.motor_and_body_aligned_c)?-1.0:1.0};

    /*
    if(c.variant == 3 ) { // POLHEM v2
        p.tA =   motorAngle[0] * c.diameter_capstan_a / c.diameter_body_a;
        p.tB =  -motorAngle[1] * c.diameter_capstan_b / c.diameter_body_b;
        p.tC =   motorAngle[2] * c.diameter_capstan_c / c.diameter_body_c;
    } else {
        p.tA = -motorAngle[0] * c.diameter_capstan_a / c.diameter_body_a;
        p.tB = -motorAngle[1] * c.diameter_capstan_b / c.diameter_body_b;
        p.tC = -motorAngle[2] * c.diameter_capstan_c / c.diameter_body_c;
    }
    */
    p.tA = dir[0] * motorAngle[0] * c.diameter_capstan_a / c.diameter_body_a;
    p.tB = dir[1] * motorAngle[1] * c.diameter_capstan_b / c.diameter_body_b;
    p.tC = dir[2] * motorAngle[2] * c.diameter_capstan_c / c.diameter_body_c;
    //std::cout << "capstan: " << c.diameter_capstan_a << "  body: "  << c.diameter_body_a << "  gives ta: " << p.tA << "\n";

    return p;
}

//==============================================================================
// WoodenHaptics configuration helper files.
//==============================================================================



double v(const std::string& json, const std::string& key){
    size_t p = json.find(":", json.find(key));
    size_t p2 = json.find(":", p+1);

    std::string value = p2!=std::string::npos? json.substr(p+1,p2-p):json.substr(p+1);

    if(value.find("true")!=std::string::npos) return 1;
    if(value.find("false")!=std::string::npos) return 0;

    return atof(value.c_str());
}

Kinematics::configuration fromJSON(std::string json){
    double d[]= {
        v(json,"variant"),
        v(json,"diameter_capstan_a"),
        v(json,"diameter_capstan_b"),
        v(json,"diameter_capstan_c"),
        v(json,"length_body_a"),
        v(json,"length_body_b"),
        v(json,"length_body_c"),
        v(json,"diameter_body_a"),
        v(json,"diameter_body_b"),
        v(json,"diameter_body_c"),
        v(json,"workspace_origin_x"),
        v(json,"workspace_origin_y"),
        v(json,"workspace_origin_z"),
        v(json,"workspace_radius"),
        v(json,"torque_constant_motor_a"),
        v(json,"torque_constant_motor_b"),
        v(json,"torque_constant_motor_c"),
        v(json,"current_for_10_v_signal"),
        v(json,"cpr_encoder_a"),
        v(json,"cpr_encoder_b"),
        v(json,"cpr_encoder_c"),
        v(json,"cpr_encoder_d"),
        v(json,"cpr_encoder_e"),
        v(json,"cpr_encoder_f"),
        v(json,"max_linear_force"),
        v(json,"max_linear_stiffness"),
        v(json,"max_linear_damping"),
        v(json,"mass_body_b"),
        v(json,"mass_body_c"),
        v(json,"length_cm_body_b"),
        v(json,"length_cm_body_c"),
        v(json,"g_constant"),
        v(json,"calibrate_enc_a"),
        v(json,"calibrate_enc_b"),
        v(json,"calibrate_enc_c"),
        v(json,"calibrate_enc_d"),
        v(json,"calibrate_enc_e"),
        v(json,"calibrate_enc_f"),
        v(json,"motor_and_body_aligned_a"),
        v(json,"motor_and_body_aligned_b"),
        v(json,"motor_and_body_aligned_c"),
        v(json,"enc_and_body_aligned_d"),
        v(json,"enc_and_body_aligned_e"),
        v(json,"enc_and_body_aligned_f")
    };
    return Kinematics::configuration(d);
}

std::string j(const std::string& key, const double& value, bool last=false){
   std::stringstream s;
   s << "    \"" << key << "\":";
   while(s.str().length()<32) s<< " ";

   if(key.find("_aligned_") != std::string::npos)
       s << (bool(value)? "true" : "false");
   else
       s << value;

   if(!last) s << ",";
   s << std::endl;
   return s.str();
}
std::string toJSON(const Kinematics::configuration& c){
   using namespace std;
   stringstream json;
   json << "{" << endl
        << j("variant",c.variant)
        << j("diameter_capstan_a",c.diameter_capstan_a)
        << j("diameter_capstan_b",c.diameter_capstan_b)
        << j("diameter_capstan_c",c.diameter_capstan_c)
        << j("length_body_a",c.length_body_a)
        << j("length_body_b",c.length_body_b)
        << j("length_body_c",c.length_body_c)
        << j("diameter_body_a",c.diameter_body_a)
        << j("diameter_body_b",c.diameter_body_b)
        << j("diameter_body_c",c.diameter_body_c)
        << j("workspace_origin_x",c.workspace_origin_x)
        << j("workspace_origin_y",c.workspace_origin_y)
        << j("workspace_origin_z",c.workspace_origin_z)
        << j("workspace_radius",c.workspace_radius)
        << j("torque_constant_motor_a",c.torque_constant_motor_a)
        << j("torque_constant_motor_b",c.torque_constant_motor_b)
        << j("torque_constant_motor_c",c.torque_constant_motor_c)
        << j("current_for_10_v_signal",c.current_for_10_v_signal)
        << j("cpr_encoder_a",c.cpr_encoder_a)
        << j("cpr_encoder_b",c.cpr_encoder_b)
        << j("cpr_encoder_c",c.cpr_encoder_c)
        << j("cpr_encoder_d",c.cpr_encoder_d)
        << j("cpr_encoder_e",c.cpr_encoder_e)
        << j("cpr_encoder_f",c.cpr_encoder_f)
        << j("max_linear_force",c.max_linear_force)
        << j("max_linear_stiffness",c.max_linear_stiffness)
        << j("max_linear_damping",c.max_linear_damping)
        << j("mass_body_b",c.mass_body_b)
        << j("mass_body_c",c.mass_body_c)
        << j("length_cm_body_b",c.length_cm_body_b)
        << j("length_cm_body_c",c.length_cm_body_c)
        << j("g_constant", c.g_constant)
        << j("calibrate_enc_a", c.calibrate_enc_a)
        << j("calibrate_enc_b", c.calibrate_enc_b)
        << j("calibrate_enc_c", c.calibrate_enc_c)
        << j("calibrate_enc_d", c.calibrate_enc_d)
        << j("calibrate_enc_e", c.calibrate_enc_e)
        << j("calibrate_enc_f", c.calibrate_enc_f)
        << j("motor_and_body_aligned_a", c.motor_and_body_aligned_a)
        << j("motor_and_body_aligned_b", c.motor_and_body_aligned_b)
        << j("motor_and_body_aligned_c", c.motor_and_body_aligned_c)
        << j("enc_and_body_aligned_d", c.enc_and_body_aligned_d)
        << j("enc_and_body_aligned_e", c.enc_and_body_aligned_e)
        << j("enc_and_body_aligned_f", c.enc_and_body_aligned_f, true)
        << "}" << endl;
   return json.str();
}

void write_config_file(const Kinematics::configuration& config){
#ifdef UNIX
//    using namespace unix;
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    std::cout << "Writing configuration to: "<< homedir
              << "/woodenhaptics.json" << std::endl;
    std::ofstream ofile;
    ofile.open(std::string(homedir) + "/woodenhaptics.json");
    ofile << toJSON(config);
    ofile.close();
#else
    std::cout << toJSON(config) << "\n";
#endif
}

Kinematics::configuration read_config_file(){
#ifdef UNIX
//    using namespace unix;
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    std::cout << "Trying loading configuration from: "<< homedir
              << "/woodenhaptics.json" << std::endl;

    std::ifstream ifile;
    ifile.open(std::string(homedir) + "/woodenhaptics.json");
    if(ifile.is_open()){
        std::stringstream buffer;
        buffer << ifile.rdbuf();
        ifile.close();
        std::cout << "Success. " << std::endl;
        return fromJSON(buffer.str());
    } else {
        std::cout << "File not found. We will write one "
                  << "based on default configuration values." << std::endl;

        write_config_file(Kinematics::configuration::default_woody());
        return Kinematics::configuration::default_woody();
    }
#else
    write_config_file(Kinematics::configuration::default_woody());
    return Kinematics::configuration::default_woody();

#endif
}
//==============================================================================



Kinematics::Kinematics():m_config(read_config_file())
{

}

fsVec3d Kinematics::computeBodyAngles(const int *encoderValues)
{
    int e[3] = {encoderValues[0],encoderValues[1],encoderValues[2]};

    pose p  = calculate_pose(m_config, e);
    double third_axis = p.tC;

#ifdef SUPPORT_POLHEMV2
    if(m_config.variant==3)
        third_axis=polhemComputeLambda(p.tB,p.tC);
#endif

    return fsVec3d(p.tA, p.tB, third_axis);
}

// ------------------------------------------------------------------------
// Billy notation
// ------------------------------------------------------------------------
double Cos(double t) { return cos(t); }
double Power(double n, int m) { return pow(n,m); }
double ArcCos(double t) { return acos(t); }
double Sin(double t) { return sin(t); }
double Sqrt(double n) { return sqrt(n); }
// ------------------------------------------------------------------------



double power(double d, int n){
    if(n!=2) exit(1);
    return d*d;
}


fsVec3d Kinematics::computePosition(const int *encoderValues)
{
    int e[3] = {encoderValues[0],encoderValues[1],encoderValues[2]};
    pose p  = calculate_pose(m_config, e);

    const double& Ln = p.Ln;
    const double& Lb = p.Lb;
    const double& Lc = p.Lc;
    const double& tA = p.tA;
    const double& tB = p.tB;
    const double& tC = p.tC;
    double x,y,z;

    if(m_config.variant == 3){ // Polhem ver. 2
        long double r[3] = {0,0,0};
#ifdef SUPPORT_POLHEMV2
        polhemKinematics(tA,tB,tC,r);
#endif
        x = (std::isnan(r[0])?0:r[0]) - m_config.workspace_origin_x;
        y = (std::isnan(r[1])?0:r[1]) - m_config.workspace_origin_y;
        z = (std::isnan(r[2])?0:r[2]) - m_config.workspace_origin_z;
    } else { // All other devices have straight forward kinematics
        x = cos(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_x;
        y = sin(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_y;
        z = Ln+Lb*cos(tB)-Lc*cos(tC)        - m_config.workspace_origin_z;
    }

    return fsVec3d(x,y,z);
}




fsVec3d Kinematics::computeMotorAmps(fsVec3d force, const int *encoderValues)
{
    int e[3] = {encoderValues[0],encoderValues[1],encoderValues[2]};

    const pose p = calculate_pose(m_config, e);

    const double& Lb = p.Lb;
    const double& Lc = p.Lc;
    const double& tA = p.tA;
    const double& tB = p.tB;
    const double& tC = p.tC;

    //std::cout << 180*tB/3.141592 << "   " << 180*tC/3.141592 <<"\n";


    double fx=force.x();
    double fy=force.y();
    double fz=force.z();

    double tx,ty,tz;
    fsVec3d t;

    if(m_config.variant == 3){ // Polhem v.2
#ifdef SUPPORT_POLHEMV2
        t = polhemComputeMotorAmps(force, tA, tB, tC, debugJacobians, debugTorques);
#else
        std::cout << "Haptikfabriken API compiled without Polhem haptic device support. "
                  << "Edit haptikfabrikenapi.pro and run qmake to enable."
                  << "Contact Jonas Forsslund (jonas@forsslundsystems.com) for latest verison of polhem.h\n";
#endif
    } else {
        // All other devices uses same kinematic structure
        tx = -sin(tA)*(Lb*sin(tB)+Lc*sin(tC))*fx +  cos(tA)*(Lb*sin(tB)+Lc*sin(tC))*fy + 0*fz;
        ty = Lb*cos(tA)*cos(tB)*fx + Lb*sin(tA)*cos(tB)*fy -Lb*sin(tB)*fz;
        tz = Lc*cos(tA)*cos(tC)*fx + Lc*sin(tA)*cos(tC)*fy + Lc*sin(tC)*fz;
        t = fsVec3d(tx,ty,tz);

        // Gravity compensation
        const double& g=m_config.g_constant;
        const double& Lb_cm = m_config.length_cm_body_b;
        const double& Lc_cm = m_config.length_cm_body_c;
        const double& mB = m_config.mass_body_b;
        const double& mC = m_config.mass_body_c;

        t = t + g*fsVec3d( 0,
                            mB*Lb_cm*sin(tB) + mC*(Lb_cm + Lc_cm)*sin(tC),
                            mC*Lc_cm*sin(tC) );
    }




    double motorTorque[3];


    // Different directions of motor on different devices
    double dir[] = {m_config.motor_and_body_aligned_a?-1.0 : 1.0,
                    m_config.motor_and_body_aligned_b?-1.0 : 1.0,
                    m_config.motor_and_body_aligned_c?-1.0 : 1.0};

    motorTorque[0] =  t.m_x * dir[0] * m_config.diameter_capstan_a / m_config.diameter_body_a;
    motorTorque[1] =  t.m_y * dir[1] * m_config.diameter_capstan_b / m_config.diameter_body_b;
    motorTorque[2] =  t.m_z * dir[2] * m_config.diameter_capstan_c / m_config.diameter_body_c;


    // Set motor torque (t) through ampere
    double motorAmpere[] = { motorTorque[0] / m_config.torque_constant_motor_a,
                             motorTorque[1] / m_config.torque_constant_motor_b,
                             motorTorque[2] / m_config.torque_constant_motor_c };

    return fsVec3d(motorAmpere[0],motorAmpere[1],motorAmpere[2]);
}

//static int debugcount = 0;
fsRot Kinematics::computeRotation(const int* encBase, const int* encRot)
{
    fsRot r;
    r.identity();

    // If no encoders d-f, we have just a 3-dof device. return identity.
    if(m_config.cpr_encoder_d == 0 ||
       m_config.cpr_encoder_e == 0 ||
       m_config.cpr_encoder_f == 0)
        return r;

    // From compute pos -------------------
    pose p  = calculate_pose(m_config, encBase);

    const double& tA = p.tA;
    const double& tC = p.tC;
    double dir[] = { m_config.enc_and_body_aligned_d?1.0:-1.0,
                     m_config.enc_and_body_aligned_e?1.0:-1.0,
                     m_config.enc_and_body_aligned_f?1.0:-1.0 };
    double tD = dir[0] * encRot[0]*2*pi/m_config.cpr_encoder_d;
    double tE = dir[1] * encRot[1]*2*pi/m_config.cpr_encoder_e;
    double tF = dir[2] * encRot[2]*2*pi/m_config.cpr_encoder_f;


    // rotate about z (body a)
    fsRot rA;
    rA.rot_z(tA);

    // rotate about x
    fsRot rB;

    // rotate about y
    fsRot rC;
    if(m_config.variant == 3){ // Polhem v.2
#ifdef SUPPORT_POLHEMV2
        const double& tB = p.tB;
        double lambda = polhemComputeLambda(tB, tC);
        rC.rot_y(-lambda);
#endif
    } else {
        rC.rot_y(-tC+3.141592/2); // TODO: need verification in vintage.
    }

    // rotate about x
    fsRot rD;
    rD.rot_x(tD);

    // rotate about y
    fsRot rE;
    rE.rot_y(tE);

    // rotate about x
    fsRot rF;
    rF.rot_x(tF);

    r =  rA*rB*rC*rD*rE*rF;
    return r;
}



} // namespace haptikfabriken
