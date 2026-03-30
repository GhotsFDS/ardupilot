/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  very simple plane simulator class. Not aerodynamically accurate,
  just enough to be able to debug control logic for new frame types
*/

#include "SIM_Plane.h"

#include <stdio.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <AP_Filesystem/AP_Filesystem.h>

using namespace SITL;

Plane::Plane(const char *frame_str) :
    Aircraft(frame_str)
{

    const char *colon = strchr(frame_str, ':');
    size_t slen = strlen(frame_str);
    // The last 5 letters are ".json"
    if (colon != nullptr && slen > 5 && strcmp(&frame_str[slen-5], ".json") == 0) {
        load_coeffs(colon+1);
    } else {
        coefficient = default_coefficients;
    }

    mass = 2.0f;

    // apply mass override from JSON model if specified
    if (coefficient.mass_override > 0) {
        mass = coefficient.mass_override;
    }

    /*
       scaling from motor power to Newtons. Allows the plane to hold
       vertically against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;

    // apply max_thrust override from JSON (e.g. for rocket motors)
    if (coefficient.max_thrust > 0) {
        thrust_scale = coefficient.max_thrust;
    }
    frame_height = 0.1f;

    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    lock_step_scheduled = true;

    if (strstr(frame_str, "-heavy")) {
        mass = 8;
    }
    if (strstr(frame_str, "-jet")) {
        // a 22kg "jet", level top speed is 102m/s
        mass = 22;
        thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    }
    if (strstr(frame_str, "-revthrust")) {
        reverse_thrust = true;
    }
    if (strstr(frame_str, "-elevon")) {
        elevons = true;
    } else if (strstr(frame_str, "-vtail")) {
        vtail = true;
    } else if (strstr(frame_str, "-dspoilers")) {
        dspoilers = true;
    } else if (strstr(frame_str, "-redundant")) {
        redundant = true;
    } else if (strstr(frame_str, "-cruciform")) {
        cruciform = true;
    }
    if (strstr(frame_str, "-elevrev")) {
        reverse_elevator_rudder = true;
    }
    if (strstr(frame_str, "-catapult")) {
        have_launcher = true;
        launch_accel = 15;
        launch_time = 2;
    }
    if (strstr(frame_str, "-bungee")) {
        have_launcher = true;
        launch_accel = 7;
        launch_time = 4;
    }
    if (strstr(frame_str, "-throw")) {
        have_launcher = true;
        launch_accel = 25;
        launch_time = 0.4;
    }
    if (strstr(frame_str, "-tailsitter")) {
        tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }
    if (strstr(frame_str, "-steering")) {
        have_steering = true;
    }

#if AP_FILESYSTEM_FILE_READING_ENABLED
    if (strstr(frame_str, "-3d")) {
        aerobatic = true;
        thrust_scale *= 1.5;
        // setup parameters for plane-3d
        AP_Param::load_defaults_file("@ROMFS/models/plane.parm", false);
        AP_Param::load_defaults_file("@ROMFS/models/plane-3d.parm", false);
    }
#endif

    if (strstr(frame_str, "-ice")) {
        ice_engine = true;
    }

    if (strstr(frame_str, "-soaring")) {
        mass = 2.0;
        coefficient.c_drag_p = 0.05;
    }

    // cruciform missile mode: enforce pitch/yaw aerodynamic symmetry
    if (cruciform) {
        float ratio_cb = coefficient.c / coefficient.b;     // c/b
        float ratio_cb2 = ratio_cb * ratio_cb;              // (c/b)²

        // control moment symmetry: c_n_deltar = c_m_deltae * c/b
        coefficient.c_n_deltar = coefficient.c_m_deltae * ratio_cb;

        // rate damping symmetry: c²*c_m_q = b²*c_n_r
        coefficient.c_n_r = coefficient.c_m_q * ratio_cb2;

        // static stability symmetry: c*c_m_a = b*c_n_b
        coefficient.c_n_b = fabsf(coefficient.c_m_a) * ratio_cb;

        // sideslip-roll coupling: cruciform symmetric → zero
        coefficient.c_l_b = 0;

        // yaw rate-roll coupling: cruciform symmetric → zero
        coefficient.c_l_r = 0;

        // cruciform + config: rudder roll coupling near-zero for symmetric body
        coefficient.c_l_deltar = 0;

        // inertia symmetry: Izz = Iyy
        if (!is_zero(coefficient.moment_of_inertia.y)) {
            coefficient.moment_of_inertia.z = coefficient.moment_of_inertia.y;
        }

        // differential horizontal fins produce no net side force
        coefficient.c_y_deltaa = 0;

        // side force symmetry: c_y_b mirrors c_lift_a (sideslip → side force = AoA → lift)
        coefficient.c_y_b = -coefficient.c_lift_a;

        // rudder side force symmetry: c_y_deltar mirrors c_lift_deltae
        coefficient.c_y_deltar = -coefficient.c_lift_deltae;

        ::printf("Cruciform mode: c_n_deltar=%.4f c_n_r=%.2f c_n_b=%.3f c_y_b=%.3f (c/b=%.3f)\n",
                 coefficient.c_n_deltar, coefficient.c_n_r, coefficient.c_n_b, coefficient.c_y_b, ratio_cb);
    }

    // air-start: disable ground constraints (position/velocity set on first update)
    air_start_done = false;
    carrier_released = false;
    if (coefficient.initial_alt_offset > 0) {
        ground_behavior = GROUND_BEHAVIOR_NONE;
        ::printf("Air-start configured: alt_offset=%.0fm vel=(%.1f,%.1f,%.1f) pitch=%.1fdeg\n",
                 coefficient.initial_alt_offset,
                 coefficient.initial_velocity.x,
                 coefficient.initial_velocity.y,
                 coefficient.initial_velocity.z,
                 coefficient.initial_pitch_angle);
    }
}

void Plane::load_coeffs(const char *model_json)
{
    // Use POSIX file I/O directly since AP::FS() may not be ready
    // during SITL model construction
    struct stat st;
    if (::stat(model_json, &st) != 0) {
        AP_HAL::panic("%s: file not found", model_json);
    }
    FILE *f = fopen(model_json, "r");
    if (f == nullptr) {
        AP_HAL::panic("%s: cannot open", model_json);
    }
    char *buf = (char *)malloc(st.st_size + 1);
    if (buf == nullptr) {
        fclose(f);
        AP_HAL::panic("%s: out of memory", model_json);
    }
    size_t n = fread(buf, 1, st.st_size, f);
    fclose(f);
    buf[n] = '\0';

    AP_JSON::value *obj = new AP_JSON::value();
    std::string err = AP_JSON::parse(*obj, std::string(buf, n));
    free(buf);
    if (!err.empty()) {
        delete obj;
        AP_HAL::panic("%s: JSON parse error: %s", model_json, err.c_str());
    }

    enum class VarType {
        FLOAT,
        VECTOR3F,
    };

    struct json_search {
        const char *label;
        void *ptr;
        VarType t;
    };
    
    json_search vars[] = {
#define COFF_FLOAT(s) { #s, &coefficient.s, VarType::FLOAT }
        COFF_FLOAT(s),
        COFF_FLOAT(b),
        COFF_FLOAT(c),
        COFF_FLOAT(c_lift_0),
        COFF_FLOAT(c_lift_deltae),
        COFF_FLOAT(c_lift_a),
        COFF_FLOAT(c_lift_q),
        COFF_FLOAT(mcoeff),
        COFF_FLOAT(oswald),
        COFF_FLOAT(alpha_stall),
        COFF_FLOAT(c_drag_q),
        COFF_FLOAT(c_drag_deltae),
        COFF_FLOAT(c_drag_p),
        COFF_FLOAT(c_y_0),
        COFF_FLOAT(c_y_b),
        COFF_FLOAT(c_y_p),
        COFF_FLOAT(c_y_r),
        COFF_FLOAT(c_y_deltaa),
        COFF_FLOAT(c_y_deltar),
        COFF_FLOAT(c_l_0),
        COFF_FLOAT(c_l_p),
        COFF_FLOAT(c_l_b),
        COFF_FLOAT(c_l_r),
        COFF_FLOAT(c_l_deltaa),
        COFF_FLOAT(c_l_deltar),
        COFF_FLOAT(c_m_0),
        COFF_FLOAT(c_m_a),
        COFF_FLOAT(c_m_q),
        COFF_FLOAT(c_m_deltae),
        COFF_FLOAT(c_n_0),
        COFF_FLOAT(c_n_b),
        COFF_FLOAT(c_n_p),
        COFF_FLOAT(c_n_r),
        COFF_FLOAT(c_n_deltaa),
        COFF_FLOAT(c_n_deltar),
        COFF_FLOAT(deltaa_max),
        COFF_FLOAT(deltae_max),
        COFF_FLOAT(deltar_max),
        { "CGOffset", &coefficient.CGOffset, VarType::VECTOR3F },
        { "mass", &coefficient.mass_override, VarType::FLOAT },
        { "moment_of_inertia", &coefficient.moment_of_inertia, VarType::VECTOR3F },
        { "max_thrust", &coefficient.max_thrust, VarType::FLOAT },
        { "initial_alt_offset", &coefficient.initial_alt_offset, VarType::FLOAT },
        { "initial_velocity", &coefficient.initial_velocity, VarType::VECTOR3F },
        { "initial_pitch_angle", &coefficient.initial_pitch_angle, VarType::FLOAT },
        COFF_FLOAT(chute_cd),
        COFF_FLOAT(chute_area),
        COFF_FLOAT(chute_open_time),
        COFF_FLOAT(chute_attach_x),
    };

    for (uint8_t i=0; i<ARRAY_SIZE(vars); i++) {
        auto v = obj->get(vars[i].label);
        if (v.is<AP_JSON::null>()) {
            // use default value
            continue;
        }
        if (vars[i].t == VarType::FLOAT) {
            parse_float(v, vars[i].label, *((float *)vars[i].ptr));

        } else if (vars[i].t == VarType::VECTOR3F) {
            parse_vector3(v, vars[i].label, *(Vector3f *)vars[i].ptr);

        }
    }

    delete obj;

    ::printf("Loaded plane aero coefficients from %s\n", model_json);
}

void Plane::parse_float(AP_JSON::value val, const char* label, float &param) {
    if (!val.is<double>()) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    param = val.get<double>();
}

void Plane::parse_vector3(AP_JSON::value val, const char* label, Vector3f &param) {
    if (!val.is<AP_JSON::value::array>() || !val.contains(2) || val.contains(3)) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    for (uint8_t j=0; j<3; j++) {
        parse_float(val.get(j), label, param[j]);
    }
}

/*
  the following functions are from last_letter
  https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp
  many thanks to Georacer!
 */
float Plane::liftCoeff(float alpha) const
{
    const float alpha0 = coefficient.alpha_stall;
    const float M = coefficient.mcoeff;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;

    // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
    const float max_alpha_delta = 0.8f;
    if (alpha-alpha0 > max_alpha_delta) {
        alpha = alpha0 + max_alpha_delta;
    } else if (alpha0-alpha > max_alpha_delta) {
        alpha = alpha0 - max_alpha_delta;
    }
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	float result  = linear+flatPlate;
	return result;
}

float Plane::dragCoeff(float alpha) const
{
    const float b = coefficient.b;
    const float s = coefficient.s;
    const float c_drag_p = coefficient.c_drag_p;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;
    const float oswald = coefficient.oswald;
    
	double AR = pow(b,2)/s;
	double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);

	return c_drag_a;
}

// Torque calculation function
Vector3f Plane::getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const
{
    float alpha = angle_of_attack;

	//calculate aerodynamic torque
    float effective_airspeed = airspeed;

    if (tailsitter || aerobatic) {
        /*
          tailsitters get airspeed from prop-wash
         */
        effective_airspeed += inputThrust * 20;

        // reduce effective angle of attack as thrust increases
        alpha *= constrain_float(1 - inputThrust, 0, 1);
    }
    
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_l_0 = coefficient.c_l_0;
    const float c_l_b = coefficient.c_l_b;
    const float c_l_p = coefficient.c_l_p;
    const float c_l_r = coefficient.c_l_r;
    const float c_l_deltaa = coefficient.c_l_deltaa;
    const float c_l_deltar = coefficient.c_l_deltar;
    const float c_m_0 = coefficient.c_m_0;
    const float c_m_a = coefficient.c_m_a;
    const float c_m_q = coefficient.c_m_q;
    const float c_m_deltae = coefficient.c_m_deltae;
    const float c_n_0 = coefficient.c_n_0;
    const float c_n_b = coefficient.c_n_b;
    const float c_n_p = coefficient.c_n_p;
    const float c_n_r = coefficient.c_n_r;
    const float c_n_deltaa = coefficient.c_n_deltaa;
    const float c_n_deltar = coefficient.c_n_deltar;
    const Vector3f &CGOffset = coefficient.CGOffset;
    
    float rho = air_density;

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	double qbar = 1.0/2.0*rho*pow(effective_airspeed,2)*s; //Calculate dynamic pressure
	double la, na, ma;
	if (is_zero(effective_airspeed))
	{
		la = 0;
		ma = 0;
		na = 0;
	}
	else
	{
		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*effective_airspeed) + c_l_r*b*r/(2*effective_airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*effective_airspeed) + c_m_deltae*inputElevator);
		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*effective_airspeed) + c_n_r*b*r/(2*effective_airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
	}


	// Add torque to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	la +=  CGOffset.y * force.z - CGOffset.z * force.y;
	ma += -CGOffset.x * force.z + CGOffset.z * force.x;
	na += -CGOffset.y * force.x + CGOffset.x * force.y;

	return Vector3f(la, ma, na);
}

// Force calculation function from last_letter
Vector3f Plane::getForce(float inputAileron, float inputElevator, float inputRudder) const
{
    const float alpha = angle_of_attack;
    const float c_drag_q = coefficient.c_drag_q;
    const float c_lift_q = coefficient.c_lift_q;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_drag_deltae = coefficient.c_drag_deltae;
    const float c_lift_deltae = coefficient.c_lift_deltae;
    const float c_y_0 = coefficient.c_y_0;
    const float c_y_b = coefficient.c_y_b;
    const float c_y_p = coefficient.c_y_p;
    const float c_y_r = coefficient.c_y_r;
    const float c_y_deltaa = coefficient.c_y_deltaa;
    const float c_y_deltar = coefficient.c_y_deltar;
    
    float rho = air_density;

	//request lift and drag alpha-coefficients from the corresponding functions
	double c_lift_a = liftCoeff(alpha);
	double c_drag_a = dragCoeff(alpha);

	//convert coefficients to the body frame
	double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
	double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double ax, ay, az;
	if (is_zero(airspeed))
	{
		ax = 0;
		ay = 0;
		az = 0;
	}
	else
	{
		ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
		// split c_x_deltae to include "abs" term
		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
		az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
		// split c_z_deltae to include "abs" term
	}
    return Vector3f(ax, ay, az);
}

void Plane::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel)
{
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float rudder   = filtered_servo_angle(input, 3);
    bool launch_triggered = input.servos[6] > 1700;
    float throttle;
    if (reverse_elevator_rudder) {
        elevator = -elevator;
        rudder = -rudder;
    }
    if (elevons) {
        // fake an elevon plane
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;

        // assume no rudder
        rudder = 0;
    } else if (vtail) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    } else if (dspoilers) {
        // fake a differential spoiler plane. Use outputs 1, 2, 4 and 5
        float dspoiler1_left = filtered_servo_angle(input, 0);
        float dspoiler1_right = filtered_servo_angle(input, 1);
        float dspoiler2_left = filtered_servo_angle(input, 3);
        float dspoiler2_right = filtered_servo_angle(input, 4);
        float elevon_left  = (dspoiler1_left + dspoiler2_left)/2;
        float elevon_right = (dspoiler1_right + dspoiler2_right)/2;
        aileron  = (elevon_right-elevon_left)/2;
        elevator = (elevon_left+elevon_right)/2;
        rudder = fabsf(dspoiler1_right - dspoiler2_right)/2 - fabsf(dspoiler1_left - dspoiler2_left)/2;
    } else if (redundant) {
        // channels 1/9 are left/right ailierons
        // channels 2/10 are left/right elevators
        // channels 4/12 are top/bottom rudders
        aileron  = (filtered_servo_angle(input, 0) + filtered_servo_angle(input, 8)) / 2.0;
        elevator = (filtered_servo_angle(input, 1) + filtered_servo_angle(input, 9)) / 2.0;
        rudder   = (filtered_servo_angle(input, 3) + filtered_servo_angle(input, 11)) / 2.0;
    } else if (cruciform) {
        // cruciform 4-fin: read SERVO5-8 (channels 4-7)
        float fin_L = filtered_servo_angle(input, 4);  // left  (horizontal)
        float fin_R = filtered_servo_angle(input, 5);  // right (horizontal)
        float fin_U = filtered_servo_angle(input, 6);  // upper (vertical)
        float fin_D = filtered_servo_angle(input, 7);  // lower (vertical)

        // unmix: 4 fins → 3-axis equivalent
        elevator = (fin_L + fin_R) / 2.0f;                    // pitch = horizontal common mode
        rudder   = (fin_U + fin_D) / 2.0f;                    // yaw   = vertical common mode
        aileron  = (fin_R - fin_L + fin_D - fin_U) / 4.0f;    // roll  = differential
    }
    //printf("Aileron: %.1f elevator: %.1f rudder: %.1f\n", aileron, elevator, rudder);

    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    
    float thrust     = throttle;

    battery_voltage = sitl->batt_voltage - 0.7*throttle;
    battery_current = (battery_voltage/sitl->batt_voltage)*50.0f*sq(throttle);

    if (ice_engine) {
        thrust = icengine.update(input);
    }

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    if (tailsitter || aerobatic) {
        /*
          tailsitters get 4x the control surfaces
         */
        aileron *= 4;
        elevator *= 4;
        rudder *= 4;
    }
    
    Vector3f force = getForce(aileron, elevator, rudder);
    rot_accel = getTorque(aileron, elevator, rudder, thrust, force);

    // convert torque (N·m) to angular acceleration (rad/s²) using moment of inertia
    const auto &I = coefficient.moment_of_inertia;
    if (!is_zero(I.x) && !is_zero(I.y) && !is_zero(I.z)) {
        rot_accel.x /= I.x;
        rot_accel.y /= I.y;
        rot_accel.z /= I.z;
    }

    // safety: clamp angular acceleration to prevent numerical divergence
    const float max_rot_accel = 50.0f; // rad/s²
    rot_accel.x = constrain_float(rot_accel.x, -max_rot_accel, max_rot_accel);
    rot_accel.y = constrain_float(rot_accel.y, -max_rot_accel, max_rot_accel);
    rot_accel.z = constrain_float(rot_accel.z, -max_rot_accel, max_rot_accel);

    // safety: check for NaN/Inf in forces and accelerations
    if (!isfinite(force.x) || !isfinite(force.y) || !isfinite(force.z)) {
        force.zero();
    }
    if (!isfinite(rot_accel.x) || !isfinite(rot_accel.y) || !isfinite(rot_accel.z)) {
        rot_accel.zero();
    }

    if (have_launcher) {
        /*
          simple simulation of a launcher
         */
        if (launch_triggered) {
            uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                launch_start_ms = now;
            }
            if (now - launch_start_ms < launch_time*1000) {
                force.x += mass * launch_accel;
                force.z += mass * launch_accel/3;
            }
        } else {
            // allow reset of catapult
            launch_start_ms = 0;
        }
    }
    
    // simulate engine RPM
    motor_mask |= (1U<<2);
    rpm[2] = thrust * 7000;
    
    // scale thrust to newtons
    thrust *= thrust_scale;

    accel_body = Vector3f(thrust, 0, 0) + force;
    accel_body /= mass;

    // add some noise
    if (thrust_scale > 0) {
        add_noise(fabsf(thrust) / thrust_scale);
    }

    if (on_ground() && !tailsitter) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }

    // === Parachute drag model ===
    // Detect SERVO9 (ScriptMotor5, FUNCTION=98) activation
    if (!parachute_deployed && input.servos[8] > 1500) {
        parachute_deployed = true;
        parachute_deploy_ms = AP_HAL::millis64();
        // re-enable ground collision for air-start models (GROUND_BEHAVIOR_NONE → FWD_ONLY)
        ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
        ::printf("Parachute deployed at %.0fm AGL, V=%.0fm/s\n",
                 -position.z, velocity_ef.length());
    }
    if (parachute_deployed) {
        float t_since = (AP_HAL::millis64() - parachute_deploy_ms) * 0.001f;
        // inflation: linear ramp 0->1 over chute_open_time
        float inflation = constrain_float(t_since / coefficient.chute_open_time, 0, 1);
        float V = velocity_air_bf.length();
        if (V > 0.1f) {
            float q = 0.5f * air_density * V * V;
            float F_chute = q * coefficient.chute_cd * coefficient.chute_area * inflation;
            Vector3f drag_dir = -velocity_air_bf.normalized();
            Vector3f chute_force = drag_dir * F_chute;
            // add drag acceleration
            accel_body += chute_force / mass;
            // torque from off-CG attachment (stabilizing pitch moment)
            Vector3f attach(coefficient.chute_attach_x, 0, 0);
            Vector3f torque = attach % chute_force;
            const auto &Ic = coefficient.moment_of_inertia;
            if (!is_zero(Ic.x) && !is_zero(Ic.y) && !is_zero(Ic.z)) {
                rot_accel.x += torque.x / Ic.x;
                rot_accel.y += torque.y / Ic.y;
                rot_accel.z += torque.z / Ic.z;
            }
        }
    }
}
    
/*
  update the plane simulation by one time step
 */
void Plane::update(const struct sitl_input &input)
{
    // air-start: initialize state on first frame (before physics)
    if (coefficient.initial_alt_offset > 0 && !air_start_done) {
        position.zero();
        position.z = -coefficient.initial_alt_offset;
        // decompose initial_velocity along pitch angle:
        // horizontal component scales by cos(pitch), vertical from speed * sin(pitch)
        // speed magnitude is always positive; nose-down (negative pitch) → positive Vz (downward in NED)
        const float pitch_rad = radians(coefficient.initial_pitch_angle);
        const float speed = coefficient.initial_velocity.length();
        velocity_ef = coefficient.initial_velocity;
        velocity_ef.x = coefficient.initial_velocity.x * cosf(pitch_rad);
        velocity_ef.y = coefficient.initial_velocity.y * cosf(pitch_rad);
        velocity_ef.z = -speed * sinf(pitch_rad);  // nose-down (negative pitch) → positive Vz (downward)
        dcm.from_euler(0, pitch_rad, radians(home_yaw));
        gyro.zero();
        // gravity in body frame at the given pitch angle
        accel_body = Vector3f(GRAVITY_MSS * sinf(pitch_rad), 0, -GRAVITY_MSS * cosf(pitch_rad));
        air_start_done = true;
        carrier_released = false;
        ::printf("Air-start: carrier hold at %.0fm AGL, V=(%.1f,%.1f,%.1f), pitch=%.1fdeg\n",
                 coefficient.initial_alt_offset,
                 velocity_ef.x, velocity_ef.y, velocity_ef.z,
                 coefficient.initial_pitch_angle);
    }

    Vector3f rot_accel;

    update_wind(input);

    calculate_forces(input, rot_accel);

    update_dynamics(rot_accel);

    // air-start: carrier hold — override state AFTER physics to prevent drift
    if (coefficient.initial_alt_offset > 0 && air_start_done && !carrier_released) {
        float throttle = filtered_servo_range(input, 2);
        if (throttle >= 0.1f) {
            carrier_released = true;
            ::printf("Air-start: RELEASED at %.0fm AGL, throttle=%.0f%%\n",
                     -position.z, throttle * 100);
        } else {
            // freeze all state: position, velocity, attitude, angular rate, accel
            position.zero();
            position.z = -coefficient.initial_alt_offset;
            const float pitch_rad = radians(coefficient.initial_pitch_angle);
            const float speed = coefficient.initial_velocity.length();
            velocity_ef = coefficient.initial_velocity;
            velocity_ef.x = coefficient.initial_velocity.x * cosf(pitch_rad);
            velocity_ef.y = coefficient.initial_velocity.y * cosf(pitch_rad);
            velocity_ef.z = -speed * sinf(pitch_rad);
            dcm.from_euler(0, pitch_rad, radians(home_yaw));
            gyro.zero();
            // reset accelerometer to gravity at the given pitch angle
            // so the AHRS converges to correct attitude during carrier hold
            accel_body = Vector3f(GRAVITY_MSS * sinf(pitch_rad), 0, -GRAVITY_MSS * cosf(pitch_rad));
        }
    }

    /*
      add in ground steering, this should be replaced with a proper
      calculation of a nose wheel effect
    */
    if (have_steering && on_ground()) {
        const float steering = filtered_servo_angle(input, 4);
        const Vector3f velocity_bf = dcm.transposed() * velocity_ef;
        const float steer_scale = radians(5);
        gyro.z += steering * velocity_bf.x * steer_scale;
    }

    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
