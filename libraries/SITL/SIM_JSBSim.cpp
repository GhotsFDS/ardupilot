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
  simulator connector for JSBSim
*/

#include "SIM_config.h"

#if AP_SIM_JSBSIM_ENABLED

#include "SIM_JSBSim.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <fenv.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Unit conversion constants
static constexpr float NEWTONS_TO_LBF = 0.224809f;
static constexpr float METERS_TO_FEET = 3.28084f;

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

#define DEBUG_JSBSIM 0

JSBSim::JSBSim(const char *frame_str) :
    Aircraft(frame_str),
    sock_control(false),
    sock_fgfdm(true),
    initialised(false),
    jsbsim_script(nullptr),
    jsbsim_fgout(nullptr),
    created_templates(false),
    started_jsbsim(false),
    opened_control_socket(false),
    opened_fdm_socket(false),
    frame(FRAME_NORMAL)
{
    if (strstr(frame_str, "elevon")) {
        frame = FRAME_ELEVON;
    } else if (strstr(frame_str, "vtail")) {
        frame = FRAME_VTAIL;
    } else if (strstr(frame_str, "cruciform") || strstr(frame_str, "STT")) {
        frame = FRAME_CRUCIFORM;
    } else {
        frame = FRAME_NORMAL;
    }
    const char *model_name = strchr(frame_str, ':');
    if (model_name != nullptr) {
        jsbsim_model = model_name + 1;
    }
    control_port = 5505 + instance*10;
    fdm_port = 5504 + instance*10;
}

/*
  create template files
 */
bool JSBSim::create_templates(void)
{
    if (created_templates) {
        return true;
    }

    asprintf(&jsbsim_script, "%s/jsbsim_start_%u.xml", autotest_dir, instance);
    asprintf(&jsbsim_fgout,  "%s/jsbsim_fgout_%u.xml", autotest_dir, instance);

    printf("JSBSim_script: '%s'\n", jsbsim_script);
    printf("JSBSim_fgout: '%s'\n", jsbsim_fgout);

    FILE *f = fopen(jsbsim_script, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim script %s", jsbsim_script);
    }
    fprintf(f,
"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
"<?xml-stylesheet type=\"text/xsl\" href=\"http://jsbsim.sf.net/JSBSimScript.xsl\"?>\n"
"<runscript xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
"    xsi:noNamespaceSchemaLocation=\"http://jsbsim.sf.net/JSBSimScript.xsd\"\n"
"    name=\"Testing %s\">\n"
"\n"
"  <description>\n"
"    test ArduPlane using %s and JSBSim\n"
"  </description>\n"
"\n"
"  <use aircraft=\"%s\" initialize=\"reset\"/>\n"
"\n"
"  <!-- we control the servos via the jsbsim console\n"
"       interface on TCP 5124 -->\n"
"  <input port=\"%u\"/>\n"
"\n"
"  <run start=\"0\" end=\"10000000\" dt=\"%.6f\">\n"
"    <property value=\"0\"> simulation/notify-time-trigger </property>\n"
"%s"
"  </run>\n"
"\n"
"</runscript>\n"
"",
            jsbsim_model,
            jsbsim_model,
            jsbsim_model,
            control_port,
            1.0/rate_hz,
            (frame == FRAME_CRUCIFORM) ? "\n" :
            "\n"
            "    <event name=\"start engine\">\n"
            "      <condition> simulation/sim-time-sec le 0.01 </condition>\n"
            "      <set name=\"propulsion/engine[0]/set-running\" value=\"1\"/>\n"
            "      <notify/>\n"
            "    </event>\n"
            "\n"
            "    <event name=\"Trim\">\n"
            "      <condition>simulation/sim-time-sec ge 0.01</condition>\n"
            "      <set name=\"simulation/do_simple_trim\" value=\"2\"/>\n"
            "      <notify/>\n"
            "    </event>\n");
    fclose(f);

    f = fopen(jsbsim_fgout, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim fgout script %s", jsbsim_fgout);
    }
    fprintf(f, "<?xml version=\"1.0\"?>\n"
            "<output name=\"127.0.0.1\" type=\"FLIGHTGEAR\" port=\"%u\" protocol=\"UDP\" rate=\"%f\">\n"
            "  <time type=\"simulation\" resolution=\"1e-6\"/>\n"
            "</output>",
            fdm_port, rate_hz);
    fclose(f);

    char *jsbsim_reset;
    asprintf(&jsbsim_reset, "%s/aircraft/%s/reset.xml", autotest_dir, jsbsim_model);

    printf("JSBSim_reset: '%s'\n", jsbsim_reset);

    f = fopen(jsbsim_reset, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim reset script %s", jsbsim_reset);
    }
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    if (frame == FRAME_CRUCIFORM) {
        // Air start: 3000m AGL, 33 m/s (carrier separation speed)
        // EKF bypassed with AHRS_EKF_TYPE=10 (SIM mode).
        // Lua handles real BOOST phase (2500N thrust for STT_BURN_TME).
        // NOTE: Use metric units (M, M/SEC) — JSBSim 1.2.x
        // may misinterpret unit="FT" in init files.
        fprintf(f,
                "<?xml version=\"1.0\"?>\n"
                "<initialize name=\"STT Air Start\">\n"
                "  <latitude unit=\"DEG\" type=\"geodetic\"> %f </latitude>\n"
                "  <longitude unit=\"DEG\"> %f </longitude>\n"
                "  <altitude unit=\"M\"> %f </altitude>\n"
                "  <ubody unit=\"M/SEC\"> %f </ubody>\n"
                "  <vbody unit=\"M/SEC\"> 0.0 </vbody>\n"
                "  <wbody unit=\"M/SEC\"> 0.0 </wbody>\n"
                "  <phi unit=\"DEG\"> 0.0 </phi>\n"
                "  <theta unit=\"DEG\"> 0.0 </theta>\n"
                "  <psi unit=\"DEG\"> %f </psi>\n"
                "</initialize>\n",
                home.lat*1.0e-7,
                home.lng*1.0e-7,
                home.alt*0.01 + 3000.0,  // 3000m AGL in meters
                33.0,                     // 33 m/s (carrier separation speed)
                wrap_360(degrees(y)));
    } else {
        // Ground start (original)
        fprintf(f,
                "<?xml version=\"1.0\"?>\n"
                "<initialize name=\"Start up location\">\n"
                "  <latitude unit=\"DEG\" type=\"geodetic\"> %f </latitude>\n"
                "  <longitude unit=\"DEG\"> %f </longitude>\n"
                "  <altitude unit=\"M\"> 1.3 </altitude>\n"
                "  <vt unit=\"FT/SEC\"> 0.0 </vt>\n"
                "  <gamma unit=\"DEG\"> 0.0 </gamma>\n"
                "  <phi unit=\"DEG\"> 0.0 </phi>\n"
                "  <theta unit=\"DEG\"> 13.0 </theta>\n"
                "  <psi unit=\"DEG\"> %f </psi>\n"
                "</initialize>\n",
                home.lat*1.0e-7,
                home.lng*1.0e-7,
                degrees(y));
    }
    fclose(f);

    created_templates = true;
    return true;
}


/*
  start JSBSim child
 */
bool JSBSim::start_JSBSim(void)
{
    if (started_jsbsim) {
        return true;
    }
    if (!open_fdm_socket()) {
        return false;
    }

    int p[2];
    int devnull = open("/dev/null", O_RDWR|O_CLOEXEC);
    if (pipe(p) != 0) {
        AP_HAL::panic("Unable to create pipe");
    }
    pid_t child_pid = fork();
    if (child_pid == 0) {
        // in child
        setsid();
        dup2(devnull, 0);
        dup2(p[1], 1);
        close(p[0]);
        for (uint8_t i=3; i<100; i++) {
            close(i);
        }
        char *logdirective;
        char *script;
        char *nice;
        char *rate;

        asprintf(&logdirective, "--logdirectivefile=%s", jsbsim_fgout);
        asprintf(&script, "--script=%s", jsbsim_script);
        asprintf(&nice, "--nice=%.8f", 10*1e-9);
        asprintf(&rate, "--simulation-rate=%f", rate_hz);

        if (chdir(autotest_dir) != 0) {
            perror(autotest_dir);
            exit(1);
        }

        int ret = execlp("JSBSim",
                         "JSBSim",
                         "--suspend",
                         rate,
                         nice,
                         logdirective,
                         script,
                         nullptr);
        if (ret != 0) {
            perror("JSBSim");
        }
        exit(1);
    }
    close(p[1]);
    jsbsim_stdout = p[0];

    // read startup to be sure it is running
    char c;
    if (read(jsbsim_stdout, &c, 1) != 1) {
        AP_HAL::panic("Unable to start JSBSim");
    }

    if (!expect("JSBSim Execution beginning")) {
        AP_HAL::panic("Failed to start JSBSim");
    }
    if (!open_control_socket()) {
        AP_HAL::panic("Failed to open JSBSim control socket");
    }

    fcntl(jsbsim_stdout, F_SETFL, fcntl(jsbsim_stdout, F_GETFL, 0) | O_NONBLOCK);

    started_jsbsim = true;
    check_stdout();
    close(devnull);
    return true;
}

/*
  check for stdout from JSBSim
 */
void JSBSim::check_stdout(void) const
{
    char line[100];
    ssize_t ret = ::read(jsbsim_stdout, line, sizeof(line));
    if (ret > 0) {
#if DEBUG_JSBSIM
        write(1, line, ret);
#endif
    }
}

/*
  a simple function to wait for a string on jsbsim_stdout
 */
bool JSBSim::expect(const char *str) const
{
    const char *basestr = str;
    while (*str) {
        char c;
        if (read(jsbsim_stdout, &c, 1) != 1) {
            return false;
        }
        if (c == *str) {
            str++;
        } else {
            str = basestr;
        }
#if DEBUG_JSBSIM
        write(1, &c, 1);
#endif
    }
    return true;
}

/*
  open control socket to JSBSim
 */
bool JSBSim::open_control_socket(void)
{
    if (opened_control_socket) {
        return true;
    }
    if (!sock_control.connect("127.0.0.1", control_port)) {
        return false;
    }
    printf("Opened JSBSim control socket\n");
    sock_control.set_blocking(false);
    opened_control_socket = true;

    // resume starts JSBSim, iterate 1 causes it to hold after one step
    // From then on, each iterate 1 in send_servos advances exactly 1 step
    char startup[] =
        "info\n"
        "resume\n"
        "iterate 1\n"
        "set atmosphere/turb-type 4\n";
    sock_control.send(startup, strlen(startup));
    return true;
}

/*
  open fdm socket from JSBSim
 */
bool JSBSim::open_fdm_socket(void)
{
    if (opened_fdm_socket) {
        return true;
    }
    if (!sock_fgfdm.bind("127.0.0.1", fdm_port)) {
        check_stdout();
        return false;
    }
    sock_fgfdm.set_blocking(false);
    sock_fgfdm.reuseaddress();
    opened_fdm_socket = true;
    return true;
}


/*
  decode and send servos
*/
void JSBSim::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    float aileron, elevator, throttle, rudder;

    if (frame == FRAME_CRUCIFORM) {
        // Cruciform missile: hold controls at zero until Lua takes over.
        // FBWA during init causes roll divergence (c_l_b=0, no passive stability).
        // With zero controls and phi=0 init, c_l_p damping keeps roll near zero.
        constexpr float BOOST_BURN_S = 1.0f;
        float sim_time_s = time_now_us * 1.0e-6f;
        bool in_boost = (sim_time_s < BOOST_BURN_S);

        // Time-based init hold: keep zero controls for first 5 seconds.
        // This covers ArduPlane boot + AHRS convergence. After that,
        // always use the unmixer — no fins_active check needed.
        // The old fins_active + bias approach caused roll divergence:
        // 0.01 bias × qbar at 175 m/s = 123°/s² roll acceleration.
        constexpr float INIT_HOLD_S = 5.0f;

        if (sim_time_s >= INIT_HOLD_S) {
            // Normal operation: unmix SERVO5-8 (CTAIL) into aileron/elevator/rudder.
            // Use RAW servo values — filtered_servo_angle's per-channel LPF
            // creates parasitic aileron during transients (each fin's filter
            // lags differently → unmixer extracts wrong-sign roll → divergence).
            float fin_L = (input.servos[4] - 1500) / 500.0f;
            float fin_R = (input.servos[5] - 1500) / 500.0f;
            float fin_U = (input.servos[6] - 1500) / 500.0f;
            float fin_D = (input.servos[7] - 1500) / 500.0f;
            elevator = (fin_L + fin_R) / 2.0f;
            rudder   = (fin_U + fin_D) / 2.0f;
            aileron  = (fin_R - fin_L + fin_D - fin_U) / 4.0f;
        } else {
            // Init hold: zero controls. c_l_p damps any numerical noise.
            aileron  = 0;
            elevator = 0;
            rudder   = 0;
        }

        // Rocket motor: single-use solid propellant, fires at separation.
        throttle = in_boost ? 1.0f : 0.0f;

        float parachute_cmd = (input.servos[8] > 1500) ? 1.0f : 0.0f;
        float wind_speed_fps = input.wind.speed / FEET_TO_METERS;

        // Send throttle command (0-1). FCS passes to fcs/throttle-pos-norm,
        // aerodynamics DRAG axis applies -562*throttle lbf (forward force).
        asprintf(&buf,
                 "set fcs/aileron-cmd-norm %f\n"
                 "set fcs/elevator-cmd-norm %f\n"
                 "set fcs/rudder-cmd-norm %f\n"
                 "set fcs/parachute-cmd-norm %f\n"
                 "set fcs/throttle-cmd-norm %f\n"
                 "set atmosphere/psiw-rad %f\n"
                 "set atmosphere/wind-mag-fps %f\n"
                 "set atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps %f\n"
                 "set atmosphere/turbulence/milspec/severity %f\n"
                 "iterate 1\n",
                 aileron, elevator, rudder,
                 parachute_cmd, throttle,
                 radians(input.wind.direction),
                 wind_speed_fps,
                 wind_speed_fps/3,
                 input.wind.turbulence);
    } else {
        // Normal / elevon / vtail frames
        aileron  = filtered_servo_angle(input, 0);
        elevator = filtered_servo_angle(input, 1);
        throttle = filtered_servo_range(input, 2);
        rudder   = filtered_servo_angle(input, 3);
        if (frame == FRAME_ELEVON) {
            float ch1 = aileron;
            float ch2 = elevator;
            aileron  = (ch2-ch1)/2.0f;
            elevator = -(ch2+ch1)/2.0f;
        } else if (frame == FRAME_VTAIL) {
            float ch1 = elevator;
            float ch2 = rudder;
            elevator = (ch2-ch1)/2.0f;
            rudder   = (ch2+ch1)/2.0f;
        }
        float wind_speed_fps = input.wind.speed / FEET_TO_METERS;
        asprintf(&buf,
                 "set fcs/aileron-cmd-norm %f\n"
                 "set fcs/elevator-cmd-norm %f\n"
                 "set fcs/rudder-cmd-norm %f\n"
                 "set fcs/throttle-cmd-norm %f\n"
                 "set atmosphere/psiw-rad %f\n"
                 "set atmosphere/wind-mag-fps %f\n"
                 "set atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps %f\n"
                 "set atmosphere/turbulence/milspec/severity %f\n"
                 "iterate 1\n",
                 aileron, elevator, rudder, throttle,
                 radians(input.wind.direction),
                 wind_speed_fps,
                 wind_speed_fps/3,
                 input.wind.turbulence);
    }

    ssize_t buflen = strlen(buf);
    ssize_t sent = sock_control.send(buf, buflen);
    free(buf);

    if (sent < 0) {
        if (errno != EAGAIN) {
            fprintf(stderr, "Fatal: Failed to send on control socket: %s\n",
                    strerror(errno));
            exit(1);
        }
    }
    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }
}

/* nasty hack ....
   JSBSim sends in little-endian
 */
void FGNetFDM::ByteSwap(void)
{
    uint32_t *buf = (uint32_t *)this;
    for (uint16_t i=0; i<sizeof(*this)/4; i++) {
        buf[i] = ntohl(buf[i]);
    }
    // fixup the 3 doubles
    buf = (uint32_t *)&longitude;
    uint32_t tmp;
    for (uint8_t i=0; i<3; i++) {
        tmp = buf[0];
        buf[0] = buf[1];
        buf[1] = tmp;
        buf += 2;
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void JSBSim::recv_fdm(const struct sitl_input &input)
{
    FGNetFDM fdm;
    check_stdout();

    do {
        ssize_t recv_ret = sock_fgfdm.recv(&fdm, sizeof(fdm), 100);
        while (recv_ret != sizeof(fdm)) {
            send_servos(input);
            drain_control_socket();
            check_stdout();
            recv_ret = sock_fgfdm.recv(&fdm, sizeof(fdm), 100);
        }
        fdm.ByteSwap();
    } while (fdm.cur_time == time_now_us);

    // Temporarily disable FP exceptions while processing FDM data.
    // JSBSim can produce extreme values (e.g. missile on ground tumbling)
    // that would trigger FPE via SITL's feenableexcept().
    int saved_excepts = fedisableexcept(FE_OVERFLOW | FE_DIVBYZERO | FE_INVALID);
    feclearexcept(FE_ALL_EXCEPT);

    accel_body = Vector3f(fdm.A_X_pilot, fdm.A_Y_pilot, fdm.A_Z_pilot) * FEET_TO_METERS;

    double p, q, r;
    SIM::convert_body_frame(degrees(fdm.phi), degrees(fdm.theta),
                             degrees(fdm.phidot), degrees(fdm.thetadot), degrees(fdm.psidot),
                             &p, &q, &r);
    gyro = Vector3f(p, q, r);
    // Clamp gyro rates to prevent AHRS DCM divergence during tumble
    // (e.g. missile bouncing on ground during init)
    constexpr float MAX_GYRO = 20.0f;  // rad/s (~1146 deg/s)
    gyro.x = constrain_float(gyro.x, -MAX_GYRO, MAX_GYRO);
    gyro.y = constrain_float(gyro.y, -MAX_GYRO, MAX_GYRO);
    gyro.z = constrain_float(gyro.z, -MAX_GYRO, MAX_GYRO);

    velocity_ef = Vector3f(fdm.v_north, fdm.v_east, fdm.v_down) * FEET_TO_METERS;
    // Use fdm.altitude (AMSL meters) for cruciform because JSBSim terrain
    // elevation may not match ArduPilot home elevation.
    // Normal planes use fdm.agl + home.alt which works when terrain ≈ home.
    int32_t alt_cm;
    if (frame == FRAME_CRUCIFORM) {
        alt_cm = int32_t(fdm.altitude * 100);  // AMSL meters → cm
    } else {
        alt_cm = int32_t(fdm.agl * 100 + home.alt);
    }
    location = {
        int32_t(RAD_TO_DEG_DOUBLE * fdm.latitude * 1.0e7),
        int32_t(RAD_TO_DEG_DOUBLE * fdm.longitude * 1.0e7),
        alt_cm,
        Location::AltFrame::ABSOLUTE
    };
    dcm.from_euler(fdm.phi, fdm.theta, fdm.psi);
    airspeed = fdm.vcas * KNOTS_TO_METERS_PER_SECOND;
    airspeed_pitot = airspeed;

    // update magnetic field
    update_mag_field_bf();

    // Clamp all values to physically reasonable ranges to prevent
    // downstream FPE in AHRS/EKF from extreme FDM values (e.g. ground impact)
    constexpr float MAX_ACCEL = 500.0f;  // m/s² (~50g)
    constexpr float MAX_VEL = 400.0f;    // m/s (~1.2 Mach)
    constexpr float MAX_ASPD = 400.0f;   // m/s

    for (uint8_t i = 0; i < 3; i++) {
        if (!std::isfinite(accel_body[i])) accel_body[i] = 0;
        accel_body[i] = constrain_float(accel_body[i], -MAX_ACCEL, MAX_ACCEL);
        if (!std::isfinite(gyro[i])) gyro[i] = 0;
        // gyro already clamped above
        if (!std::isfinite(velocity_ef[i])) velocity_ef[i] = 0;
        velocity_ef[i] = constrain_float(velocity_ef[i], -MAX_VEL, MAX_VEL);
    }
    if (!std::isfinite(airspeed)) airspeed = 0;
    airspeed = constrain_float(airspeed, 0, MAX_ASPD);
    if (!std::isfinite(airspeed_pitot)) airspeed_pitot = 0;
    airspeed_pitot = constrain_float(airspeed_pitot, 0, MAX_ASPD);

    // Re-enable FP exceptions
    feclearexcept(FE_ALL_EXCEPT);
    if (saved_excepts >= 0) {
        feenableexcept(saved_excepts);
    }

    rpm[0] = fdm.rpm[0];
    rpm[1] = fdm.rpm[1];

    time_now_us = fdm.cur_time;
}

void JSBSim::drain_control_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = sock_control.recv(buf, buflen, 0);
    } while (received > 0);
}
/*
  update the JSBSim simulation by one time step
 */
void JSBSim::update(const struct sitl_input &input)
{
    while (!initialised) {
        if (!create_templates() ||
            !start_JSBSim() ||
            !open_control_socket() ||
            !open_fdm_socket()) {
            time_now_us = 1;
            return;
        }
        initialised = true;
    }
    send_servos(input);
    recv_fdm(input);
    adjust_frame_time(rate_hz);
    sync_frame_time();
    drain_control_socket();
}

} // namespace SITL

#endif  // AP_SIM_JSBSIM_ENABLED
