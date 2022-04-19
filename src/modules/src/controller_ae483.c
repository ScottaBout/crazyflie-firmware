#include "controller_ae483.h"
#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "radiolink.h"
#include "string.h"
#include "debug.h"
#include "configblock.h"
#include "time.h"
#include "usec_time.h"

// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;

// An example parameter
// static bool use_observer = false;

// Parameters
static uint16_t use_observer = 0;
static bool reset_observer = false;

// State
static float o_x = 0.0f;
static float o_y = 0.0f;
static float o_z = 0.0f;
static float psi = 0.0f;
static float theta = 0.0f;
static float phi = 0.0f;
static float v_x = 0.0f;
static float v_y = 0.0f;
static float v_z = 0.0f;
static float w_x = 0.0f;
static float w_y = 0.0f;
static float w_z = 0.0f;

// Setpoint
static float o_x_des = 0.0f;
static float o_y_des = 0.0f;
static float o_z_des = 0.0f;

// Input
static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;
static float f_z = 0.0f;

// Motor power command
static uint16_t m_1 = 0;
static uint16_t m_2 = 0;
static uint16_t m_3 = 0;
static uint16_t m_4 = 0;

// Measurements
static float n_x = 0.0f;
static float n_y = 0.0f;
static float r = 0.0f;
static float a_z = 0.0f;

// Constants
static float k_flow = 4.09255568f;
static float g = 9.81f;
static float dt = 0.002f;
static float o_z_eq = 0.5f;

// Measurement errors
static float n_x_err = 0.0f;
static float n_y_err = 0.0f;
static float r_err = 0.0f;

// OptiTrack State
static float x = 0.0f;
static float y = 0.0f;
static float z = 0.0f;
static float qx = 0.0f;
static float qy = 0.0f;
static float qz = 0.0f;
static float qw = 0.0f;
static float old_x = 0.0f;
static float old_y = 0.0f;
static float old_z = 0.0f;
static float old_ts = 0.0f;
static float dt_pose = 0.0f;

// static long get_nanos(void) {
//   struct timespec ts;
//   clock_gettime(CLOCK_REALTIME, &ts);
//   // timespec_get(&ts, TIME_UTC);
//   return (long)ts.tv_sec * 1000000000L + ts.tv_nsec;
// }

void ae483UpdateWithTOF(tofMeasurement_t *tof)
{
  tof_distance = tof->distance;
  tof_count++;
}

void ae483UpdateWithFlow(flowMeasurement_t *flow)
{
  flow_dpixelx = flow->dpixelx;
  flow_dpixely = flow->dpixely;
  flow_count++;
}

void ae483UpdateWithDistance(distanceMeasurement_t *meas)
{
  // If you have a loco positioning deck, this function will be called
  // each time a distance measurement is available. You will have to write
  // code to handle these measurements. These data are available:
  //
  //  meas->anchorId  uint8_t   id of anchor with respect to which distance was measured
  //  meas->x         float     x position of this anchor
  //  meas->y         float     y position of this anchor
  //  meas->z         float     z position of this anchor
  //  meas->distance  float     the measured distance
}

void ae483UpdateWithPosition(positionMeasurement_t *meas)
{
  // This function will be called each time you send an external position
  // measurement (x, y, z) from the client, e.g., from a motion capture system.

  old_x = x;            // Store previous x component of external position measurement
  old_y = y;            // Store previous x component of external position measurement
  old_z = z;            // Store previous x component of external position measurement
  float new_ts = (float)usecTimestamp();  // Get current timestamp in nanoseconds
  dt_pose = (float)(new_ts - old_ts) / 1000000.0f; // Time difference between current and previous time stamp in seconds
  old_ts = new_ts;      // Store current timestamp as previous timestamp
  x = meas->x;          // float     x component of external position measurement
  y = meas->y;          // float     y component of external position measurement
  z = meas->z;          // float     z component of external position measurement
}

void ae483UpdateWithPose(poseMeasurement_t *meas)
{
  // This function will be called each time you send an external "pose" measurement
  // (position as x, y, z and orientation as quaternion) from the client, e.g., from
  // a motion capture system. You will have to write code to handle these measurements.
  // These data are available:
  //
  old_x = x;            // Store previous x component of external position measurement
  old_y = y;            // Store previous x component of external position measurement
  old_z = z;            // Store previous x component of external position measurement
  float new_ts = (float)usecTimestamp();  // Get current timestamp in nanoseconds
  dt_pose = (float)(new_ts - old_ts) / 1000000.0f; // Time difference between current and previous time stamp in seconds
  old_ts = new_ts;      // Store current timestamp as previous timestamp
  x = meas->x;          // float     x component of external position measurement
  y = meas->y;          // float     y component of external position measurement
  z = meas->z;          // float     z component of external position measurement
  qx = meas->quat.x;    // float     x component of quaternion from external orientation measurement
  qy = meas->quat.y;    // float     y component of quaternion from external orientation measurement
  qz = meas->quat.z;    // float     z component of quaternion from external orientation measurement
  qw = meas->quat.w;    // float     w component of quaternion from external orientation measurement
}

void ae483UpdateWithData(const struct AE483Data* data)
{
  // This function will be called each time AE483-specific data are sent
  // from the client to the drone. You will have to write code to handle
  // these data. For the example AE483Data struct, these data are:
  //
  //  data->x         float
  //  data->y         float
  //  data->z         float
  //
  // Exactly what "x", "y", and "z" mean in this context is up to you.
}


void controllerAE483Init(void)
{
  // Do nothing
}

bool controllerAE483Test(void)
{
  // Do nothing (test is always passed)
  return true;
}

void controllerAE483(control_t *control,
                     setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Whatever is in here executes at 500 Hz

    // Desired position
    o_x_des = setpoint->position.x;
    o_y_des = setpoint->position.y;
    o_z_des = setpoint->position.z;

    // Measurements
    w_x = radians(sensors->gyro.x);
    w_y = radians(sensors->gyro.y);
    w_z = radians(sensors->gyro.z);
    a_z = g * sensors->acc.z;
    n_x = flow_dpixelx;
    n_y = flow_dpixely;
    r = tof_distance;

    if (reset_observer) {
      o_x = 0.0f;
      o_y = 0.0f;
      o_z = 0.0f;
      psi = 0.0f;
      theta = 0.0f;
      phi = 0.0f;
      v_x = 0.0f;
      v_y = 0.0f;
      v_z = 0.0f;
      reset_observer = false;
    }
    // State estimates
    if (use_observer == 1) {
        // AE483 observer
        n_x_err = (k_flow * ((v_x / o_z_eq) - w_y)) - n_x;
        n_y_err = (k_flow * ((v_y / o_z_eq) + w_x)) - n_y;
        r_err = o_z - r;
        
        o_x += dt * (v_x);   
        o_y += dt * (v_y);   
        o_z += dt * (v_z - 19.761073f*(r_err));   
        psi += dt * (w_z);   
        theta += dt * (w_y - (0.014907f*n_x_err) ); 
        phi += dt * (w_x - (-0.012128f*n_y_err));   
        v_x += dt * (g*theta - (0.209124f*n_x_err));   
        v_y += dt * (-g*phi - (0.218372f*n_y_err));   
        v_z += dt * ((a_z - g) - (97.250000f*r_err)); 
    
    } if (use_observer == 0) {
      // Stock complementary filter
      o_x = state->position.x;
      o_y = state->position.y;
      o_z = state->position.z;
      psi = radians(state->attitude.yaw);
      theta = - radians(state->attitude.pitch);
      phi = radians(state->attitude.roll);
      v_x = state->velocity.x;
      v_y = state->velocity.y;
      v_z = state->velocity.z;
    }
    if (use_observer == 2) {
      // OptiTrack observer

      // Get State position directly from OptiTrack
      o_x = x;
      o_y = y;
      o_z = z;
      // Convert quaterion to (roll, pitch, yaw) Euler angles using Tait-Bryan convention
      // (yaw, then pitch about new pitch axis, then roll about new roll axis)
	    psi = atan2f(2.0f * (qw * qz + qx * qy), 1 - 2 * (fsqr(qy) + fsqr(qz))); // yaw
      theta = asinf(2.0f * (qw * qy - qx * qz)); // pitch
      phi = atan2f(2.0f * (qw * qx + qy * qz), 1 - 2 * (fsqr(qx) + fsqr(qy))); // roll
      // Get x, y, z velocities based on current and previous position 
      // and time difference between the two
      v_x = (x - old_x) / dt_pose;
      v_y = (y - old_y) / dt_pose;
      v_z = (z - old_z) / dt_pose;
    }
   
    
    // Parse measurements
    n_x = flow_dpixelx;
    n_y = flow_dpixely;
    r = tof_distance;
    a_z = 9.81f * sensors->acc.z;

    if (setpoint->mode.z == modeDisable) {
      // If there is no desired position, then all
      // motor power commands should be zero

      powerSet(0, 0, 0, 0);
    } else {
      // Otherwise, motor power commands should be
      // chosen by the controller

      // FIXME
      tau_x = 0.00323942f * (o_y - o_y_des) -0.00540897f * phi + 0.00199800f * v_y -0.00061004f * w_x;
      tau_y = -0.00250925f * (o_x - o_x_des) -0.00537065f * theta -0.00194855f * v_x -0.00060909f * w_y;
      tau_z = -0.00046032f * psi -0.00015843f * w_z;
      f_z = -0.21596153f * (o_z - o_z_des) -0.12617571f * v_z + 0.31392000f;
        
      m_1 = limitUint16( -3968254.0f * tau_x -3968254.0f * tau_y -116822429.9f * tau_z + 119047.6f * f_z );
      m_2 = limitUint16( -3968254.0f * tau_x + 3968254.0f * tau_y + 116822429.9f * tau_z + 119047.6f * f_z );
      m_3 = limitUint16( 3968254.0f * tau_x + 3968254.0f * tau_y -116822429.9f * tau_z + 119047.6f * f_z );
      m_4 = limitUint16( 3968254.0f * tau_x -3968254.0f * tau_y + 116822429.9f * tau_z + 119047.6f * f_z );
      
      // Apply motor power commands
      powerSet(m_1, m_2, m_3, m_4);
  }

  // if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
  //   // Whatever is in here executes at 100 Hz

  // }

  // By default, we do nothing (set all motor power commands to zero)
  // powerSet(0, 0, 0, 0);
  }
}

//              1234567890123456789012345678 <-- max total length
//              group   .name
// LOG_GROUP_START(ae483log)
// LOG_ADD(LOG_UINT16,         num_tof,                &tof_count)
// LOG_ADD(LOG_UINT16,         num_flow,               &flow_count)
// LOG_GROUP_STOP(ae483log)

// //                1234567890123456789012345678 <-- max total length
// //                group   .name
// PARAM_GROUP_START(ae483par)
// PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
// PARAM_GROUP_STOP(ae483par)
