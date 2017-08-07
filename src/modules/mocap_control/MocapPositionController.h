#ifndef MOCAP_POSITION_CONTROLLER_H
#define MOCAP_POSITION_CONTROLLER_H

#include <cstdio>
#include <cstdint>
#include <uORB/uORB.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/mocap_position_command.h>
#include <uORB/topics/mocap_position_command_gains.h>

#include "ParameterUtils.h"
#include "L1PositionObserver.h"

class MocapPositionController
{
public:
  MocapPositionController() :
    ctrl_state_set(false),
    local_pos_set(false),
    cmd_time(0)
  {
    e3.zero();
    e3(2) = 1.0f;
    gravity.zero();
    current_rpm_cmd.zero();
  }
  ~MocapPositionController() { }

  bool initialize()
  {
    if (!loadParameters())
    {
      puts("[MPC] failed to load parameters");
      return false;
    }

    if (!registerCallbacks())
    {
      puts("[MPC] failed to register callbacks");
      return false;
    }

    l1_pos_observer.loadParameters();

    return true;
  }

  bool update()
  {
    bool cmd_updated = false;
    bool updated;

    /* orb_check signifying: check to see whether the topic has updated since the last time we read it */
    orb_check(local_position_sub, &updated);
    if (updated)
    {  
      //printf("updated local pos\n");   
      struct vehicle_local_position_s local_position; // Making a structure and ready to keep the info inside 'local_position'
      orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position); // Copying the contents of ORB_ID: 'vehicle_local_position' using handle 'local_position_sub' inside 'local_position'
      localPositionMessageCallback(local_position);  // Got the local_position setted through this. Look at the function declaration below for more clear understanding
    }

    orb_check(mocap_position_cmd_sub, &updated); // mocap_position_cmd_sub is a handle of original topic and &updated would be using that value from here on
    if (updated)
    {
      //printf("pos_cmd is received\n");
      struct mocap_position_command_s cmd_in;
      orb_copy(ORB_ID(mocap_position_command), mocap_position_cmd_sub, &cmd_in); // Means now cmd_in.('anything here inside original msg file'). Now cmd_in. can access the contents inside the msg file now
      cmd_updated = mocapPositionCommandMessageCallback(cmd_in);
    }

    orb_check(mocap_position_cmd_gains_sub, &updated);
    if (updated)
    {
      //printf("mocap_position_command_gains is received\n");
      struct mocap_position_command_gains_s cmd_in;
      orb_copy(ORB_ID(mocap_position_command_gains), mocap_position_cmd_gains_sub, &cmd_in);
      mocapPositionCommandGainsMessageCallback(cmd_in); // Fill and retrieve the contents/values of mocap_position_command_gains
    }

    hrt_abstime tnow = hrt_absolute_time();

    if (cmd_updated)
      cmd_time = tnow;


    if (tnow - cmd_time > cmd_ttl_us)
      return false;

    //printf("until here??\n");
    return updatePositionController();
  }

  void finalize()
  {
    closeSubscriptions(); // Used in the main loop at the end disabling all controllers
  }

  void setControlState(const control_state_s& ctrl_state_) // making a structure out of control_state and using it in ctrl_state_
  {
    //printf("inside control state\n");
    memcpy(&ctrl_state, &ctrl_state_, sizeof(ctrl_state_)); // & copying the contents of newly created ctrl_state_ inside ctrl_state
    ctrl_state_set = true; // Making an integral condition true of: That the control_state is set to true now
  }

  void setCurrentRPMCommand(const MotorManager::rpm_cmd_t& in) // Called from main loop
  {
    for (unsigned int i = 0; i < 4; i++)
      current_rpm_cmd(i) = in.motor[i];
    printf("Got rpm cmds\n");

  }

  void getCascadedAttitudeCommand(cascaded_attitude_command_t& out) // Called from main loop. Using the value of 'att_cmd' generated in this code
  {
    out = att_cmd;
    //printf("got cascaded attitude commands\n");
  }

private: // Private as all these handles and objects are used inclusively by this code unlike Public where main code was able to use/access them
  void closeSubscriptions() // closing all ORB_ID Subscriptions using their respective handles
  {
    close(mocap_position_cmd_sub);
    close(mocap_position_cmd_gains_sub);
    close(local_position_sub);
  }

  bool loadParameters() // Loading internal Parameters used by MocapPositionController code. We can include our EPC internal params here later
  {
    cmd_ttl_us =
      static_cast<hrt_abstime>(parameter_utils::getFloatParam("MCC_CMD_TTL")*1.0e6f);
    mass = parameter_utils::getFloatParam("MCC_TOTAL_MASS");
    gravity(2) = parameter_utils::getFloatParam("MCC_GRAVITY");

    return true;
  }

  bool registerCallbacks() // Literally, registering Callbacks, means, we are confirming here whether the handle created by us is actually subscribing and filling itself with data or not
  {
    //puts("GH");
    mocap_position_cmd_sub = orb_subscribe(ORB_ID(mocap_position_command));  // mocap_position_cmd_sub is a handle. Original ID from which we are fetching data is: 'mocap_position_command'
    if (mocap_position_cmd_sub < 0) // Checking if handle got data or not
    {
      puts("[MPC] mocap_position_cmd_sub failed");
      return false;
    }
    
    mocap_position_cmd_gains_sub = orb_subscribe(ORB_ID(mocap_position_command_gains)); // Similar cycle as done for mocap_position_cmd would be done for mocap_position_cmd_gains
    if (mocap_position_cmd_gains_sub < 0)
    {
      puts("[MPC] mocap_position_cmd_gains_sub failed");
      return false;
    }

    // Main ID: vehicle_local_position
    // Topic handle: local_position_sub


    local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position)); // Creating topic handle for vehicle_local_position
    if (local_position_sub < 0)
    {
      puts("[MPC] local_position_sub failed");
      return false;
    }

    return true; // This final return true signifies that all registerCallbacks are successfull
  }

  bool updatePositionController() // Main Controller loop | And everything would be generated used here, which would be used for future references
  {

    if (!ctrl_state_set || !local_pos_set) // Confirming that control_state is set and local_position is set
      return false;

    //printf("first test pass\n");
    if (!cmd.cmd_set || !cmd_gains.gains_set) // Confirming inputs from commanded setpoints from mocap and cmd_gains are also set before proceeding further insdie the loop
    {
#if 0
      static unsigned int counter = 0;
      if (counter++ > 100)
      {
        cmd.print();
        cmd_gains.print();
        counter = 0;
      }
#endif
      return false;
    }

    //printf("second test pass\n");

    // Now, until here, we have confirmed that we have local position as well as commanded position setpoints. Read motion_manager architecture accordingly to set these things and check accordingly
    math::Vector<3> pos(local_pos.x, local_pos.y, local_pos.z);
    math::Vector<3> vel(local_pos.vx, local_pos.vy, local_pos.vz);
    math::Quaternion q(ctrl_state.q);
    math::Matrix<3, 3> R = q.to_dcm();
    math::Vector<3> Rde3 = R*e3;

    math::Vector<3> e_pos = pos - cmd.pos;
    math::Vector<3> e_vel = vel - cmd.vel;
    printf("commanded pos: %3.4f\n", double(cmd.pos(0)));
    printf("local pos: %3.4f\n", double(pos(0)));

    //printf("e_vel & e_pos calculated\n");

    // World force in NED frame
    math::Vector<3> fd_w =
      (-cmd_gains.kp.emult(e_pos) - cmd_gains.kd.emult(e_vel) + cmd.acc - gravity)*mass;

    // L1 Position Observe (provides world force error in NED frame)
    math::Vector<3> l1_fw = l1_pos_observer.update(R, vel, current_rpm_cmd);

    //math::Vector<3> l1_fw = 0;
    // Total force
    fd_w -= l1_fw;

    // thrust magnitude (in NED)
    att_cmd.thrust = fd_w(0)*Rde3(0) + fd_w(1)*Rde3(1) + fd_w(2)*Rde3(2);

    //printf("thrust magnitude calculation accomplished\n");

#if 0
    static unsigned int debug_counter1 = 0;
    if (debug_counter1++ > 100)
    {
      puts("e_pos");
      e_pos.print();
      puts("e_vel");
      e_vel.print();
      puts("fd_w");
      fd_w.print();
      puts("l1_fw");
      l1_fw.print();
      debug_counter1 = 0;
    }
#endif

    // Attitude calculation requires the negative
    math::Vector<3> fd_w_att = -fd_w;

    math::Vector<3> b3;
    if (fd_w_att.length_squared() < 1e-5f)
      b3 = e3;
    else
      b3 = fd_w_att.normalized();

    math::Vector<3> c1(cosf(cmd.heading(0)), sinf(cmd.heading(0)), 0.0f);
    // b3 x c1
    math::Vector<3> b2 = b3 % c1;
    b2.normalize();
    // b2 x b3
    math::Vector<3> b1 = b2 % b3;

    math::Matrix<3, 3> desired_rotation;
    desired_rotation.set_col(0, b1);
    desired_rotation.set_col(1, b2);
    desired_rotation.set_col(2, b3);

    att_cmd.q.from_dcm(desired_rotation);
    computeAngularReference(desired_rotation, att_cmd.thrust, e_vel,
                            att_cmd.ang_vel, att_cmd.ang_acc);
    math::Vector<3> desired_euler_zyx = desired_rotation.to_euler();
    computeHeadingReference(desired_euler_zyx, att_cmd.ang_vel, att_cmd.ang_acc);
    att_cmd.cmd_set = true;

#if 0
    static unsigned int debug_counter = 0;
    if (debug_counter++ > 100)
    {
      att_cmd.print();
      puts("desired rotation");
      desired_rotation.print();
      debug_counter = 0;
    }
#endif
    //printf("update position over\n");
    return true;
  }

  void computeHeadingReference(const math::Vector<3>& desired_euler_zyx,
                               math::Vector<3>& omg_des,
                               math::Vector<3>& omg_ddes)
  {
    float phi = desired_euler_zyx(0);
    float theta = desired_euler_zyx(1);

    float cph = cosf(phi);
    float sph = sinf(phi);

    float cth = cosf(theta);
    float sth = sinf(theta);

    float psi_d = cmd.heading(1);
    float psi_dd = cmd.heading(2);

    float p = omg_des(0);
    float q = omg_des(1);
    float q_d = omg_ddes(1);

    if (fabsf(cph) < 1.0e-5f)
    {
      omg_des(2) = 0.0f;
      omg_ddes(2) = 0.0f;
    }
    else
    {
      omg_des(2) = (psi_d*powf(cph,2.0f)*cth - q*sph + psi_d*cth*powf(sph,2.0f))/cph;
      omg_ddes(2) = -(p*q + 2.0f*psi_d*q*sth - psi_dd*cph*cth + q_d*cph*sph - 2.0f*powf(psi_d,2.0f)*cth*sph*sth - p*psi_d*cth*sph)/powf(cph,2.0f);
    }
  }

  void computeAngularReference(const math::Matrix<3,3>& R,
                               const float& thrust_des,
                               const math::Vector<3>& vel_err,
                               math::Vector<3>& omg_des,
                               math::Vector<3>& omg_ddes)
  {
    float R11 = R(0,0);
    float R12 = R(0,1);
    float R13 = R(0,2);
    float R21 = R(1,0);
    float R22 = R(1,1);
    float R23 = R(1,2);
    float R31 = R(2,0);
    float R32 = R(2,1);
    float R33 = R(2,2);
    float Td = fabsf(thrust_des);

    math::Vector<3> acc_err; acc_err.zero();
    math::Vector<3> K =
      (-cmd_gains.kp.emult(vel_err) - cmd_gains.kd.emult(acc_err) + cmd.jerk)*mass;

    float d =
      R11*R22*R33 - R11*R23*R32 - R12*R21*R33 + R12*R23*R31 + R13*R21*R32 - R13*R22*R31;
    float w1 =
      -(K(0)*(R21*R33 - R23*R31) + K(1)*(R13*R31 - R11*R33) + K(2)*(R11*R23 - R13*R21))/(Td*d);
    float w2 =
      -(K(0)*(R22*R33 - R23*R32) + K(1)*(R13*R32 - R12*R33) + K(2)*(R12*R23 - R13*R22))/(Td*d);
    float Tdd =
      -(K(0)*(R21*R32 - R22*R31) + K(1)*(R12*R31 - R11*R32) + K(2)*(R11*R22 - R12*R21))/d;

    omg_des(0) = w1;
    omg_des(1) = w2;
    omg_des(2) = 0.0f;

    omg_ddes(0) = -2.0f*w1*Tdd/Td;
    omg_ddes(1) = -2.0f*w2*Tdd/Td;
    omg_ddes(2) = 0.0f;
  }

  typedef struct CascadedPositionCommand
  {
    math::Vector<3> pos;
    math::Vector<3> vel;
    math::Vector<3> acc;
    math::Vector<3> jerk;
    math::Vector<3> heading;
    bool cmd_set;

    CascadedPositionCommand() :
      cmd_set(false)
    {
      pos.zero();
      vel.zero();
      acc.zero();
      jerk.zero();
      heading.zero();
    }

    void print()
    {
      // Hack to work around NuttX printf float bug
      puts("pos cmd:");
      char buf[128];
      sprintf(buf, "pos = [%0.2f, %0.2f, %0.2f]",
              double(pos(0)), double(pos(1)), double(pos(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "vel = [%0.2f, %0.2f, %0.2f]",
              double(vel(0)), double(vel(1)), double(vel(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "acc = [%0.2f, %0.2f, %0.2f]",
              double(acc(0)), double(acc(1)), double(acc(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "jerk = [%0.2f, %0.2f, %0.2f]",
              double(jerk(0)), double(jerk(1)), double(jerk(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "heading = [%0.2f, %0.2f, %0.2f]",
              double(heading(0)), double(heading(1)), double(heading(2)));
      printf("\t%s\n", buf);
      printf("\tcmd_set = %s\n", cmd_set ? "true" : "false");
    }
  } cascaded_position_command_t;

  typedef struct CascadedPositionCommandGains
  {
    math::Vector<3> kp;
    math::Vector<3> kd;

    bool gains_set;

    CascadedPositionCommandGains() : gains_set(false)
    {
      kp.zero();
      kd.zero();
    }

    void print()
    {
      // Hack to work around NuttX printf float bug
      puts("pos cmd gains:");
      char buf[128];
      sprintf(buf, "kp gains = [%0.2f, %0.2f, %0.2f]",
              double(kp(0)), double(kp(1)), double(kp(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "kd gains = [%0.2f, %0.2f, %0.2f]",
              double(kd(0)), double(kd(1)), double(kd(2)));
      printf("\t%s\n", buf);
      printf("\tgains_set = %s\n", gains_set ? "true" : "false");
    }
  } cascaded_position_command_gains_t;

  bool mocapPositionCommandMessageCallback(const mocap_position_command_s& in)
  {
    if (!cmd_gains.gains_set)
      return false;

    cmd.pos.set(in.pos);
    cmd.vel.set(in.vel);
    cmd.acc.set(in.acc);
    cmd.jerk.set(in.jerk);
    cmd.heading.set(in.heading);
    cmd.cmd_set = true;
    
    return true;
  }

  bool mocapPositionCommandGainsMessageCallback(const mocap_position_command_gains_s& in)
  {
    cmd_gains.kp.set(in.kp);
    cmd_gains.kd.set(in.kd);
    cmd_gains.gains_set = true;

    return true;
  }

  void printLocalPosition()
  {
    // Hack to work around NuttX printf float bug
    puts("local position:");
    char buf[128];
    sprintf(buf, "pos = [%0.2f, %0.2f, %0.2f]",
            double(local_pos.x), double(local_pos.y), double(local_pos.z));
    printf("%s\n", buf);
    sprintf(buf, "vel = [%0.2f, %0.2f, %0.2f]",
            double(local_pos.vx), double(local_pos.vy), double(local_pos.vz));
    printf("%s\n", buf);
  }

  void localPositionMessageCallback(const vehicle_local_position_s& local_pos_)
  {
    memcpy(&local_pos, &local_pos_, sizeof(local_pos_));  // Copied the contents of local_pos_ inside local_pos which we would be using later on, for accessing all the local position values inside the message
    local_pos_set = true;  // Self explanatory

#if 0
    static unsigned int counter = 0;
    if (counter++ > 100)
    {
      printLocalPosition();
      counter = 0;
    }
#endif
  }

  bool ctrl_state_set;
  bool local_pos_set;

  hrt_abstime cmd_time;
  hrt_abstime cmd_ttl_us;

  struct control_state_s ctrl_state;
  struct vehicle_local_position_s local_pos;
  cascaded_attitude_command_t att_cmd;
  cascaded_position_command_t cmd;
  cascaded_position_command_gains_t cmd_gains;

  int mocap_position_cmd_sub;
  int mocap_position_cmd_gains_sub;
  int local_position_sub;

  math::Vector<3> gravity;
  math::Vector<3> e3;
  float mass;

  L1PositionObserver l1_pos_observer;
  math::Vector<4> current_rpm_cmd;
};

#endif
