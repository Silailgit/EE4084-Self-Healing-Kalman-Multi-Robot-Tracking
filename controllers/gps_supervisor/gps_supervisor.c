#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <webots/emitter.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 64
#define NOISE_STD 0.02
#define ROBOT_HEIGHT -0.000227
#define SPEED 0.04
#define ARRIVAL_THRESHOLD 0.08
#define NUM_WAYPOINTS 4

double random_noise() {
  return NOISE_STD * ((double)rand() / RAND_MAX * 2.0 - 1.0);
}

double distance2d(double x1, double y1, double x2, double y2) {
  double dx = x1 - x2;
  double dy = y1 - y2;
  return sqrt(dx * dx + dy * dy);
}

void move_towards(double p[3], double target_x, double target_y, double dt) {
  double dx = target_x - p[0];
  double dy = target_y - p[1];
  double d = sqrt(dx * dx + dy * dy);

  if (d > ARRIVAL_THRESHOLD) {
    p[0] += SPEED * dx / d * dt;
    p[1] += SPEED * dy / d * dt;
  }

  p[2] = ROBOT_HEIGHT;
}

void log_target(FILE *file, double time, const char *name, const double *pos) {
  double meas_x = pos[0] + random_noise();
  double meas_y = pos[1] + random_noise();

  printf("%s TRUE: %.3f %.3f | MEAS: %.3f %.3f\n",
         name, pos[0], pos[1], meas_x, meas_y);

  fprintf(file, "%.3f,%s,%.6f,%.6f,%.6f,%.6f\n",
          time, name, pos[0], pos[1], meas_x, meas_y);
}

int main() {
printf("GPS SUPERVISOR STARTED\n");
fflush(stdout);
  wb_robot_init();
  srand(time(NULL));

  WbDeviceTag emitter = wb_robot_get_device("emitter");

  FILE *file = fopen("tracking_log.csv", "w");
  fprintf(file, "time,target,true_x,true_z,meas_x,meas_z\n");

  WbNodeRef target1 = wb_supervisor_node_get_from_def("TARGET_1");
  WbNodeRef target2 = wb_supervisor_node_get_from_def("TARGET_2");

  WbFieldRef t1_translation =
    wb_supervisor_node_get_field(target1, "translation");
  WbFieldRef t2_translation =
    wb_supervisor_node_get_field(target2, "translation");

  double route1[NUM_WAYPOINTS][2] = {
    {-0.60, -0.50},
    { 0.35,  0.25},
    {-0.60,  0.50},
    { 0.35,  0.25}
  };

  double route2[NUM_WAYPOINTS][2] = {
    { 0.60,  0.50},
    { 0.35,  0.25},
    { 0.60, -0.50},
    { 0.35,  0.25}
  };

  double p1[3] = {-0.60, -0.50, ROBOT_HEIGHT};
  double p2[3] = { 0.60,  0.50, ROBOT_HEIGHT};

  int r1_index = 1;
  int r2_index = 1;

  wb_supervisor_field_set_sf_vec3f(t1_translation, p1);
  wb_supervisor_field_set_sf_vec3f(t2_translation, p2);

  while (wb_robot_step(TIME_STEP) != -1) {
    double time = wb_robot_get_time();
    double dt = TIME_STEP / 1000.0;

    double r1_target_x = route1[r1_index][0];
    double r1_target_y = route1[r1_index][1];

    double r2_target_x = route2[r2_index][0];
    double r2_target_y = route2[r2_index][1];

    move_towards(p1, r1_target_x, r1_target_y, dt);
    move_towards(p2, r2_target_x, r2_target_y, dt);

    if (distance2d(p1[0], p1[1], r1_target_x, r1_target_y) < ARRIVAL_THRESHOLD) {
      r1_index = (r1_index + 1) % NUM_WAYPOINTS;
      printf("TARGET_1 reached waypoint, next waypoint: %d\n", r1_index);
    }

    if (distance2d(p2[0], p2[1], r2_target_x, r2_target_y) < ARRIVAL_THRESHOLD) {
      r2_index = (r2_index + 1) % NUM_WAYPOINTS;
      printf("TARGET_2 reached waypoint, next waypoint: %d\n", r2_index);
    }

    wb_supervisor_field_set_sf_vec3f(t1_translation, p1);
    wb_supervisor_field_set_sf_vec3f(t2_translation, p2);

    const double *r1 = wb_supervisor_field_get_sf_vec3f(t1_translation);
    const double *r2 = wb_supervisor_field_get_sf_vec3f(t2_translation);

    printf("T=%.2f\n", time);
    log_target(file, time, "TARGET_1", r1);
    log_target(file, time, "TARGET_2", r2);
    printf("----------------------\n");

    fflush(stdout);
    fflush(file);

    wb_emitter_send(emitter, r1, 3 * sizeof(double));
  }

  fclose(file);
  wb_robot_cleanup();

  return 0;
}