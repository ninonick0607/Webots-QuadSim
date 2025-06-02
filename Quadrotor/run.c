#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/display.h>

int main() {
  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);
  const int width = wb_camera_get_width(camera);
  const int height = wb_camera_get_height(camera);
  WbDeviceTag display = wb_robot_get_device("display");

  while(wb_robot_step(time_step) != -1) {
    const unsigned char *data = wb_camera_get_image(camera);
    if (data) {
      WbImageRef ir = wb_display_image_new(display, width, height, data, WB_IMAGE_ARGB);
      wb_display_image_paste(display, ir, 0, 0, false);
      wb_display_image_delete(display, ir);
    }
  }

  wb_robot_cleanup();
  return 0;
}