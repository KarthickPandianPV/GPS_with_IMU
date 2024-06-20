#include "time.h"
#include "vec3f.hpp"
#include "xlsxwriter.h"

namespace gps_with_imu {
class Datalogger {
 private:
  lxw_workbook *workbook;
  lxw_worksheet *worksheet;
  lxw_format *format_bold;
  lxw_format *format_txt_clr;
  time_t rawtime;
  struct tm *timeinfo;
  char date_string[30];
  char time_string[30];
  char type[10] = ".xlsx";
  long row = 1;

 public:
  Datalogger(/* args */);
  ~Datalogger();
  void initialize();
  void write_data(vec3f magnetic_field, vec3f acceleration, vec3f velocity,
                  vec3f distance);
};

}  // namespace gps_with_imu